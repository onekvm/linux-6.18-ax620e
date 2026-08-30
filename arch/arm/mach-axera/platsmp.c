// SPDX-License-Identifier: GPL-2.0-only
/*
 * SMP operations for Axera AX620E (AArch32).
 *
 * Copyright (C) 2002 ARM Ltd.
 */

#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/limits.h>
#include <linux/of.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <asm/cacheflush.h>
#include <asm/smp.h>
#include <asm/smp_plat.h>

static u32 cpus_release_paddr;
static bool cpus_inited;
static DEFINE_SPINLOCK(boot_lock);

volatile int axera_pen_release = -1;

extern void secondary_holding_pen(void);

static void axera_write_pen_release(int val)
{
	axera_pen_release = val;
	smp_wmb();
	sync_cache_w(&axera_pen_release);
}

static void ax620x_smp_init_cpus(void)
{
	struct device_node *cpus_node;

	cpus_node = of_find_node_by_path("/cpus");
	if (!cpus_node) {
		pr_err("No CPU information found in DT\n");
		return;
	}

	if (of_property_read_u32(cpus_node, "secondary-boot-reg",
				 &cpus_release_paddr)) {
		pr_err("required secondary release register not specified for cpus\n");
		of_node_put(cpus_node);
		return;
	}

	of_node_put(cpus_node);
	cpus_inited = true;
}

static void ax620x_smp_prepare_cpus(unsigned int max_cpus)
{
	void __iomem *cpus_release_vaddr;
	phys_addr_t secondary_startup_paddr;

	if (!cpus_inited)
		return;

	cpus_release_vaddr = ioremap((phys_addr_t)cpus_release_paddr,
				     sizeof(u32));
	if (!cpus_release_vaddr) {
		cpus_inited = false;
		pr_err("unable to ioremap secondary_release register for cpus\n");
		return;
	}

	secondary_startup_paddr = __pa_symbol(secondary_holding_pen);
	if (WARN_ON(secondary_startup_paddr > (phys_addr_t)U32_MAX)) {
		iounmap(cpus_release_vaddr);
		cpus_inited = false;
		return;
	}

	writel_relaxed(secondary_startup_paddr, cpus_release_vaddr);
	dsb_sev();

	iounmap(cpus_release_vaddr);
}

static int ax620x_smp_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;

	if (!cpus_inited)
		return -ENOSYS;

	spin_lock(&boot_lock);

	axera_write_pen_release(cpu_logical_map(cpu));
	arch_send_wakeup_ipi_mask(cpumask_of(cpu));

	timeout = jiffies + (2 * HZ);
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (axera_pen_release == -1)
			break;
		udelay(10);
	}

	spin_unlock(&boot_lock);

	return axera_pen_release != -1 ? -ENOSYS : 0;
}

static void ax620x_secondary_init(unsigned int cpu)
{
	axera_write_pen_release(-1);
}

#ifdef CONFIG_HOTPLUG_CPU
#include <asm/cp15.h>

static inline void cpu_enter_lowpower(void)
{
	unsigned int v;

	asm volatile(
		"mcr	p15, 0, %1, c7, c5, 0\n"
	"	mcr	p15, 0, %1, c7, c10, 4\n"
	"	mrc	p15, 0, %0, c1, c0, 1\n"
	"	bic	%0, %0, %3\n"
	"	mcr	p15, 0, %0, c1, c0, 1\n"
	"	mrc	p15, 0, %0, c1, c0, 0\n"
	"	bic	%0, %0, %2\n"
	"	mcr	p15, 0, %0, c1, c0, 0\n"
	  : "=&r" (v)
	  : "r" (0), "Ir" (CR_C), "Ir" (0x40)
	  : "cc");
}

static inline void cpu_leave_lowpower(void)
{
	unsigned int v;

	asm volatile(
		"mrc	p15, 0, %0, c1, c0, 0\n"
	"	orr	%0, %0, %1\n"
	"	mcr	p15, 0, %0, c1, c0, 0\n"
	"	mrc	p15, 0, %0, c1, c0, 1\n"
	"	orr	%0, %0, %2\n"
	"	mcr	p15, 0, %0, c1, c0, 1\n"
	  : "=&r" (v)
	  : "Ir" (CR_C), "Ir" (0x40)
	  : "cc");
}

static inline void platform_do_lowpower(unsigned int cpu, int *spurious)
{
	for (;;) {
		wfi();
		if (axera_pen_release == cpu_logical_map(cpu))
			break;
		(*spurious)++;
	}
}

static void ax620x_cpu_die(unsigned int cpu)
{
	int spurious = 0;

	cpu_enter_lowpower();
	platform_do_lowpower(cpu, &spurious);
	cpu_leave_lowpower();

	if (spurious)
		pr_warn("CPU%u: %u spurious wakeup calls\n", cpu, spurious);
}
#endif

static const struct smp_operations ax620x_smp_ops = {
	.smp_init_cpus		= ax620x_smp_init_cpus,
	.smp_prepare_cpus	= ax620x_smp_prepare_cpus,
	.smp_boot_secondary	= ax620x_smp_boot_secondary,
	.smp_secondary_init	= ax620x_secondary_init,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= ax620x_cpu_die,
#endif
};

CPU_METHOD_OF_DECLARE(axera_smp_ax620x, "axera,ax620e-smp", &ax620x_smp_ops);
