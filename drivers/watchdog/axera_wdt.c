// SPDX-License-Identifier: GPL-2.0-only
/* AX620E/AX630C watchdog. Register protocol follows Axera's GPL BSP driver.
 * This is not a DesignWare watchdog: the reload latch and kick differ.
 * Expiry is left to hardware; no interrupt handler feeds a stalled system.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/watchdog.h>

#define AX_WDT_ENABLE		0x00
#define AX_WDT_RELOAD		0x0c
#define AX_WDT_RELOAD_LATCH	0x18
#define AX_WDT_KICK		0x30
#define AX_WDT_IRQ_CLEAR		0x3c
#define AX_WDT_IRQ_ENABLE	0x54
#define AX_WDT_KICK_VALUE	0x61696370
#define AX_WDT_MAX_RELOAD	0xffffU
#define AX_WDT_RELOAD_SHIFT	16
/* First expiry latches an interrupt and reloads, second expiry resets.
 * Masking the interrupt does not skip the first countdown.
 */
#define AX_WDT_STAGES		2U

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0444);
MODULE_PARM_DESC(nowayout, "Prevent stopping an armed watchdog");

struct axera_wdt {
	void __iomem *regs;
	struct clk *tclk;
	struct clk *pclk;
	unsigned long rate;
	unsigned int pulse_us;
	struct watchdog_device wdd;
};

static int axera_wdt_ping(struct watchdog_device *wdd)
{
	struct axera_wdt *wdt = watchdog_get_drvdata(wdd);

	writel(1, wdt->regs + AX_WDT_IRQ_CLEAR);
	writel(AX_WDT_KICK_VALUE, wdt->regs + AX_WDT_KICK);
	udelay(wdt->pulse_us);
	writel(0, wdt->regs + AX_WDT_KICK);
	return 0;
}

static void axera_wdt_reload(struct axera_wdt *wdt, u32 reload)
{
	writel(reload, wdt->regs + AX_WDT_RELOAD);
	writel(1, wdt->regs + AX_WDT_RELOAD_LATCH);
	udelay(wdt->pulse_us);
	writel(0, wdt->regs + AX_WDT_RELOAD_LATCH);
}

static int axera_wdt_set_timeout(struct watchdog_device *wdd, unsigned int seconds)
{
	struct axera_wdt *wdt = watchdog_get_drvdata(wdd);
	u32 reload = DIV_ROUND_UP_ULL((u64)seconds * wdt->rate,
				    AX_WDT_STAGES << AX_WDT_RELOAD_SHIFT);

	if (!reload || reload > AX_WDT_MAX_RELOAD)
		return -EINVAL;
	axera_wdt_reload(wdt, reload);
	/* The reload rounds up in hardware quanta; report its whole seconds.
	 * At 24 MHz the fractional quantum must not grow the next SETTIMEOUT.
	 */
	wdd->timeout = div_u64((u64)reload * AX_WDT_STAGES << AX_WDT_RELOAD_SHIFT,
			       wdt->rate);
	return axera_wdt_ping(wdd);
}

static int axera_wdt_start(struct watchdog_device *wdd)
{
	struct axera_wdt *wdt = watchdog_get_drvdata(wdd);
	int ret = axera_wdt_set_timeout(wdd, wdd->timeout);

	if (ret)
		return ret;
	writel(0, wdt->regs + AX_WDT_IRQ_ENABLE);
	writel(1, wdt->regs + AX_WDT_IRQ_CLEAR);
	writel(1, wdt->regs + AX_WDT_ENABLE);
	return axera_wdt_ping(wdd);
}

static int axera_wdt_stop(struct watchdog_device *wdd)
{
	struct axera_wdt *wdt = watchdog_get_drvdata(wdd);

	writel(0, wdt->regs + AX_WDT_ENABLE);
	/* Cross the timer clock domain before releasing clocks or returning. */
	udelay(wdt->pulse_us);
	readl(wdt->regs + AX_WDT_ENABLE);
	return 0;
}

static int axera_wdt_restart(struct watchdog_device *wdd,
			     unsigned long action, void *data)
{
	struct axera_wdt *wdt = watchdog_get_drvdata(wdd);

	writel(0, wdt->regs + AX_WDT_IRQ_ENABLE);
	axera_wdt_reload(wdt, 1);
	writel(1, wdt->regs + AX_WDT_ENABLE);
	axera_wdt_ping(wdd);
	/* Both countdowns take at most 4.096 seconds on the 32 kHz clock. */
	mdelay(4200);
	return 0;
}

static const struct watchdog_info axera_wdt_info = {
	.identity = "Axera AX620E watchdog",
	.options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops axera_wdt_ops = {
	.owner = THIS_MODULE,
	.start = axera_wdt_start,
	.stop = axera_wdt_stop,
	.ping = axera_wdt_ping,
	.set_timeout = axera_wdt_set_timeout,
	/* The counter reloads between stages, so it cannot report timeleft. */
	.restart = axera_wdt_restart,
};

static void axera_wdt_disable_clocks(void *data)
{
	struct axera_wdt *wdt = data;

	clk_disable_unprepare(wdt->tclk);
	clk_disable_unprepare(wdt->pclk);
}

static int axera_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *resets;
	struct watchdog_device *wdd;
	struct axera_wdt *wdt;
	u32 reload;
	int ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;
	wdt->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(wdt->regs))
		return PTR_ERR(wdt->regs);
	wdt->tclk = devm_clk_get(dev, "tclk");
	if (IS_ERR(wdt->tclk))
		return dev_err_probe(dev, PTR_ERR(wdt->tclk), "timer clock\n");
	wdt->pclk = devm_clk_get(dev, "pclk");
	if (IS_ERR(wdt->pclk))
		return dev_err_probe(dev, PTR_ERR(wdt->pclk), "bus clock\n");
	resets = devm_reset_control_array_get_exclusive(dev);
	if (IS_ERR(resets))
		return dev_err_probe(dev, PTR_ERR(resets), "resets\n");
	ret = clk_prepare_enable(wdt->pclk);
	if (ret)
		return ret;
	ret = clk_prepare_enable(wdt->tclk);
	if (ret) {
		clk_disable_unprepare(wdt->pclk);
		return ret;
	}
	ret = devm_add_action_or_reset(dev, axera_wdt_disable_clocks, wdt);
	if (ret)
		return ret;
	wdt->rate = clk_get_rate(wdt->tclk);
	if (wdt->rate < 32000 || wdt->rate > 24000000)
		return dev_err_probe(dev, -EINVAL, "unsupported timer rate %lu\n", wdt->rate);
	/* Axera specifies a 40 us latch pulse for the slow clock domain. */
	wdt->pulse_us = wdt->rate < 1000000 ? 40 : 1;
	/* Do not assert resets or change a firmware-enabled watchdog's rate. */
	ret = reset_control_deassert(resets);
	if (ret)
		return ret;
	udelay(100);
	wdd = &wdt->wdd;
	wdd->parent = dev;
	wdd->info = &axera_wdt_info;
	wdd->ops = &axera_wdt_ops;
	wdd->min_timeout = DIV_ROUND_UP(AX_WDT_STAGES << AX_WDT_RELOAD_SHIFT,
				       wdt->rate);
	wdd->max_timeout = div_u64((u64)AX_WDT_MAX_RELOAD * AX_WDT_STAGES << AX_WDT_RELOAD_SHIFT,
				   wdt->rate);
	wdd->max_hw_heartbeat_ms = wdd->max_timeout * 1000U;
	wdd->timeout = 60;
	watchdog_set_drvdata(wdd, wdt);
	watchdog_set_nowayout(wdd, nowayout);
	if (readl(wdt->regs + AX_WDT_ENABLE) & 1) {
		reload = readl(wdt->regs + AX_WDT_RELOAD) & AX_WDT_MAX_RELOAD;
		if (!reload)
			return dev_err_probe(dev, -EINVAL, "running watchdog has no reload value\n");
		wdd->timeout = div_u64((u64)reload * AX_WDT_STAGES << AX_WDT_RELOAD_SHIFT,
					     wdt->rate);
		set_bit(WDOG_HW_RUNNING, &wdd->status);
		axera_wdt_ping(wdd);
	}
	watchdog_init_timeout(wdd, 0, dev);
	if (watchdog_hw_running(wdd)) {
		ret = axera_wdt_set_timeout(wdd, wdd->timeout);
		if (ret)
			return ret;
	}
	writel(0, wdt->regs + AX_WDT_IRQ_ENABLE);
	writel(1, wdt->regs + AX_WDT_IRQ_CLEAR);
	watchdog_stop_on_reboot(wdd);
	watchdog_stop_on_unregister(wdd);
	watchdog_set_restart_priority(wdd, 128);
	ret = devm_watchdog_register_device(dev, wdd);
	if (ret)
		return ret;
	dev_info(dev, "%lu Hz hardware watchdog, timeout %u s, nowayout %u\n",
		 wdt->rate, wdd->timeout, nowayout);
	return 0;
}

static const struct of_device_id axera_wdt_of_match[] = {
	{ .compatible = "axera,ax620e-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, axera_wdt_of_match);

static struct platform_driver axera_wdt_driver = {
	.probe = axera_wdt_probe,
	.driver = {
		.name = "axera-wdt",
		.of_match_table = axera_wdt_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(axera_wdt_driver);
MODULE_DESCRIPTION("Axera AX620E/AX630C hardware watchdog");
MODULE_LICENSE("GPL");
