// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Device Tree machine support for Axera AX620E SoCs.
 */

#include <linux/init.h>
#include <asm/mach/arch.h>

static const char * const axera_dt_compat[] __initconst = {
	"axera,ax620e",
	NULL,
};

DT_MACHINE_START(AXERA_DT, "Axera AX620E (Device Tree)")
	.l2c_aux_val	= 0,
	.l2c_aux_mask	= ~0,
	.dt_compat	= axera_dt_compat,
MACHINE_END
