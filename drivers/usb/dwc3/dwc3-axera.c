// SPDX-License-Identifier: GPL-2.0-only
/*
 * Axera AX620E DWC3 glue: enable USB2 clocks/resets then populate snps,dwc3.
 *
 * Register offsets from the public maix_ax620e_sdk 4.19 dwc3-axera driver.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include <linux/usb/role.h>

#include "core.h"

#define FLASH_SYS_GLB_BASE		0x10030000
#define FLASH_CLK_MUX0_SET		0x4000
#define FLASH_CLK_MUX0_CLR		0x8000
#define FLASH_CLK_EB0_SET		0x4004
#define FLASH_CLK_EB0_CLR		0x8004
#define FLASH_CLK_EB1_SET		0x4008
#define FLASH_CLK_EB1_CLR		0x8008
#define FLASH_SW_RST0_SET		0x4014
#define FLASH_SW_RST0_CLR		0x8014

#define USB2_PHY_SW_RST			24
#define USB2_VCC_SW_RST			25
#define CLK_USB2_REF_EB			12
#define CLK_USB2_REF_ALT_CLK_EB		14
#define CLK_BUS_CLK_USB2_EB		5
#define FLASH_USB2_CTRL_SET		0x4040
#define FLASH_USB2_CTRL_CLR		0x8040
#define USB2_VBUSVALID			6

/* Vendor dwc3-axera.c axera_usb_adj_ref_clk(), called from dwc3 core init. */
#define DWC3_GUCTL			0xc12c
#define DWC3_GFLADJ			0xc630
#define DWC3_GFLADJ_REFCLK_FLADJ_MASK	GENMASK(21, 8)
#define DWC3_GFLADJ_REFCLK_LPM_SEL	BIT(23)
#define DWC3_GFLADJ_240MHZDECR		GENMASK(30, 24)
#define DWC3_GUCTL_REFCLKPER_MASK	0xffc00000
#define AXERA_GFLADJ_FLADJ		0x7f0
#define AXERA_GFLADJ_240MHZ_DECR	0xa
#define AXERA_GUCTL_REFCLKPER		0x29

struct dwc3_axera {
	struct device *dev;
	void __iomem *sys;
	bool default_device;
};

static void axera_usb_writel(struct dwc3_axera *ax, u32 off, u32 val)
{
	writel(val, ax->sys + off);
}

static void axera_usb_clk_enable(struct dwc3_axera *ax)
{
	axera_usb_writel(ax, FLASH_CLK_EB0_SET,
			 BIT(CLK_USB2_REF_EB) | BIT(CLK_USB2_REF_ALT_CLK_EB));
	axera_usb_writel(ax, FLASH_CLK_EB1_SET, BIT(CLK_BUS_CLK_USB2_EB));

	/* bus_clk mux bits 8:6 = 101b → 312 MHz */
	axera_usb_writel(ax, FLASH_CLK_MUX0_CLR, BIT(6) | BIT(7) | BIT(8));
	axera_usb_writel(ax, FLASH_CLK_MUX0_SET, BIT(6) | BIT(8));
}

static void axera_usb_clk_disable(struct dwc3_axera *ax)
{
	axera_usb_writel(ax, FLASH_CLK_EB0_CLR,
			 BIT(CLK_USB2_REF_EB) | BIT(CLK_USB2_REF_ALT_CLK_EB));
	axera_usb_writel(ax, FLASH_CLK_EB1_CLR, BIT(CLK_BUS_CLK_USB2_EB));
}

static void axera_usb_reset(struct dwc3_axera *ax, bool assert)
{
	u32 bits = BIT(USB2_PHY_SW_RST) | BIT(USB2_VCC_SW_RST);

	if (assert)
		axera_usb_writel(ax, FLASH_SW_RST0_SET, bits);
	else
		axera_usb_writel(ax, FLASH_SW_RST0_CLR, bits);
}

static void axera_usb_set_device_mode(struct dwc3_axera *ax, bool device)
{
	axera_usb_writel(ax, device ? FLASH_USB2_CTRL_SET : FLASH_USB2_CTRL_CLR,
			 BIT(USB2_VBUSVALID));
}

/*
 * dwc3_setup_role_switch() treats anything other than an explicit "host"
 * default as peripheral. Match that before the child is probed, because
 * the first dwc3_set_mode() runs inside the child probe.
 */
static bool axera_child_starts_as_device(struct device_node *glue)
{
	struct device_node *child;
	const char *dr_mode = NULL;
	const char *role = NULL;

	child = of_get_next_child(glue, NULL);
	if (!child)
		return false;

	of_property_read_string(child, "dr_mode", &dr_mode);
	of_property_read_string(child, "role-switch-default-mode", &role);
	of_node_put(child);

	if (dr_mode && !strcmp(dr_mode, "host"))
		return false;
	if (dr_mode && !strcmp(dr_mode, "peripheral"))
		return true;
	if (role && !strcmp(role, "host"))
		return false;
	return dr_mode && !strcmp(dr_mode, "otg");
}

static void axera_pre_set_role(struct dwc3 *dwc, enum usb_role role)
{
	struct dwc3_axera *ax;
	bool device;

	if (!dwc->dev->parent)
		return;
	ax = dev_get_drvdata(dwc->dev->parent);
	if (!ax)
		return;

	switch (role) {
	case USB_ROLE_DEVICE:
		device = true;
		break;
	case USB_ROLE_HOST:
		device = false;
		break;
	default:
		device = ax->default_device;
		break;
	}
	axera_usb_set_device_mode(ax, device);
}

static const struct dwc3_glue_ops axera_dwc3_glue_ops = {
	.pre_set_role = axera_pre_set_role,
};

static struct dwc3 *axera_child_dwc(struct dwc3_axera *ax, struct device **held)
{
	struct device_node *child;
	struct platform_device *pdev;
	struct dwc3 *dwc;

	*held = NULL;
	child = of_get_next_child(ax->dev->of_node, NULL);
	if (!child)
		return NULL;
	pdev = of_find_device_by_node(child);
	of_node_put(child);
	if (!pdev)
		return NULL;
	dwc = platform_get_drvdata(pdev);
	*held = &pdev->dev;
	return dwc;
}

static void axera_usb_adj_refclk(void __iomem *regs)
{
	u32 value;

	value = readl(regs + DWC3_GFLADJ);
	value &= ~DWC3_GFLADJ_REFCLK_FLADJ_MASK;
	value &= ~DWC3_GFLADJ_240MHZDECR;
	value |= FIELD_PREP(DWC3_GFLADJ_240MHZDECR, AXERA_GFLADJ_240MHZ_DECR);
	value |= DWC3_GFLADJ_REFCLK_LPM_SEL;
	value |= FIELD_PREP(DWC3_GFLADJ_REFCLK_FLADJ_MASK, AXERA_GFLADJ_FLADJ);
	writel(value, regs + DWC3_GFLADJ);

	value = readl(regs + DWC3_GUCTL);
	value &= ~DWC3_GUCTL_REFCLKPER_MASK;
	value |= AXERA_GUCTL_REFCLKPER << 22;
	writel(value, regs + DWC3_GUCTL);
}

static int dwc3_axera_probe(struct platform_device *pdev)
{
	struct dwc3_axera *ax;
	struct device_node *child;
	void __iomem *dwc_regs = NULL;
	int ret;

	ax = devm_kzalloc(&pdev->dev, sizeof(*ax), GFP_KERNEL);
	if (!ax)
		return -ENOMEM;

	ax->dev = &pdev->dev;
	ax->sys = devm_ioremap(&pdev->dev, FLASH_SYS_GLB_BASE, SZ_64K);
	if (!ax->sys)
		return -ENOMEM;

	platform_set_drvdata(pdev, ax);

	axera_usb_clk_enable(ax);
	axera_usb_reset(ax, true);
	fsleep(1000);
	axera_usb_reset(ax, false);
	/*
	 * Vendor device mode sets USB2_VBUSVALID; host mode clears it.
	 * Do this before the child probes so the first gadget start sees VBUS.
	 * Later online switches update the same bit from pre_set_role.
	 */
	ax->default_device = axera_child_starts_as_device(pdev->dev.of_node);
	axera_usb_set_device_mode(ax, ax->default_device);

	ret = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (ret) {
		axera_usb_reset(ax, true);
		axera_usb_clk_disable(ax);
		return ret;
	}

	/*
	 * 6.18 derives GFLADJ from snps,ref-clock-period-ns and gets ~0.
	 * Vendor writes fladj 0x7f0 and 240 MHz decr 0xa for the 24 MHz ref.
	 */
	child = of_get_next_child(pdev->dev.of_node, NULL);
	if (child) {
		struct resource res;

		if (!of_address_to_resource(child, 0, &res))
			dwc_regs = devm_ioremap(&pdev->dev, res.start,
						resource_size(&res));
		of_node_put(child);
	}
	if (dwc_regs)
		axera_usb_adj_refclk(dwc_regs);

	{
		struct device *child_dev = NULL;
		struct dwc3 *dwc = axera_child_dwc(ax, &child_dev);

		if (dwc)
			dwc->glue_ops = &axera_dwc3_glue_ops;
		if (child_dev)
			put_device(child_dev);
	}

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
	return 0;
}

static void dwc3_axera_remove(struct platform_device *pdev)
{
	struct dwc3_axera *ax = platform_get_drvdata(pdev);
	struct device *child_dev = NULL;
	struct dwc3 *dwc = axera_child_dwc(ax, &child_dev);

	if (dwc)
		dwc->glue_ops = NULL;
	if (child_dev)
		put_device(child_dev);

	of_platform_depopulate(&pdev->dev);
	axera_usb_reset(ax, true);
	axera_usb_clk_disable(ax);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
}

static const struct of_device_id dwc3_axera_match[] = {
	{ .compatible = "axera,dwc3" },
	{ }
};
MODULE_DEVICE_TABLE(of, dwc3_axera_match);

static struct platform_driver dwc3_axera_driver = {
	.probe = dwc3_axera_probe,
	.remove = dwc3_axera_remove,
	.driver = {
		.name = "dwc3-axera",
		.of_match_table = dwc3_axera_match,
	},
};
module_platform_driver(dwc3_axera_driver);

MODULE_DESCRIPTION("Axera AX620E DWC3 glue");
MODULE_LICENSE("GPL");
