// SPDX-License-Identifier: GPL-2.0-only
/*
 * Axera AX620E DWMAC glue (EQOS 4.10a, RGMII on NanoKVM Pro)
 *
 * Register map taken from the public maix_ax620e_sdk 4.19 driver.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/stmmac.h>

#include "stmmac.h"
#include "stmmac_platform.h"

#define FLASH_SYS_GLB_BASE		0x10030000
#define EMAC_FLASH_CLK_MUX_0		0x0
#define EMAC_RGMII_TX_SEL		4
#define EMAC_FLASH_SW_RST		0x14
#define EMAC_SW_RST			8
#define EMAC_EPHY_SW_RST		9
#define EMAC_FLASH_EMAC0		0x28
#define EMAC_PHY_IF_SEL			9
#define EMAC_EXT_PAD_SEL		10
#define EMAC_PHY_LOOPBACK_EN		11
#define EPHY_CLK_25M			25000000

struct axera_eqos {
	struct device *dev;
	void __iomem *sys;
	struct clk *ephy_clk;
	struct clk *rgmii_tx_clk;
	struct clk *rmii_phy_clk;
	struct reset_control *emac_rst;
	struct gpio_desc *phy_reset;
	phy_interface_t phy_interface;
};

static void axera_sys_update(struct axera_eqos *eqos, u32 off, u32 mask, u32 val)
{
	u32 v = readl(eqos->sys + off);

	v &= ~mask;
	v |= val;
	writel(v, eqos->sys + off);
}

static void axera_select_phy_interface(struct axera_eqos *eqos)
{
	u32 mask = (0x3 << EMAC_PHY_IF_SEL) | BIT(EMAC_EXT_PAD_SEL) |
		   BIT(EMAC_PHY_LOOPBACK_EN);
	u32 val = 0;

	if (phy_interface_mode_is_rgmii(eqos->phy_interface) ||
	    eqos->phy_interface == PHY_INTERFACE_MODE_GMII)
		val = (0x1 << EMAC_PHY_IF_SEL) | BIT(EMAC_EXT_PAD_SEL);
	else if (eqos->phy_interface == PHY_INTERFACE_MODE_RMII)
		val = 0;

	axera_sys_update(eqos, EMAC_FLASH_EMAC0, mask, val);
}

static void axera_emac_sw_rst(struct axera_eqos *eqos)
{
	if (eqos->emac_rst) {
		reset_control_assert(eqos->emac_rst);
		fsleep(5000);
		reset_control_deassert(eqos->emac_rst);
		return;
	}

	axera_sys_update(eqos, EMAC_FLASH_SW_RST, BIT(EMAC_SW_RST),
			 BIT(EMAC_SW_RST));
	fsleep(5000);
	axera_sys_update(eqos, EMAC_FLASH_SW_RST, BIT(EMAC_SW_RST), 0);
}

static void axera_phy_gpio_reset(struct axera_eqos *eqos)
{
	if (!eqos->phy_reset)
		return;

	gpiod_set_value_cansleep(eqos->phy_reset, 1);
	fsleep(15000);
	gpiod_set_value_cansleep(eqos->phy_reset, 0);
	fsleep(75000);
}

static void axera_eqos_fix_speed(void *priv, int speed, unsigned int mode)
{
	struct axera_eqos *eqos = priv;
	u32 sel;

	if (!phy_interface_mode_is_rgmii(eqos->phy_interface))
		return;

	switch (speed) {
	case SPEED_10:
		sel = 0x0;
		break;
	case SPEED_100:
		sel = 0x1;
		break;
	case SPEED_1000:
		sel = 0x2;
		break;
	default:
		return;
	}

	axera_sys_update(eqos, EMAC_FLASH_CLK_MUX_0, 0x3 << EMAC_RGMII_TX_SEL,
			 sel << EMAC_RGMII_TX_SEL);
}

static int axera_dwmac_init(struct platform_device *pdev, void *priv)
{
	struct axera_eqos *eqos = priv;

	axera_phy_gpio_reset(eqos);
	return 0;
}

static int dwmac_axera_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat;
	struct stmmac_resources stmmac_res;
	struct axera_eqos *eqos;
	int ret;

	eqos = devm_kzalloc(&pdev->dev, sizeof(*eqos), GFP_KERNEL);
	if (!eqos)
		return -ENOMEM;
	eqos->dev = &pdev->dev;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	plat = devm_stmmac_probe_config_dt(pdev, stmmac_res.mac);
	if (IS_ERR(plat))
		return PTR_ERR(plat);

	eqos->sys = devm_ioremap(eqos->dev, FLASH_SYS_GLB_BASE, SZ_4K);
	if (!eqos->sys)
		return -ENOMEM;

	eqos->phy_interface = plat->phy_interface;
	eqos->emac_rst = devm_reset_control_get_optional_exclusive(eqos->dev,
								   "emac_rst");
	if (IS_ERR(eqos->emac_rst))
		return PTR_ERR(eqos->emac_rst);

	eqos->phy_reset = devm_gpiod_get_optional(eqos->dev, "phy-rst",
						  GPIOD_OUT_HIGH);
	if (IS_ERR(eqos->phy_reset))
		eqos->phy_reset = devm_gpiod_get_optional(eqos->dev, "phy-reset",
							  GPIOD_OUT_HIGH);
	if (IS_ERR(eqos->phy_reset))
		return PTR_ERR(eqos->phy_reset);

	plat->stmmac_clk = devm_clk_get_optional_enabled(eqos->dev, "emac_aclk");
	if (IS_ERR(plat->stmmac_clk))
		return PTR_ERR(plat->stmmac_clk);

	eqos->ephy_clk = devm_clk_get_optional(eqos->dev, "ephy_clk");
	if (IS_ERR(eqos->ephy_clk))
		return PTR_ERR(eqos->ephy_clk);
	if (eqos->ephy_clk) {
		clk_set_rate(eqos->ephy_clk, EPHY_CLK_25M);
		ret = clk_prepare_enable(eqos->ephy_clk);
		if (ret)
			return ret;
	}

	eqos->rgmii_tx_clk = devm_clk_get_optional_enabled(eqos->dev,
							   "rgmii_tx_clk");
	if (IS_ERR(eqos->rgmii_tx_clk))
		return PTR_ERR(eqos->rgmii_tx_clk);

	eqos->rmii_phy_clk = devm_clk_get_optional_enabled(eqos->dev,
							   "rmii_phy_clk");
	if (IS_ERR(eqos->rmii_phy_clk))
		return PTR_ERR(eqos->rmii_phy_clk);

	plat->clk_ptp_ref = devm_clk_get_optional_enabled(eqos->dev, "ptp_clk");
	if (IS_ERR(plat->clk_ptp_ref))
		plat->clk_ptp_ref = NULL;
	if (plat->clk_ptp_ref)
		plat->clk_ptp_rate = clk_get_rate(plat->clk_ptp_ref);
	else if (plat->stmmac_clk)
		plat->clk_ptp_rate = clk_get_rate(plat->stmmac_clk);

	/* CSR clock is typically 208–312 MHz → MDC = csr/204 */
	plat->clk_csr = STMMAC_CSR_300_500M;
	plat->core_type = DWMAC_CORE_GMAC4;
	plat->pmt = 1;
	plat->fix_mac_speed = axera_eqos_fix_speed;
	plat->init = axera_dwmac_init;
	plat->bsp_priv = eqos;

	axera_emac_sw_rst(eqos);
	axera_select_phy_interface(eqos);
	axera_phy_gpio_reset(eqos);

	ret = stmmac_pltfr_probe(pdev, plat, &stmmac_res);
	if (ret && eqos->ephy_clk)
		clk_disable_unprepare(eqos->ephy_clk);
	return ret;
}

static const struct of_device_id dwmac_axera_match[] = {
	{ .compatible = "axera,dwmac-4.10a" },
	{ }
};
MODULE_DEVICE_TABLE(of, dwmac_axera_match);

static struct platform_driver dwmac_axera_driver = {
	.probe = dwmac_axera_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		.name = "dwmac-axera",
		.pm = &stmmac_pltfr_pm_ops,
		.of_match_table = dwmac_axera_match,
	},
};
module_platform_driver(dwmac_axera_driver);

MODULE_DESCRIPTION("Axera AX620E DWMAC glue");
MODULE_LICENSE("GPL");
