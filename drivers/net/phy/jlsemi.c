// SPDX-License-Identifier: GPL-2.0
/*
 * JLSemi JL2xxx gigabit PHY (OUI/model 0x937c403, ID 0x937c403x).
 * NanoKVM Pro straps this PHY. The vendor 4.19 tree's RGMII and patch
 * macros are off, so clause-22 genphy is enough. -id phy-modes program
 * the vendor 2 ns delay bits (page 3336, reg 17).
 */

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/phy.h>

#define JL2XXX_PHY_ID		0x937c4030

#define JL2XXX_PAGE_REG		0x1f
#define JL2XXX_RGMII_PAGE	3336
#define JL2XXX_RGMII_REG	17
#define JL2XXX_TX_DELAY_2NS	BIT(8)
#define JL2XXX_RX_DELAY_2NS	BIT(9)

static int jl2xxx_read_page(struct phy_device *phydev)
{
	return __phy_read(phydev, JL2XXX_PAGE_REG);
}

static int jl2xxx_write_page(struct phy_device *phydev, int page)
{
	return __phy_write(phydev, JL2XXX_PAGE_REG, page);
}

static int jl2xxx_config_init(struct phy_device *phydev)
{
	u16 set = 0;
	u16 mask = JL2XXX_TX_DELAY_2NS | JL2XXX_RX_DELAY_2NS;

	switch (phydev->interface) {
	case PHY_INTERFACE_MODE_RGMII_ID:
		set = mask;
		break;
	case PHY_INTERFACE_MODE_RGMII_TXID:
		set = JL2XXX_TX_DELAY_2NS;
		break;
	case PHY_INTERFACE_MODE_RGMII_RXID:
		set = JL2XXX_RX_DELAY_2NS;
		break;
	default:
		/* rgmii: leave pin straps. Vendor static RGMII mode is none. */
		return 0;
	}

	return phy_modify_paged(phydev, JL2XXX_RGMII_PAGE, JL2XXX_RGMII_REG,
				mask, set);
}

static struct phy_driver jl2xxx_drivers[] = {
	{
		PHY_ID_MATCH_MODEL(JL2XXX_PHY_ID),
		.name		= "JLSemi JL2xxx",
		.config_init	= jl2xxx_config_init,
		.read_page	= jl2xxx_read_page,
		.write_page	= jl2xxx_write_page,
		.read_status	= genphy_read_status,
		.soft_reset	= genphy_soft_reset,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
	},
};

module_phy_driver(jl2xxx_drivers);

static const struct mdio_device_id __maybe_unused jl2xxx_tbl[] = {
	{ PHY_ID_MATCH_MODEL(JL2XXX_PHY_ID) },
	{ }
};

MODULE_DEVICE_TABLE(mdio, jl2xxx_tbl);
MODULE_DESCRIPTION("JLSemi JL2xxx PHY driver");
MODULE_LICENSE("GPL");
