// SPDX-License-Identifier: GPL-2.0-only
/* AX620E/AX630C temperature sensor, based on Axera's 4.19 driver. */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/thermal.h>

#define AX_THM_MA_CTRL		0x0c
#define AX_THM_CTRL		0x18
#define AX_THM_CLK_EN		0x1c
#define AX_THM_RSTN		0x24
#define AX_ADC_FILTER_VOL_SEL	0x2c
#define AX_ADC_FILTER_VOL_EN	0x30
#define AX_MSR_ONE_EN		0x38
#define AX_THM_DATA0		0x78
#define AX_MSR_VREF		0xb0
#define AX_THM_MON_CH		0xc4
#define AX_THM_MON_EN		0xc8
#define AX_THM_MON_INTERVAL	0xcc
#define AX_THM_TEMP_HIGH	0xd0
#define AX_THM_TEMP_MEDIAN	0xd4
#define AX_THM_TEMP_LOW		0xd8
#define AX_THM_INT_MASK		0x104

/* Calibration is in the vendor IRAM0 misc_info structure at 0x740. */
#define AX_MISC_INFO_ADDR	0x740
#define AX_MISC_THM_VREF	0x50

struct axera_thermal {
	void __iomem *regs;
	struct mutex lock;
	u32 vref_samples[3];
};

static int axera_thermal_get_temp(struct thermal_zone_device *zone, int *temp)
{
	struct axera_thermal *sensor = thermal_zone_device_priv(zone);
	u32 step, vref;
	s64 millidegrees;

	mutex_lock(&sensor->lock);
	step = readl(sensor->regs + AX_THM_DATA0) & 0x3ff;
	sensor->vref_samples[0] = sensor->vref_samples[1];
	sensor->vref_samples[1] = sensor->vref_samples[2];
	sensor->vref_samples[2] = readl(sensor->regs + AX_MSR_VREF);
	vref = (sensor->vref_samples[0] + sensor->vref_samples[1] +
		sensor->vref_samples[2]) / 3;
	mutex_unlock(&sensor->lock);

	if (!step || !vref || vref > 4095)
		return -EIO;

	/* Axera's step2temp(), calculated in 64 bits to retain precision. */
	millidegrees = div_s64(90000000LL * step, vref * 242) - 277650;
	if (millidegrees < -40000 || millidegrees > 150000)
		return -ERANGE;
	*temp = millidegrees;
	return 0;
}

static const struct thermal_zone_device_ops axera_thermal_ops = {
	.get_temp = axera_thermal_get_temp,
};

static u32 axera_thermal_temp_to_step(u32 celsius, u32 vref)
{
	/* Match the vendor driver's integer threshold conversion. */
	return ((celsius * 100 + 450 + 27315) / 100) * (vref * 242) / 90000;
}

static int axera_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct axera_thermal *sensor;
	struct thermal_zone_device *zone;
	struct regmap *common;
	void __iomem *calibration;
	u32 thm_vref, value, vref;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;
	mutex_init(&sensor->lock);
	sensor->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sensor->regs))
		return PTR_ERR(sensor->regs);

	common = syscon_regmap_lookup_by_phandle(dev->of_node,
						 "axera,common-syscon");
	if (IS_ERR(common))
		return dev_err_probe(dev, PTR_ERR(common), "missing common syscon\n");

	calibration = devm_ioremap(dev, AX_MISC_INFO_ADDR, 0x60);
	if (!calibration)
		return -ENOMEM;
	thm_vref = readl(calibration + AX_MISC_THM_VREF) << 5;
	if (!thm_vref)
		thm_vref = 7 << 5;

	/* Match the vendor clock, reset, ADC and thermal monitor sequence. */
	ret = regmap_write(common, 0x34, BIT(10) | BIT(14));
	if (ret)
		return ret;
	ret = regmap_write(common, 0x5c, GENMASK(20, 19));
	if (ret)
		return ret;
	writel(0, sensor->regs + AX_THM_RSTN);
	writel(0, sensor->regs + AX_THM_MON_EN);
	value = readl(sensor->regs + AX_THM_CLK_EN);
	writel(value | BIT(0), sensor->regs + AX_THM_CLK_EN);
	writel(0, sensor->regs + AX_ADC_FILTER_VOL_SEL);
	writel(0xf, sensor->regs + AX_ADC_FILTER_VOL_EN);
	writel(0x100, sensor->regs + AX_THM_MON_INTERVAL);
	value = readl(sensor->regs + AX_THM_MON_CH);
	writel(value | (0xf << 10), sensor->regs + AX_THM_MON_CH);
	writel(0, sensor->regs + AX_THM_INT_MASK);
	writel(1, sensor->regs + AX_THM_MA_CTRL);
	writel(1, sensor->regs);
	writel(thm_vref | 0x18, sensor->regs + AX_THM_CTRL);
	value = readl(sensor->regs + AX_THM_CLK_EN);
	writel(value | BIT(1), sensor->regs + AX_THM_CLK_EN);
	value = readl(sensor->regs + AX_THM_MON_CH);
	writel(value | BIT(0), sensor->regs + AX_THM_MON_CH);
	writel(1, sensor->regs + AX_THM_RSTN);
	writel(BIT(14), sensor->regs + AX_MSR_ONE_EN);
	udelay(100);
	vref = readl(sensor->regs + AX_MSR_VREF);
	sensor->vref_samples[0] = vref;
	sensor->vref_samples[1] = vref;
	sensor->vref_samples[2] = vref;
	if (!vref || vref > 4095)
		return dev_err_probe(dev, -EIO, "invalid thermal reference voltage\n");
	writel(axera_thermal_temp_to_step(80, vref),
	       sensor->regs + AX_THM_TEMP_LOW);
	writel(axera_thermal_temp_to_step(105, vref),
	       sensor->regs + AX_THM_TEMP_MEDIAN);
	writel(axera_thermal_temp_to_step(120, vref),
	       sensor->regs + AX_THM_TEMP_HIGH);
	value = readl(sensor->regs + AX_THM_MON_CH);
	writel(value | BIT(14), sensor->regs + AX_THM_MON_CH);
	writel(1, sensor->regs + AX_THM_MON_EN);

	zone = devm_thermal_of_zone_register(dev, 0, sensor,
					      &axera_thermal_ops);
	if (IS_ERR(zone))
		return dev_err_probe(dev, PTR_ERR(zone),
				     "failed to register thermal zone\n");
	dev_info(dev, "AX620E temperature sensor ready\n");
	return 0;
}

static const struct of_device_id axera_thermal_of_match[] = {
	{ .compatible = "axera,ax620e-tsensor" },
	{ }
};
MODULE_DEVICE_TABLE(of, axera_thermal_of_match);

static struct platform_driver axera_thermal_driver = {
	.probe = axera_thermal_probe,
	.driver = {
		.name = "axera-ax620e-thermal",
		.of_match_table = axera_thermal_of_match,
	},
};
module_platform_driver(axera_thermal_driver);

MODULE_DESCRIPTION("Axera AX620E/AX630C thermal sensor");
MODULE_LICENSE("GPL");
