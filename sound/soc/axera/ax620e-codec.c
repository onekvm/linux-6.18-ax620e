// SPDX-License-Identifier: GPL-2.0-only
/* AX620E internal playback codec. Register values follow Axera's GPL BSP. */
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define RX_IIS_FORMAT 0x084
#define TX_ENABLE 0x100
#define TX_RATE_HI 0x104
#define TX_RATE_LO 0x108
#define TX_MUTE 0x118
#define TX_IIS_ENABLE 0x140
#define TX_IIS_FORMAT 0x144
#define TX_IIS_CLOCK 0x148
#define TX_ANALOG_L 0x160
#define TX_ANALOG_R 0x164
#define TX_DIGITAL_L 0x168
#define TX_DIGITAL_R 0x16c

struct ax_codec {
	struct regmap *map;
	struct clk_bulk_data clocks[2];
};

static int ax_codec_params(struct snd_pcm_substream *stream,
			   struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct ax_codec *ax = dev_get_drvdata(dai->dev);
	u32 width, divider;

	if (params_rate(params) != 48000 || params_channels(params) != 2)
		return -EINVAL;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE: width = 0; divider = 2; break;
	case SNDRV_PCM_FORMAT_S24_LE: width = 8; divider = 1; break;
	case SNDRV_PCM_FORMAT_S32_LE: width = 12; divider = 1; break;
	default: return -EINVAL;
	}
	regmap_update_bits(ax->map, RX_IIS_FORMAT, GENMASK(7, 6), divider << 6);
	regmap_update_bits(ax->map, TX_IIS_FORMAT, GENMASK(5, 2), 0x20 | width);
	regmap_write(ax->map, TX_RATE_HI, 0);
	regmap_write(ax->map, TX_RATE_LO, 63);
	return regmap_write(ax->map, TX_IIS_CLOCK, 0);
}

static int ax_codec_format(struct snd_soc_dai *dai, unsigned int format)
{
	if ((format & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S ||
	    (format & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBC_CFC)
		return -EINVAL;
	return 0;
}

static int ax_codec_sysclk(struct snd_soc_dai *dai, int id, unsigned int rate, int dir)
{
	return rate == 12288000 ? 0 : -EINVAL;
}

static int ax_codec_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	struct ax_codec *ax = dev_get_drvdata(dai->dev);
	return regmap_update_bits(ax->map, TX_MUTE, BIT(4), mute ? BIT(4) : 0);
}

static int ax_codec_trigger(struct snd_pcm_substream *stream, int command,
			    struct snd_soc_dai *dai)
{
	struct ax_codec *ax = dev_get_drvdata(dai->dev);
	bool enable;

	switch (command) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE: enable = true; break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH: enable = false; break;
	default: return -EINVAL;
	}
	regmap_write(ax->map, TX_IIS_ENABLE, enable);
	return regmap_write(ax->map, TX_ENABLE, enable);
}

static const struct snd_soc_dai_ops ax_codec_ops = {
	.hw_params = ax_codec_params,
	.set_fmt = ax_codec_format,
	.set_sysclk = ax_codec_sysclk,
	.mute_stream = ax_codec_mute,
	.trigger = ax_codec_trigger,
};

static struct snd_soc_dai_driver ax_codec_dai = {
	.name = "ax620e-codec",
	.playback = {
		.stream_name = "Playback", .channels_min = 2, .channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &ax_codec_ops,
};

static const struct snd_kcontrol_new ax_codec_controls[] = {
	SOC_DOUBLE_R("Playback Volume", TX_DIGITAL_L, TX_DIGITAL_R, 0, 15, 0),
};
static const struct snd_soc_component_driver ax_codec_component = {
	.controls = ax_codec_controls,
	.num_controls = ARRAY_SIZE(ax_codec_controls),
};
static const struct regmap_config ax_codec_regmap = {
	.reg_bits = 32, .val_bits = 32, .reg_stride = 4, .max_register = 0xffc,
};

static void ax_codec_disable(void *data)
{
	struct ax_codec *ax = data;
	regmap_write(ax->map, TX_ENABLE, 0);
	regmap_write(ax->map, TX_IIS_ENABLE, 0);
	clk_bulk_disable_unprepare(ARRAY_SIZE(ax->clocks), ax->clocks);
}

static int ax_codec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ax_codec *ax;
	struct reset_control *resets;
	void __iomem *base;
	int ret;

	ax = devm_kzalloc(dev, sizeof(*ax), GFP_KERNEL);
	if (!ax)
		return -ENOMEM;
	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);
	ax->map = devm_regmap_init_mmio(dev, base, &ax_codec_regmap);
	if (IS_ERR(ax->map))
		return PTR_ERR(ax->map);
	ax->clocks[0].id = "bus";
	ax->clocks[1].id = "mclk";
	ret = devm_clk_bulk_get(dev, 2, ax->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "codec clocks\n");
	resets = devm_reset_control_array_get_exclusive(dev);
	if (IS_ERR(resets))
		return dev_err_probe(dev, PTR_ERR(resets), "codec resets\n");
	ret = clk_bulk_prepare_enable(2, ax->clocks);
	if (ret)
		return ret;
	ret = reset_control_deassert(resets);
	if (ret) {
		clk_bulk_disable_unprepare(2, ax->clocks);
		return ret;
	}
	ret = devm_add_action_or_reset(dev, ax_codec_disable, ax);
	if (ret)
		return ret;
	dev_set_drvdata(dev, ax);
	regmap_write(ax->map, 0x24, 0x05); /* analog DAC control */
	regmap_write(ax->map, TX_ANALOG_L, 0x31);
	regmap_write(ax->map, TX_ANALOG_R, 0x31);
	regmap_write(ax->map, TX_DIGITAL_L, 0x0f);
	regmap_write(ax->map, TX_DIGITAL_R, 0x0f);
	return devm_snd_soc_register_component(dev, &ax_codec_component, &ax_codec_dai, 1);
}

static const struct of_device_id ax_codec_match[] = {
	{ .compatible = "axera,ax620e-codec" }, {}
};
MODULE_DEVICE_TABLE(of, ax_codec_match);
static struct platform_driver ax_codec_driver = {
	.probe = ax_codec_probe,
	.driver = { .name = "ax620e-codec", .of_match_table = ax_codec_match },
};
module_platform_driver(ax_codec_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AX620E internal playback codec");
