// SPDX-License-Identifier: GPL-2.0-only
/* Capture-only LT6911 -> AX630C I2S slave. External BCLK = 64 * Fs.
 * PCM and DMA use 16-bit samples; the HDMI wire uses 32-bit slots.
 */
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define IER 0x00
#define IRER 0x04
#define CER 0x0c
#define CCR 0x10
#define RXFFR 0x14
#define RER(n) (0x28 + (n) * 0x40)
#define RCR(n) (0x30 + (n) * 0x40)
#define IMR(n) (0x3c + (n) * 0x40)
#define ROR(n) (0x40 + (n) * 0x40)
#define RFCR(n) (0x48 + (n) * 0x40)
#define RXDMA 0x1c0
#define RXDMA_RESET 0x1c4
#define DMACR 0x200
#define RX_DMA_ENABLE BIT(16)

struct ax_hdmi {
	struct snd_soc_card card;
	struct snd_soc_dai_link link;
	struct snd_soc_dai_link_component cpu, platform, codec;
	struct snd_dmaengine_dai_dma_data capture;
	struct snd_pcm_substream *substream;
	void __iomem *base;
	struct clk *pclk;
	struct reset_control *bus_reset, *core_reset;
	int irq;
};

/* ASoC owns dev->driver_data once the card has been registered. */
static struct ax_hdmi *ax_hdmi_data(struct device *dev)
{
	return snd_soc_card_get_drvdata(dev_get_drvdata(dev));
}

static void ax_stop(struct ax_hdmi *hdmi)
{
	writel(0, hdmi->base + DMACR);
	writel(0x33, hdmi->base + IMR(1));
	writel(0, hdmi->base + IRER);
	writel(0, hdmi->base + CER);
	writel(0, hdmi->base + IER);
}

static int ax_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct ax_hdmi *hdmi = ax_hdmi_data(dai->dev);
	int ret;

	/* The slave reset must be released with its APB clock gated. This
	 * sequence also works when HDMI has stopped providing BCLK.
	 */
	ret = reset_control_assert(hdmi->core_reset);
	if (ret)
		return ret;
	clk_disable_unprepare(hdmi->pclk);
	ret = reset_control_deassert(hdmi->core_reset);
	if (clk_prepare_enable(hdmi->pclk))
		return -EIO;
	if (ret)
		return ret;
	ax_stop(hdmi);
	snd_soc_dai_set_dma_data(dai, substream, &hdmi->capture);
	WRITE_ONCE(hdmi->substream, substream);
	return snd_pcm_hw_constraint_step(substream->runtime, 0,
					 SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 8);
}

static void ax_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct ax_hdmi *hdmi = ax_hdmi_data(dai->dev);

	ax_stop(hdmi);
	WRITE_ONCE(hdmi->substream, NULL);
	synchronize_irq(hdmi->irq);
}

static int ax_hw_params(struct snd_pcm_substream *substream,
		       struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct ax_hdmi *hdmi = ax_hdmi_data(dai->dev);
	unsigned int i;

	if (params_rate(params) != 48000 || params_channels(params) != 2 ||
	    params_format(params) != SNDRV_PCM_FORMAT_S16_LE)
		return -EINVAL;
	ax_stop(hdmi);
	for (i = 0; i < 4; i++) {
		writel(0, hdmi->base + RER(i));
		writel(0x33, hdmi->base + IMR(i));
	}
	/* AX630C's external codec route uses RX1 only. */
	writel(2, hdmi->base + RCR(1));
	writel(7, hdmi->base + RFCR(1));
	writel(1, hdmi->base + RER(1));
	writel(0x10, hdmi->base + CCR);
	return 0;
}

static int ax_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct ax_hdmi *hdmi = ax_hdmi_data(dai->dev);

	writel(1, hdmi->base + RXFFR);
	writel(1, hdmi->base + RXDMA_RESET);
	readl(hdmi->base + ROR(1));
	return 0;
}

static int ax_trigger(struct snd_pcm_substream *substream, int cmd,
		      struct snd_soc_dai *dai)
{
	struct ax_hdmi *hdmi = ax_hdmi_data(dai->dev);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		writel(1, hdmi->base + IER);
		writel(0x31, hdmi->base + IMR(1)); /* RX overflow interrupt only */
		writel(1, hdmi->base + IRER);
		writel(RX_DMA_ENABLE, hdmi->base + DMACR);
		writel(1, hdmi->base + CER);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
		ax_stop(hdmi);
		return 0;
	default:
		return -EINVAL;
	}
}

static irqreturn_t ax_irq(int irq, void *data)
{
	struct ax_hdmi *hdmi = data;
	struct snd_pcm_substream *substream;

	if (!(readl(hdmi->base + ROR(1)) & 1))
		return IRQ_NONE;
	substream = READ_ONCE(hdmi->substream);
	if (substream)
		snd_pcm_stop_xrun(substream);
	return IRQ_HANDLED;
}

static const struct snd_soc_dai_ops ax_dai_ops = {
	.startup = ax_startup,
	.shutdown = ax_shutdown,
	.hw_params = ax_hw_params,
	.prepare = ax_prepare,
	.trigger = ax_trigger,
};

static struct snd_soc_dai_driver ax_dai = {
	.name = "ax630c-hdmi-rx",
	.capture = {
		.stream_name = "HDMI Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &ax_dai_ops,
};

static const struct snd_soc_component_driver ax_component = {
	.name = "ax630c-hdmi-i2s",
	.legacy_dai_naming = 1,
};

static const struct snd_pcm_hardware ax_pcm_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_BATCH,
	.buffer_bytes_max = 256 * 1024,
	.period_bytes_min = 256,
	.period_bytes_max = 65536,
	.periods_min = 2,
	.periods_max = 256,
};

static const struct snd_dmaengine_pcm_config ax_pcm_config = {
	.prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config,
	.pcm_hardware = &ax_pcm_hw,
	.prealloc_buffer_size = 64 * 1024,
};

static int ax_card_probe(struct snd_soc_card *card)
{
	/* ALSA otherwise derives just "HDMI" from the last word of the name. */
	snd_card_set_id(card->snd_card, "OneKVMHDMI");
	return 0;
}

static int ax_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ax_hdmi *hdmi;
	struct regmap *syscon;
	struct resource *res;
	int ret;

	hdmi = devm_kzalloc(dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;
	hdmi->card.dev = dev;
	hdmi->card.owner = THIS_MODULE;
	hdmi->card.name = "OneKVM HDMI";
	hdmi->card.late_probe = ax_card_probe;
	hdmi->card.dai_link = &hdmi->link;
	hdmi->card.num_links = 1;
	snd_soc_card_set_drvdata(&hdmi->card, hdmi);
	platform_set_drvdata(pdev, &hdmi->card);
	hdmi->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(hdmi->base))
		return PTR_ERR(hdmi->base);
	hdmi->capture.addr = res->start + RXDMA;
	hdmi->capture.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	hdmi->capture.maxburst = 8;
	hdmi->pclk = devm_clk_get_enabled(dev, "pclk");
	if (IS_ERR(hdmi->pclk))
		return PTR_ERR(hdmi->pclk);
	hdmi->bus_reset = devm_reset_control_get_exclusive(dev, "bus");
	if (IS_ERR(hdmi->bus_reset))
		return PTR_ERR(hdmi->bus_reset);
	hdmi->core_reset = devm_reset_control_get_exclusive(dev, "core");
	if (IS_ERR(hdmi->core_reset))
		return PTR_ERR(hdmi->core_reset);
	ret = reset_control_deassert(hdmi->bus_reset);
	if (ret)
		return ret;
	syscon = syscon_regmap_lookup_by_phandle(dev->of_node, "axera,syscon");
	if (IS_ERR(syscon))
		return PTR_ERR(syscon);
	/* Modify only the slave/external-codec route, preserve master/TDM. */
	ret = regmap_update_bits(syscon, 0x3c,
		BIT(19) | BIT(17) | GENMASK(10, 5), BIT(19) | (3 << 9) | BIT(5));
	if (ret)
		return ret;
	ax_stop(hdmi);
	hdmi->irq = platform_get_irq(pdev, 0);
	if (hdmi->irq < 0)
		return hdmi->irq;
	ret = devm_request_irq(dev, hdmi->irq, ax_irq, 0, dev_name(dev), hdmi);
	if (ret)
		return ret;
	ret = devm_snd_dmaengine_pcm_register(dev, &ax_pcm_config, 0);
	if (ret)
		return dev_err_probe(dev, ret, "register DMA PCM\n");
	ret = devm_snd_soc_register_component(dev, &ax_component, &ax_dai, 1);
	if (ret)
		return ret;
	hdmi->cpu.of_node = dev->of_node;
	hdmi->platform.of_node = dev->of_node;
	hdmi->codec.name = "snd-soc-dummy";
	hdmi->codec.dai_name = "snd-soc-dummy-dai";
	hdmi->link.name = "HDMI";
	hdmi->link.stream_name = "HDMI Capture";
	hdmi->link.cpus = &hdmi->cpu;
	hdmi->link.num_cpus = 1;
	hdmi->link.platforms = &hdmi->platform;
	hdmi->link.num_platforms = 1;
	hdmi->link.codecs = &hdmi->codec;
	hdmi->link.num_codecs = 1;
	hdmi->link.capture_only = 1;
	hdmi->link.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBP_CFP;
	return devm_snd_soc_register_card(dev, &hdmi->card);
}

static const struct of_device_id ax_match[] = {
	{ .compatible = "axera,ax630c-hdmi-audio" },
	{}
};
MODULE_DEVICE_TABLE(of, ax_match);
static struct platform_driver ax_driver = {
	.probe = ax_probe,
	.driver = { .name = "ax630c-hdmi-audio", .of_match_table = ax_match },
};
module_platform_driver(ax_driver);
MODULE_DESCRIPTION("AX630C HDMI I2S slave capture");
MODULE_LICENSE("GPL");
