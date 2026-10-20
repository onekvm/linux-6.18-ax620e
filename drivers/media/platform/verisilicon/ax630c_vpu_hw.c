// SPDX-License-Identifier: GPL-2.0-only
/* AX630C integration of the Hantro G1 stateless V4L2 decoder. */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/reset.h>

#include "hantro.h"
#include "hantro_g1_regs.h"
#include "hantro_hw.h"

static const struct hantro_fmt ax630c_vdec_formats[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.codec_mode = HANTRO_MODE_NONE,
		.frmsize = {
			.min_width = FMT_MIN_WIDTH, .max_width = 1920,
			.step_width = MB_DIM,
			.min_height = FMT_MIN_HEIGHT, .max_height = 1088,
			.step_height = MB_DIM,
		},
	}, {
		.fourcc = V4L2_PIX_FMT_H264_SLICE,
		.codec_mode = HANTRO_MODE_H264_DEC,
		.max_depth = 2,
		.frmsize = {
			.min_width = FMT_MIN_WIDTH, .max_width = 1920,
			.step_width = MB_DIM,
			.min_height = FMT_MIN_HEIGHT, .max_height = 1088,
			.step_height = MB_DIM,
		},
	},
};

static int ax630c_vdec_init(struct hantro_dev *vpu)
{
	u32 id;
	int ret;

	ret = clk_bulk_prepare_enable(vpu->variant->num_clocks, vpu->clocks);
	if (ret)
		return ret;
	ret = reset_control_deassert(vpu->resets);
	if (ret)
		goto disable;
	id = vdpu_read(vpu, 0);
	if (id != 0x6e645000) {
		dev_err(vpu->dev, "unsupported VDEC hardware ID %08x\n", id);
		ret = -ENODEV;
		goto disable;
	}
	vdpu_write(vpu, 0, G1_REG_INTERRUPT);
	vdpu_write(vpu, 0, G1_REG_PP_INTERRUPT);
	dev_info(vpu->dev, "Hantro G1 %08x, V4L2 stateless H.264\n", id);
disable:
	clk_bulk_disable_unprepare(vpu->variant->num_clocks, vpu->clocks);
	return ret;
}

static void ax630c_vdec_reset(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;

	/* This integration has no G1 SOFT_RESET register. The DT reset is
	 * private to VDEC: never reset shared VIN, VENC or CDMA domains.
	 */
	vdpu_write(vpu, G1_REG_INTERRUPT_DEC_IRQ_DIS, G1_REG_INTERRUPT);
	reset_control_assert(vpu->resets);
	udelay(2);
	reset_control_deassert(vpu->resets);
	vdpu_write(vpu, 0, G1_REG_INTERRUPT);
	vdpu_write(vpu, 0, G1_REG_PP_INTERRUPT);
}

static const struct hantro_codec_ops ax630c_vdec_codec_ops[] = {
	[HANTRO_MODE_H264_DEC] = {
		.run = hantro_g1_h264_dec_run,
		.reset = ax630c_vdec_reset,
		.init = hantro_h264_dec_init,
		.exit = hantro_h264_dec_exit,
	},
};

static const struct hantro_irq ax630c_vdec_irqs[] = {
	{ "vdec", hantro_g1_irq },
};
static const char * const ax630c_vdec_clocks[] = { "vdec_clk" };

const struct hantro_variant ax630c_vdec_variant = {
	.dec_fmts = ax630c_vdec_formats,
	.num_dec_fmts = ARRAY_SIZE(ax630c_vdec_formats),
	.codec = HANTRO_H264_DECODER,
	.codec_ops = ax630c_vdec_codec_ops,
	.init = ax630c_vdec_init,
	.irqs = ax630c_vdec_irqs,
	.num_irqs = ARRAY_SIZE(ax630c_vdec_irqs),
	.clk_names = ax630c_vdec_clocks,
	.num_clocks = ARRAY_SIZE(ax630c_vdec_clocks),
};
