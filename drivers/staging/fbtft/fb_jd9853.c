// SPDX-License-Identifier: GPL-2.0+
/*
 * Jadard JD9853, the NanoKVM Pro Desk 172x320 panel.
 *
 * Register sequence is the Sipeed 4.19 fb_jd9853 init (iawak9lkm), run twice.
 * The panel is a 172-column window at column offset 0x22 on a 240-column
 * controller. Unchanged frames are skipped; changed rows use a matching window.
 * No tearing-effect
 * pin and no DMA byte swap: stock spi-dw-mmio is PIO and does not swap.
 *
 * Mainline fbtft uses the logical GPIO API. dc-gpios must be ACTIVE_HIGH
 * (high = data). reset-gpios stays ACTIVE_LOW.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <video/mipi_display.h>

#include "fbtft.h"

#define DRVNAME			"fb_jd9853"

#define MADCTL_MV		BIT(5)
#define MADCTL_MX		BIT(6)
#define MADCTL_MY		BIT(7)
#define MADCTL_BGR		BIT(3)

/* Columns 0x22..0xcd and rows 0x000..0x13f: 172 x 320. */
#define JD9853_COL_START	0x0022
#define JD9853_COL_END		0x00cd
#define JD9853_ROW_START	0x0000
#define JD9853_ROW_END		0x013f

struct jd9853_frame {
	u8 *snapshot;
	u8 *last;
	bool valid;
};

static int set_var(struct fbtft_par *par)
{
	struct jd9853_frame *frame = par->extra;
	u8 madctl = 0;

	if (frame)
		frame->valid = false;
	if (par->bgr)
		madctl |= MADCTL_BGR;

	switch (par->info->var.rotate) {
	case 0:
		break;
	case 90:
		madctl |= MADCTL_MV | MADCTL_MY;
		break;
	case 180:
		madctl |= MADCTL_MX | MADCTL_MY;
		break;
	case 270:
		madctl |= MADCTL_MV | MADCTL_MX;
		break;
	default:
		return -EINVAL;
	}

	write_reg(par, MIPI_DCS_SET_ADDRESS_MODE, madctl);
	return 0;
}

static void set_addr_win(struct fbtft_par *par, int xs, int ys, int xe, int ye)
{
	u8 col_cmd = MIPI_DCS_SET_COLUMN_ADDRESS;
	u8 row_cmd = MIPI_DCS_SET_PAGE_ADDRESS;

	switch (par->info->var.rotate) {
	case 0:
	case 180:
		break;
	case 90:
	case 270:
		swap(col_cmd, row_cmd);
		break;
	default:
		return;
	}

	write_reg(par, col_cmd,
		  JD9853_COL_START >> 8, JD9853_COL_START & 0xff,
		  JD9853_COL_END >> 8, JD9853_COL_END & 0xff);
	write_reg(par, row_cmd,
		  JD9853_ROW_START >> 8, JD9853_ROW_START & 0xff,
		  JD9853_ROW_END >> 8, JD9853_ROW_END & 0xff);
	write_reg(par, MIPI_DCS_SET_TEAR_ON, 0x00);
	write_reg(par, MIPI_DCS_WRITE_MEMORY_START);
}

/* Keep the 0x22 column offset and move the row window with the payload. */
static int write_rows(struct fbtft_par *par, unsigned int first,
		      unsigned int last)
{
	struct fb_info *info = par->info;
	struct jd9853_frame *frame = par->extra;
	size_t offset = first * info->fix.line_length;
	size_t len = (last - first + 1) * info->fix.line_length;
	void *live = info->screen_buffer;
	int ret;

	if (info->var.rotate == 90 || info->var.rotate == 270) {
		/* Preserve the existing full-window hardware-rotation path. */
		set_addr_win(par, 0, 0, 0, 0);
	} else {
		write_reg(par, MIPI_DCS_SET_COLUMN_ADDRESS,
			  JD9853_COL_START >> 8, JD9853_COL_START & 0xff,
			  JD9853_COL_END >> 8, JD9853_COL_END & 0xff);
		write_reg(par, MIPI_DCS_SET_PAGE_ADDRESS,
			  first >> 8, first & 0xff, last >> 8, last & 0xff);
		write_reg(par, MIPI_DCS_WRITE_MEMORY_START);
	}
	/* Snapshot before transfer: mmap writers may replace the next frame. */
	info->screen_buffer = frame->snapshot;
	ret = fbtft_write_vmem16_bus8(par, offset, len);
	info->screen_buffer = live;
	return ret;
}

static void update_display(struct fbtft_par *par, unsigned int start_line,
			   unsigned int end_line)
{
	struct fb_info *info = par->info;
	struct jd9853_frame *frame = par->extra;
	size_t stride = info->fix.line_length;
	unsigned int first, row, height = info->var.yres;
	int ret;

	/* fbdev page damage can cover unchanged rows. Compare actual pixels,
	 * including disjoint runs such as the clock's old and new second dot.
	 */
	memcpy(frame->snapshot, info->screen_buffer, info->fix.smem_len);
	if (frame->valid &&
	    !memcmp(frame->snapshot, frame->last, info->fix.smem_len))
		return;

	if (!frame->valid || info->var.rotate == 90 || info->var.rotate == 270) {
		ret = write_rows(par, 0, height - 1);
		if (ret < 0)
			goto failed;
	} else {
		row = 0;
		while (row < height) {
			if (!memcmp(frame->snapshot + row * stride,
				    frame->last + row * stride, stride)) {
				row++;
				continue;
			}
			first = row++;
			while (row < height &&
			       memcmp(frame->snapshot + row * stride,
				      frame->last + row * stride, stride))
				row++;
			ret = write_rows(par, first, row - 1);
			if (ret < 0)
				goto failed;
		}
	}
	memcpy(frame->last, frame->snapshot, info->fix.smem_len);
	frame->valid = true;
	/* Reset preserves GRAM. Never expose the previous boot's pixels while
	 * either initialization pass is running; the first full transfer must
	 * finish before the panel is allowed to scan out again.
	 */
	if (!par->first_update_done)
		write_reg(par, MIPI_DCS_SET_DISPLAY_ON);
	par->first_update_done = true;
	return;

failed:
	/* A partial failure needs a complete redraw on the next update. */
	frame->valid = false;
	dev_err(info->device, "display update failed: %d\n", ret);
}

static int blank(struct fbtft_par *par, bool on)
{
	if (!on && !par->first_update_done)
		return 0;
	write_reg(par, on ? MIPI_DCS_SET_DISPLAY_OFF : MIPI_DCS_SET_DISPLAY_ON);
	return 0;
}

static int init_display(struct fbtft_par *par)
{
	struct jd9853_frame *frame = par->extra;
	int i;

	if (!frame) {
		frame = devm_kzalloc(par->info->device, sizeof(*frame), GFP_KERNEL);
		if (!frame)
			return -ENOMEM;
		frame->snapshot = devm_kmalloc(par->info->device,
					      par->info->fix.smem_len, GFP_KERNEL);
		frame->last = devm_kmalloc(par->info->device,
					  par->info->fix.smem_len, GFP_KERNEL);
		if (!frame->snapshot || !frame->last)
			return -ENOMEM;
		par->extra = frame;
	}
	frame->valid = false;
	par->first_update_done = false;

	for (i = 0; i < 2; i++) {
		mdelay(100);
		par->fbtftops.reset(par);
		mdelay(50);

		write_reg(par, 0xDF, 0x98, 0x53);
		write_reg(par, 0xB2, 0x23);
		write_reg(par, 0xB7, 0x00, 0x47, 0x00, 0x6F);
		write_reg(par, 0xBB, 0x1C, 0x1A, 0x55, 0x73, 0x63, 0xF0);
		write_reg(par, 0xC0, 0x44, 0xA4);
		write_reg(par, 0xC1, 0x12);
		write_reg(par, 0xC3, 0x7D, 0x07, 0x14, 0x06, 0xCF, 0x71, 0x72,
			  0x77);
		write_reg(par, 0xC4, 0x00, 0x00, 0xA0, 0x79, 0x0B, 0x0A, 0x16,
			  0x79, 0x0B, 0x0A, 0x16, 0x82);
		write_reg(par, 0xC8, 0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27,
			  0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00,
			  0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28,
			  0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00);
		write_reg(par, 0xD0, 0x04, 0x06, 0x6B, 0x0F, 0x00);
		write_reg(par, 0xD7, 0x00, 0x30);
		write_reg(par, 0xE6, 0x14);
		write_reg(par, 0xDE, 0x01);
		write_reg(par, 0xB7, 0x03, 0x13, 0xEF, 0x35, 0x35);
		write_reg(par, 0xC1, 0x14, 0x15, 0xC0);
		write_reg(par, 0xC2, 0x06, 0x3A);
		write_reg(par, 0xC4, 0x72, 0x12);
		write_reg(par, 0xBE, 0x00);
		write_reg(par, 0xDE, 0x02);
		write_reg(par, 0xE5, 0x00, 0x02, 0x00);
		write_reg(par, 0xE5, 0x01, 0x02, 0x00);
		write_reg(par, 0xDE, 0x00);
		write_reg(par, MIPI_DCS_SET_TEAR_OFF);
		write_reg(par, 0x44, 0x00, 0x00);
		write_reg(par, MIPI_DCS_SET_TEAR_ON, 0x00);
		write_reg(par, 0x44, 0x00, 0x00);
		write_reg(par, MIPI_DCS_SET_PIXEL_FORMAT, 0x55);
		set_var(par);
		set_addr_win(par, 0, 0, 0, 0);
		write_reg(par, MIPI_DCS_EXIT_SLEEP_MODE);
		mdelay(120);
		write_reg(par, 0xDE, 0x02);
		write_reg(par, 0xE5, 0x00, 0x02, 0x00);
		write_reg(par, 0xDE, 0x00);
	}

	return 0;
}

static struct fbtft_display display = {
	.regwidth = 8,
	.width = 172,
	.height = 320,
	.gamma_num = 0,
	.gamma_len = 0,
	.gamma = "",
	.fbtftops = {
		.init_display = init_display,
		.set_var = set_var,
		.set_addr_win = set_addr_win,
		.update_display = update_display,
		.blank = blank,
	},
};

FBTFT_REGISTER_SPI_DRIVER(DRVNAME, "jadard", "jd9853", &display);

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("spi:jd9853");
MODULE_DESCRIPTION("FB driver for the Jadard JD9853 LCD Controller");
MODULE_AUTHOR("iawak9lkm");
MODULE_LICENSE("GPL");
