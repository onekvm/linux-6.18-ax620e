// SPDX-License-Identifier: GPL-2.0-only
/* AX620E peripheral DMA: cyclic receive using the native 64-byte LLI ring.
 * Register/descriptor layout is hardware-specific, not DW AXI DMAC.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include "virt-dma.h"

#define AX_CHANNELS 16
#define AX_ENABLE 0x00
#define AX_CONTROL 0x04
#define AX_START 0x0c
#define AX_STATUS 0x10
#define AX_LLI_H(c) (0x20 + (c) * 8)
#define AX_LLI_L(c) (0x24 + (c) * 8)
#define AX_CLEAR 0xa0
#define AX_HIGH_PERF 0xa4
#define AX_IRQ_ENABLE 0xd8
#define AX_IRQ_STATUS 0xe0
#define AX_IRQ_MASK(c) (0xf4 + (c) * 16)
#define AX_IRQ_CLEAR(c) (0xf8 + (c) * 16)
#define AX_IRQ_CAUSE(c) (0x100 + (c) * 16)
#define AX_BLOCK_DONE BIT(1)
#define AX_ERRORS GENMASK(5, 3)
#define AX_IRQ_ALL GENMASK(5, 0)

#define LLI_IRQ BIT_ULL(2)
#define LLI_SOURCE_PERIPHERAL BIT_ULL(5)
#define LLI_WRITE_BURST GENMASK_ULL(7, 6)
#define LLI_READ_BURST GENMASK_ULL(9, 8)
#define LLI_DEST_WIDTH GENMASK_ULL(26, 24)
#define LLI_SOURCE_WIDTH GENMASK_ULL(29, 27)
#define LLI_DEST_INCREMENT BIT_ULL(30)
#define LLI_DEST_OUTSTANDING GENMASK_ULL(35, 32)
#define LLI_SOURCE_OUTSTANDING GENMASK_ULL(39, 36)
#define LLI_DEST_BURST GENMASK_ULL(51, 42)
#define LLI_SOURCE_BURST GENMASK_ULL(61, 52)
#define LLI_NEXT GENMASK_ULL(36, 0)
#define LLI_TRANSFERS GENMASK_ULL(58, 37)

struct ax_lli {
	__le64 source, destination, control, next_count;
	u64 reserved[4];
};

struct ax_ring {
	struct virt_dma_desc vd;
	struct ax_lli *lli;
	dma_addr_t dma, buffer;
	size_t bytes, period, allocation, completed;
};

struct ax_dmaper;
struct ax_channel {
	struct virt_dma_chan vc;
	struct ax_dmaper *chip;
	struct ax_ring *active;
	struct dma_slave_config config;
	u32 id, request;
	bool failed;
};

struct ax_dmaper {
	struct dma_device dma;
	void __iomem *regs;
	struct regmap *syscon;
	struct ax_channel channels[AX_CHANNELS];
	/* Shared enable registers; lock order is vc.lock then this lock. */
	spinlock_t lock;
};

static struct ax_channel *ax_chan(struct dma_chan *chan)
{
	return container_of(to_virt_chan(chan), struct ax_channel, vc);
}

static void ax_update(struct ax_dmaper *chip, u32 reg, u32 mask, bool set)
{
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&chip->lock, flags);
	value = readl(chip->regs + reg);
	writel(set ? value | mask : value & ~mask, chip->regs + reg);
	spin_unlock_irqrestore(&chip->lock, flags);
}

static void ax_desc_free(struct virt_dma_desc *vd)
{
	struct ax_ring *ring = container_of(vd, struct ax_ring, vd);

	dma_free_coherent(vd->tx.chan->device->dev, ring->allocation,
			  ring->lli, ring->dma);
	kfree(ring);
}

static int ax_config(struct dma_chan *chan, struct dma_slave_config *config)
{
	if (config->direction != DMA_DEV_TO_MEM ||
	    (config->src_addr_width != DMA_SLAVE_BUSWIDTH_2_BYTES &&
	     config->src_addr_width != DMA_SLAVE_BUSWIDTH_4_BYTES) ||
	    !config->src_maxburst || config->src_maxburst > 1023)
		return -EINVAL;
	ax_chan(chan)->config = *config;
	return 0;
}

static struct dma_async_tx_descriptor *
ax_prep_cyclic(struct dma_chan *chan, dma_addr_t buffer, size_t bytes,
	       size_t period, enum dma_transfer_direction direction,
	       unsigned long flags)
{
	struct ax_channel *c = ax_chan(chan);
	struct ax_ring *ring;
	unsigned int i, count, width = c->config.src_addr_width;
	u64 control;

	if (direction != DMA_DEV_TO_MEM || !width || !period ||
	    period > 0x3ffff0 * width || bytes % period ||
	    !IS_ALIGNED(buffer | period, 8))
		return NULL;
	count = bytes / period;
	if (count < 2 || count > 256)
		return NULL;
	ring = kzalloc(sizeof(*ring), GFP_NOWAIT);
	if (!ring)
		return NULL;
	ring->allocation = count * sizeof(*ring->lli);
	ring->lli = dma_alloc_coherent(chan->device->dev, ring->allocation,
				       &ring->dma, GFP_NOWAIT);
	if (!ring->lli) {
		kfree(ring);
		return NULL;
	}
	ring->buffer = buffer;
	ring->bytes = bytes;
	ring->period = period;
	/* The memory side packs into 64-bit writes; peripheral reads stay at
	 * the FIFO sample width. Hardware burst lengths are counts, not log2.
	 */
	control = LLI_IRQ | LLI_SOURCE_PERIPHERAL | LLI_DEST_INCREMENT |
		FIELD_PREP(LLI_WRITE_BURST, 2) | FIELD_PREP(LLI_READ_BURST, 1) |
		FIELD_PREP(LLI_DEST_WIDTH, 3) |
		FIELD_PREP(LLI_SOURCE_WIDTH, ilog2(width)) |
		FIELD_PREP(LLI_DEST_OUTSTANDING, 15) |
		FIELD_PREP(LLI_SOURCE_OUTSTANDING, 15) |
		FIELD_PREP(LLI_DEST_BURST, c->config.src_maxburst * width / 8) |
		FIELD_PREP(LLI_SOURCE_BURST, c->config.src_maxburst);
	memset(ring->lli, 0, ring->allocation);
	for (i = 0; i < count; i++) {
		struct ax_lli *lli = &ring->lli[i];
		dma_addr_t next = ring->dma + ((i + 1) % count) * sizeof(*lli);

		lli->source = cpu_to_le64(c->config.src_addr);
		lli->destination = cpu_to_le64(buffer + i * period);
		lli->control = cpu_to_le64(control);
		lli->next_count = cpu_to_le64(FIELD_PREP(LLI_NEXT, next) |
					    FIELD_PREP(LLI_TRANSFERS, period / width));
	}
	return vchan_tx_prep(&c->vc, &ring->vd, flags);
}

static void ax_issue(struct dma_chan *chan)
{
	struct ax_channel *c = ax_chan(chan);
	struct ax_dmaper *chip = c->chip;
	struct virt_dma_desc *vd;
	unsigned long flags;
	u32 req = c->request, shift = c->id % 5 * 6;

	spin_lock_irqsave(&c->vc.lock, flags);
	if (!vchan_issue_pending(&c->vc) || c->active)
		goto out;
	vd = vchan_next_desc(&c->vc);
	if (!vd)
		goto out;
	list_del(&vd->node);
	c->active = container_of(vd, struct ax_ring, vd);
	c->failed = false;
	regmap_write(chip->syscon, req < 32 ? 0x138 : 0x130, BIT(req % 32));
	regmap_write(chip->syscon, 0x15c - c->id / 5 * 8, 0x3f << shift);
	regmap_write(chip->syscon, 0x158 - c->id / 5 * 8, req << shift);
	writel(AX_IRQ_ALL, chip->regs + AX_IRQ_CLEAR(c->id));
	writel(upper_32_bits(c->active->dma), chip->regs + AX_LLI_H(c->id));
	writel(lower_32_bits(c->active->dma), chip->regs + AX_LLI_L(c->id));
	dma_wmb();
	ax_update(chip, AX_HIGH_PERF, BIT(c->id), true);
	writel(AX_BLOCK_DONE | AX_ERRORS, chip->regs + AX_IRQ_MASK(c->id));
	ax_update(chip, AX_IRQ_ENABLE, BIT(c->id), true);
	ax_update(chip, AX_ENABLE, BIT(c->id), true);
	writel(BIT(c->id), chip->regs + AX_START);
out:
	spin_unlock_irqrestore(&c->vc.lock, flags);
}

static enum dma_status ax_status(struct dma_chan *chan, dma_cookie_t cookie,
				struct dma_tx_state *state)
{
	struct ax_channel *c = ax_chan(chan);
	struct ax_ring *ring;
	enum dma_status status;
	unsigned long flags;

	status = dma_cookie_status(chan, cookie, state);
	if (status == DMA_COMPLETE)
		return status;
	spin_lock_irqsave(&c->vc.lock, flags);
	ring = c->active;
	if (ring && ring->vd.tx.cookie == cookie) {
		if (c->failed)
			status = DMA_ERROR;
		/* The destination register may advance before the corresponding
		 * writes are visible. Only completed blocks belong to the client.
		 */
		dma_set_residue(state, ring->bytes - ring->completed);
	}
	spin_unlock_irqrestore(&c->vc.lock, flags);
	return status;
}

static int ax_terminate(struct dma_chan *chan)
{
	struct ax_channel *c = ax_chan(chan);
	struct ax_dmaper *chip = c->chip;
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&c->vc.lock, flags);
	ax_update(chip, AX_IRQ_ENABLE, BIT(c->id), false);
	writel(0, chip->regs + AX_IRQ_MASK(c->id));
	/* Suspend has a bounded wait even if the external I2S clock vanished.
	 * CLEAR flushes outstanding state before descriptors can be released.
	 */
	ax_update(chip, AX_CONTROL, BIT(c->id), true);
	readl_poll_timeout_atomic(chip->regs + AX_STATUS, value,
				 value & BIT(c->id), 1, 100);
	writel(BIT(c->id), chip->regs + AX_CLEAR);
	ax_update(chip, AX_ENABLE, BIT(c->id), false);
	ax_update(chip, AX_CONTROL, BIT(c->id), false);
	readl(chip->regs + AX_ENABLE);
	writel(AX_IRQ_ALL, chip->regs + AX_IRQ_CLEAR(c->id));
	if (c->active) {
		vchan_terminate_vdesc(&c->active->vd);
		c->active = NULL;
	}
	/* Pending descriptors have never been exposed to hardware. */
	while (!list_empty(&c->vc.desc_issued)) {
		struct virt_dma_desc *vd = vchan_next_desc(&c->vc);

		list_del(&vd->node);
		vchan_terminate_vdesc(vd);
	}
	list_splice_tail_init(&c->vc.desc_submitted, &c->vc.desc_terminated);
	spin_unlock_irqrestore(&c->vc.lock, flags);
	return 0;
}

static void ax_synchronize(struct dma_chan *chan)
{
	vchan_synchronize(&ax_chan(chan)->vc);
}

static void ax_free(struct dma_chan *chan)
{
	struct ax_channel *c = ax_chan(chan);

	ax_terminate(chan);
	ax_synchronize(chan);
	regmap_write(c->chip->syscon, c->request < 32 ? 0x13c : 0x134,
		     BIT(c->request % 32));
	vchan_free_chan_resources(&c->vc);
}

static irqreturn_t ax_interrupt(int irq, void *data)
{
	struct ax_dmaper *chip = data;
	u32 pending = readl(chip->regs + AX_IRQ_STATUS) & GENMASK(15, 0);
	unsigned int i;

	if (!pending)
		return IRQ_NONE;
	for (i = 0; i < AX_CHANNELS; i++) {
		struct ax_channel *c = &chip->channels[i];
		u32 cause;

		if (!(pending & BIT(i)))
			continue;
		spin_lock(&c->vc.lock);
		cause = readl(chip->regs + AX_IRQ_CAUSE(i));
		writel(cause, chip->regs + AX_IRQ_CLEAR(i));
		if (cause & AX_ERRORS) {
			c->failed = true;
			ax_update(chip, AX_IRQ_ENABLE, BIT(i), false);
			dev_err_ratelimited(chip->dma.dev, "channel %u DMA error %#x\n", i, cause);
		}
		if (c->active && (cause & AX_BLOCK_DONE)) {
			struct ax_ring *ring = c->active;

			ring->completed = (ring->completed + ring->period) % ring->bytes;
		}
		if (c->active && (cause & (AX_BLOCK_DONE | AX_ERRORS)))
			vchan_cyclic_callback(&c->active->vd);
		spin_unlock(&c->vc.lock);
	}
	return IRQ_HANDLED;
}

static struct dma_chan *ax_xlate(struct of_phandle_args *spec, struct of_dma *of)
{
	struct ax_dmaper *chip = of->of_dma_data;
	struct dma_chan *chan;

	if (spec->args_count != 1 || spec->args[0] >= 64)
		return NULL;
	chan = dma_get_any_slave_channel(&chip->dma);
	if (chan)
		ax_chan(chan)->request = spec->args[0];
	return chan;
}

static void ax_channels_cleanup(void *data)
{
	struct ax_dmaper *chip = data;
	unsigned int i;

	for (i = 0; i < AX_CHANNELS; i++) {
		struct ax_channel *c = &chip->channels[i];

		ax_terminate(&c->vc.chan);
		vchan_synchronize(&c->vc);
		vchan_free_chan_resources(&c->vc);
		list_del(&c->vc.chan.device_node);
	}
}

static void ax_of_unregister(void *data)
{
	of_dma_controller_free(data);
}

static int ax_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ax_dmaper *chip;
	struct reset_control *resets;
	struct clk_bulk_data *clks;
	int i, irq, ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(chip->regs))
		return PTR_ERR(chip->regs);
	chip->syscon = syscon_regmap_lookup_by_phandle(dev->of_node, "axera,syscon");
	if (IS_ERR(chip->syscon))
		return PTR_ERR(chip->syscon);
	ret = devm_clk_bulk_get_all_enabled(dev, &clks);
	if (ret < 0)
		return ret;
	resets = devm_reset_control_array_get_exclusive(dev);
	if (IS_ERR(resets))
		return PTR_ERR(resets);
	ret = reset_control_deassert(resets);
	if (ret)
		return ret;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(37));
	if (ret)
		return ret;
	spin_lock_init(&chip->lock);
	writel(0, chip->regs + AX_IRQ_ENABLE);
	writel(0, chip->regs + AX_ENABLE);
	writel(0, chip->regs + AX_CONTROL); /* disable LLI prefetch */
	INIT_LIST_HEAD(&chip->dma.channels);
	for (i = 0; i < AX_CHANNELS; i++) {
		struct ax_channel *c = &chip->channels[i];

		c->id = i;
		c->chip = chip;
		c->vc.desc_free = ax_desc_free;
		vchan_init(&c->vc, &chip->dma);
	}
	ret = devm_add_action_or_reset(dev, ax_channels_cleanup, chip);
	if (ret)
		return ret;
	dma_cap_set(DMA_SLAVE, chip->dma.cap_mask);
	dma_cap_set(DMA_CYCLIC, chip->dma.cap_mask);
	chip->dma.dev = dev;
	chip->dma.directions = BIT(DMA_DEV_TO_MEM);
	chip->dma.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
		BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	chip->dma.dst_addr_widths = chip->dma.src_addr_widths;
	chip->dma.residue_granularity = DMA_RESIDUE_GRANULARITY_SEGMENT;
	chip->dma.device_config = ax_config;
	chip->dma.device_prep_dma_cyclic = ax_prep_cyclic;
	chip->dma.device_issue_pending = ax_issue;
	chip->dma.device_tx_status = ax_status;
	chip->dma.device_terminate_all = ax_terminate;
	chip->dma.device_synchronize = ax_synchronize;
	chip->dma.device_free_chan_resources = ax_free;
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_irq(dev, irq, ax_interrupt, 0, dev_name(dev), chip);
	if (ret)
		return ret;
	ret = dmaenginem_async_device_register(&chip->dma);
	if (ret)
		return ret;
	ret = of_dma_controller_register(dev->of_node, ax_xlate, chip);
	if (ret)
		return ret;
	return devm_add_action_or_reset(dev, ax_of_unregister, dev->of_node);
}

static const struct of_device_id ax_match[] = {
	{ .compatible = "axera,ax620e-dmaper" },
	{}
};
MODULE_DEVICE_TABLE(of, ax_match);

static struct platform_driver ax_driver = {
	.probe = ax_probe,
	.driver = {
		.name = "axera-dmaper",
		.of_match_table = ax_match,
	},
};
module_platform_driver(ax_driver);
MODULE_DESCRIPTION("AX620E peripheral cyclic RX DMA");
MODULE_LICENSE("GPL");
