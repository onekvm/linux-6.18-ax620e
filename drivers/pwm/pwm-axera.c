// SPDX-License-Identifier: GPL-2.0-only
/* AX620E four-channel timer PWM. Register protocol from the Axera GPL BSP. */
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/reset.h>

#define AX_PWM_MODE 0x1e
#define AX_PWM_ENABLE 0x01
#define AX_PWM_CTRL(n) (0x08 + 0x14 * (n))
#define AX_PWM_LOW(n) (0x14 * (n))
#define AX_PWM_HIGH(n) (0xb0 + 4 * (n))

struct axera_pwm {
	void __iomem *base;
	struct clk_bulk_data clocks[5];
	unsigned long rate;
};

static int axera_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			   const struct pwm_state *state)
{
	struct axera_pwm *ax = pwmchip_get_drvdata(chip);
	u64 period, high;

	if (state->polarity != PWM_POLARITY_NORMAL)
		return -EINVAL;
	if (!state->enabled) {
		writel(AX_PWM_MODE, ax->base + AX_PWM_CTRL(pwm->hwpwm));
		return 0;
	}
	/* Bound before multiplication, including userspace's u64 period. */
	if (state->period > div_u64((u64)U32_MAX * NSEC_PER_SEC, ax->rate))
		return -ERANGE;
	period = mul_u64_u32_div(state->period, ax->rate, NSEC_PER_SEC);
	high = mul_u64_u32_div(state->duty_cycle, ax->rate, NSEC_PER_SEC);
	if (period < 2 || high > period)
		return -ERANGE;
	/* The hardware has a minimum one-tick high/low phase. */
	high = clamp_t(u64, high, 1, period - 1);
	writel(AX_PWM_MODE, ax->base + AX_PWM_CTRL(pwm->hwpwm));
	writel(period - high, ax->base + AX_PWM_LOW(pwm->hwpwm));
	writel(high, ax->base + AX_PWM_HIGH(pwm->hwpwm));
	writel(AX_PWM_MODE | AX_PWM_ENABLE, ax->base + AX_PWM_CTRL(pwm->hwpwm));
	return 0;
}

static int axera_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
			       struct pwm_state *state)
{
	struct axera_pwm *ax = pwmchip_get_drvdata(chip);
	u64 low = readl(ax->base + AX_PWM_LOW(pwm->hwpwm));
	u64 high = readl(ax->base + AX_PWM_HIGH(pwm->hwpwm));

	state->enabled = !!(readl(ax->base + AX_PWM_CTRL(pwm->hwpwm)) & AX_PWM_ENABLE);
	state->polarity = PWM_POLARITY_NORMAL;
	state->period = div_u64((low + high) * NSEC_PER_SEC, ax->rate);
	state->duty_cycle = div_u64(high * NSEC_PER_SEC, ax->rate);
	return 0;
}

static const struct pwm_ops axera_pwm_ops = {
	.apply = axera_pwm_apply,
	.get_state = axera_pwm_get_state,
};

static void axera_pwm_clocks_disable(void *data)
{
	struct axera_pwm *ax = data;
	clk_bulk_disable_unprepare(ARRAY_SIZE(ax->clocks), ax->clocks);
}

static int axera_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *resets;
	struct pwm_chip *chip;
	struct axera_pwm *ax;
	static const char * const names[] = { "pclk", "ch0", "ch1", "ch2", "ch3" };
	int i, ret;

	chip = devm_pwmchip_alloc(dev, 4, sizeof(*ax));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	ax = pwmchip_get_drvdata(chip);
	ax->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ax->base))
		return PTR_ERR(ax->base);
	for (i = 0; i < ARRAY_SIZE(names); i++)
		ax->clocks[i].id = names[i];
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(ax->clocks), ax->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "PWM clocks\n");
	resets = devm_reset_control_array_get_exclusive(dev);
	if (IS_ERR(resets))
		return dev_err_probe(dev, PTR_ERR(resets), "PWM resets\n");
	ret = clk_bulk_prepare_enable(ARRAY_SIZE(ax->clocks), ax->clocks);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, axera_pwm_clocks_disable, ax);
	if (ret)
		return ret;
	ret = reset_control_deassert(resets);
	if (ret)
		return ret;
	ax->rate = clk_get_rate(ax->clocks[1].clk);
	if (!ax->rate || ax->rate > U32_MAX)
		return -EINVAL;
	chip->ops = &axera_pwm_ops;
	return devm_pwmchip_add(dev, chip);
}

static const struct of_device_id axera_pwm_match[] = {
	{ .compatible = "axera,ax620e-pwm" }, {}
};
MODULE_DEVICE_TABLE(of, axera_pwm_match);
static struct platform_driver axera_pwm_driver = {
	.probe = axera_pwm_probe,
	.driver = { .name = "axera-pwm", .of_match_table = axera_pwm_match },
};
module_platform_driver(axera_pwm_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Axera AX620E timer PWM");
