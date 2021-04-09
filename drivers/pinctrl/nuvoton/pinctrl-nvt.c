// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Nuvoton Technology Corp.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/gpio/driver.h>

#include "../core.h"
#include "../pinconf.h"
#include "../pinctrl-utils.h"
#include "pinctrl-nvt.h"

#define NVT_MUX_MASK	0xF

#define GPIO_PORT_NUM 14
#define GPIO_OFFSET 0x10

#define GPIO_MODE 0x0
#define GPIO_DOUT 0x4
#define GPIO_PIN 0x8
#define GPIO_INTSRC 0x20
#define GPIO_SMTEN 0x24
#define GPIO_SLEWCTL 0x28
#define GPIO_SPW 0x2c
#define GPIO_PUSEL 0x30
#define GPIO_DS 0x38
#define GPIO_UDS 0x3C

/*!< Input Mode */
#define GPIO_MODE_INPUT 0x0UL
/*!< Output Mode */
#define GPIO_MODE_OUTPUT 0x1UL
/*!< Open-Drain Mode */
#define GPIO_MODE_OPEN_DRAIN 0x2UL
/*!< Quasi-bidirectional Mode */
#define GPIO_MODE_QUASI 0x3UL

/*!< GPIO PUSEL setting for Pull-up Mode */
#define GPIO_PUSEL_DISABLE 0x0UL
/*!< GPIO PUSEL setting for Pull-up Mode */
#define GPIO_PUSEL_PULL_UP 0x1UL
/*!< GPIO PUSEL setting for Pull-down Mode */
#define GPIO_PUSEL_PULL_DOWN 0x2UL
#define GPIO_PUSEL_MASK 0x3UL	/*!< GPIO PUSEL Mask */

/*!< Generate the MODE mode setting for each pin  */
#define GPIO_SET_MODE(pin, mode) ((mode) << ((pin)<<1))

#define GPIO_PIN_DATA(base, pin) \
	(*((unsigned int *)((base+0x800) + ((pin)<<2))))

char *gpio_port_name[] = {
	"gpioa", "gpiob", "gpioc", "gpiod",
	"gpioe", "gpiof", "gpiog", "gpioh",
	"gpioi", "gpioj", "gpiok", "gpiol",
	"gpiom", "gpion"
};

/**
 * struct nvt_pin_func - pin function description for the pins in a group
 * @name: name of the pin function, used to lookup the function.
 * @groups: names of pin groups that provide this function.
 * @ngroups: number of groups.
 */
struct nvt_pin_func {
	const char *name;
	const char **groups;
	u32 ngroups;
};

/**
 * struct nvt_pin_group - per-pin setting in a group
 * @offset: MFP register offset
 * @shift: MFP bit position
 * @muxval: MFP mux value
 * @configs: Pin configs for this setting
 * @nconfigs: Number of configs for this setting
 */
struct nvt_pin_setting {
	u32 offset;
	u32 shift;
	u32 muxval;
	unsigned long *configs;
	unsigned int nconfigs;
};

/**
 * struct nvt_pin_group - per-group data
 * @name: group name
 * @npins: number of pins included in this group.
 * @pins: the pins included in this group.
 * @settings: pin settings for this group
 */
struct nvt_pin_group {
	const char *name;
	unsigned int npins;
	unsigned int *pins;
	struct nvt_pin_setting *settings;
};

struct nvt_pin_bank {
	void __iomem *reg_base;
	struct clk *clk;
	int irq;
	u8 nr_pins;
	char *name;
	u8 bank_num;
	bool valid;
	struct device_node *of_node;
	struct gpio_chip chip;
	struct irq_chip irqc;
	u32 irqtype;
	u32 irqinten;
	spinlock_t lock;
};

struct nvt_pin_ctrl {
	struct nvt_pin_bank *pin_banks;
	u32 nr_banks;
	u32 nr_pins;
};

struct nvt_pinctrl {
	struct device *dev;
	struct nvt_pin_ctrl *ctrl;
	struct pinctrl_dev *pctl;
	const struct nvt_pinctrl_soc_info *info;
	struct regmap *regmap;
	struct nvt_pin_group *groups;
	unsigned int ngroups;
	struct nvt_pin_func *functions;
	unsigned int nfunctions;
};

static int nvt_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct nvt_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);

	return npctl->ngroups;
}

static const char *nvt_get_group_name(struct pinctrl_dev *pctldev,
				      unsigned int selector)
{
	struct nvt_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);

	return npctl->groups[selector].name;
}

static int nvt_get_group_pins(struct pinctrl_dev *pctldev,
			      unsigned int selector, const unsigned int **pins,
			      unsigned int *npins)
{
	struct nvt_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);

	if (selector >= npctl->ngroups)
		return -EINVAL;

	*pins = npctl->groups[selector].pins;
	*npins = npctl->groups[selector].npins;

	return 0;
}

static inline const struct nvt_pin_group *nvt_pinctrl_find_group_by_name(
		const struct nvt_pinctrl *npctl, const char *name)
{
	int i;

	for (i = 0; i < npctl->ngroups; i++) {
		if (!strcmp(npctl->groups[i].name, name))
			return &npctl->groups[i];
	}

	return NULL;
}

static int nvt_pinctrl_dt_node_to_map_func(struct pinctrl_dev *pctldev,
					   struct device_node *np,
					   struct pinctrl_map **map,
					   unsigned int *num_maps)
{
	struct nvt_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);
	const struct nvt_pin_group *grp;
	struct pinctrl_map *new_map;
	struct device_node *parent;
	int map_num = 1;
	int i;

	/*
	 * first find the group of this node and check if we need create
	 * config maps for pins
	 */
	grp = nvt_pinctrl_find_group_by_name(npctl, np->name);
	if (!grp) {
		dev_err(npctl->dev, "unable to find group for node %s\n",
			np->name);
		return -EINVAL;
	}
	map_num += grp->npins;
	new_map = devm_kzalloc(pctldev->dev, sizeof(*new_map) * map_num,
			       GFP_KERNEL);
	if (!new_map)
		return -ENOMEM;

	*map = new_map;
	*num_maps = map_num;
	/* create mux map */
	parent = of_get_parent(np);
	if (!parent) {
		devm_kfree(pctldev->dev, new_map);
		return -EINVAL;
	}
	new_map[0].type = PIN_MAP_TYPE_MUX_GROUP;
	new_map[0].data.mux.function = parent->name;
	new_map[0].data.mux.group = np->name;
	of_node_put(parent);
	/* create config map */
	new_map++;
	for (i = 0; i < grp->npins; i++) {
		new_map[i].type = PIN_MAP_TYPE_CONFIGS_PIN;
		new_map[i].data.configs.group_or_pin = pin_get_name(pctldev,
			grp->pins[i]);
		new_map[i].data.configs.configs = grp->settings[i].configs;
		new_map[i].data.configs.num_configs = grp->settings[i].nconfigs;
	}
	dev_dbg(pctldev->dev, "maps: function %s group %s num %d\n",
		(*map)->data.mux.function, (*map)->data.mux.group, map_num);
	return 0;
}

static void nvt_dt_free_map(struct pinctrl_dev *pctldev,
                                    struct pinctrl_map *map, unsigned num_maps)
{
	devm_kfree(pctldev->dev,map);
}


static const struct pinctrl_ops nvt_pctrl_ops = {
	.get_groups_count = nvt_get_groups_count,
	.get_group_name = nvt_get_group_name,
	.get_group_pins = nvt_get_group_pins,
	.dt_node_to_map = nvt_pinctrl_dt_node_to_map_func,
	.dt_free_map = nvt_dt_free_map,
};

static int nvt_pinmux_get_func_count(struct pinctrl_dev *pctldev)
{
	struct nvt_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);

	return npctl->nfunctions;
}

static const char *nvt_pinmux_get_func_name(struct pinctrl_dev *pctldev,
					    unsigned int selector)
{
	struct nvt_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);

	return npctl->functions[selector].name;
}

static int nvt_pinmux_get_func_groups(struct pinctrl_dev *pctldev,
				      unsigned int function,
				      const char *const **groups,
				      unsigned int *const num_groups)
{
	struct nvt_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);

	*groups = npctl->functions[function].groups;
	*num_groups = npctl->functions[function].ngroups;

	return 0;
}

static int nvt_pinmux_set_mux(struct pinctrl_dev *pctldev,
			      unsigned int selector, unsigned int group)
{
	struct nvt_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);
	struct nvt_pin_group *grp = &npctl->groups[group];
	unsigned int i, reg;
	struct nvt_pin_setting *setting = grp->settings;

	dev_dbg(npctl->dev, "enable function %s group %s\n",
		npctl->functions[selector].name, npctl->groups[group].name);

	for (i = 0; i < grp->npins; i++) {
		regmap_read(npctl->regmap, setting->offset, &reg);
		reg &= ~(NVT_MUX_MASK << setting->shift);
		reg |= setting->muxval << setting->shift;
		regmap_write(npctl->regmap, setting->offset, reg);
		setting++;
	}

	return 0;
}

struct pinmux_ops nvt_pmx_ops = {
	.get_functions_count = nvt_pinmux_get_func_count,
	.get_function_name = nvt_pinmux_get_func_name,
	.get_function_groups = nvt_pinmux_get_func_groups,
	.set_mux = nvt_pinmux_set_mux,
	.strict = true,
};

static int nvt_gpio_core_direction_in(struct gpio_chip *gc,
				      unsigned int gpio_num)
{
	unsigned long flags;
	unsigned int value;
	struct nvt_pin_bank *bank = gpiochip_get_data(gc);
	void __iomem *base = bank->reg_base;

	spin_lock_irqsave(&bank->lock, flags);
	value = __raw_readl(base + GPIO_MODE);
	value &= ~GPIO_SET_MODE(gpio_num, GPIO_MODE_QUASI);
	value |= GPIO_SET_MODE(gpio_num, GPIO_MODE_INPUT);
	__raw_writel(value, base + GPIO_MODE);
	spin_unlock_irqrestore(&bank->lock, flags);
	return 0;
}

static int nvt_gpio_core_get(struct gpio_chip *gc, unsigned int gpio_num)
{
	struct nvt_pin_bank *bank = gpiochip_get_data(gc);
	void __iomem *base = bank->reg_base;

	return GPIO_PIN_DATA(base, gpio_num);
}

static void nvt_gpio_core_set(struct gpio_chip *gc,
			      unsigned int gpio_num, int val)
{
	struct nvt_pin_bank *bank = gpiochip_get_data(gc);
	void __iomem *base = bank->reg_base;

	GPIO_PIN_DATA(base, gpio_num) = val;
}

static int nvt_gpio_core_direction_out(struct gpio_chip *gc,
				       unsigned int gpio_num, int val)
{
	unsigned long flags;
	unsigned int value;
	struct nvt_pin_bank *bank = gpiochip_get_data(gc);
	void __iomem *base = bank->reg_base;
	spin_lock_irqsave(&bank->lock, flags);
	value = __raw_readl(base + GPIO_MODE);
	value &= ~GPIO_SET_MODE(gpio_num, GPIO_MODE_QUASI);
	value |= GPIO_SET_MODE(gpio_num, GPIO_MODE_OUTPUT);
	__raw_writel(value, base + GPIO_MODE);
	spin_unlock_irqrestore(&bank->lock, flags);
	nvt_gpio_core_set(gc, gpio_num, val);
	return 0;
}

static int nvt_gpio_core_to_request(struct gpio_chip *chip,
	unsigned int offset)
{

	return 0;
}

static void nvt_gpio_core_to_free(struct gpio_chip *chip,
	unsigned int offset)
{

}

static int nvt_gpio_core_to_irq(struct gpio_chip *chip,
	unsigned int offset)
{
	unsigned int irqno = offset;

	return irqno;
}

static void nvt_irq_gpio_mask(struct irq_data *d)
{
	unsigned int num;
	struct nvt_pin_bank *bank =
	    gpiochip_get_data(irq_data_get_irq_chip_data(d));

	num = (d->hwirq);
	__raw_writel(__raw_readl((unsigned int *)(bank->reg_base + 0x1C)) &
		     ~(0x1 << (num + 16)),
		     (unsigned int *)(bank->reg_base + 0x1C));
	__raw_writel(__raw_readl((unsigned int *)(bank->reg_base + 0x1C)) &
		     ~(0x1 << num), (unsigned int *)(bank->reg_base + 0x1C));
}

static void nvt_irq_gpio_unmask(struct irq_data *d)
{
	unsigned int num, tmp;
	struct nvt_pin_bank *bank =
	    gpiochip_get_data(irq_data_get_irq_chip_data(d));

	num = (d->hwirq);
	tmp = bank->irqtype & (0x1 << (num));
	__raw_writel(__raw_readl((unsigned int *)(bank->reg_base + 0x18)) | tmp,
		     (unsigned int *)(bank->reg_base + 0x18));
	tmp = bank->irqinten & (0x1 << (num + 16));
	__raw_writel(__raw_readl((unsigned int *)(bank->reg_base + 0x1C)) | tmp,
		     (unsigned int *)(bank->reg_base + 0x1C));
	tmp = bank->irqinten & (0x1 << num);
	__raw_writel(__raw_readl((unsigned int *)(bank->reg_base + 0x1C)) | tmp,
		     (unsigned int *)(bank->reg_base + 0x1C));
}

static int nvt_irq_irqtype(struct irq_data *d, unsigned int type)
{
	unsigned int num;
	struct nvt_pin_bank *bank =
	    gpiochip_get_data(irq_data_get_irq_chip_data(d));

	num = (d->hwirq);
	if (type == IRQ_TYPE_PROBE) {
		__raw_writel(__raw_readl
			((unsigned int *)(bank->reg_base +
			0x18)) & ~(0x1 << num),
			(unsigned int *)(bank->reg_base + 0x18));
		__raw_writel(__raw_readl
			((unsigned int *)(bank->reg_base +
			0x1C)) | (0x1 << num) | ((0x1 << num) << 16),
			(unsigned int *)(bank->reg_base + 0x1C));
		bank->irqtype &= ~(0x1 << num);
		bank->irqinten |= (0x1 << num);
		bank->irqinten |= (0x1 << (num + 16));
		return 0;
	}

	if (type & IRQ_TYPE_LEVEL_MASK) {
		__raw_writel(__raw_readl
			((unsigned int *)(bank->reg_base + 0x18)) |
			(0x1 << num),
			(unsigned int *)(bank->reg_base + 0x18));
		__raw_writel(__raw_readl
			((unsigned int *)(bank->reg_base +
			0x1C)) & ~((0x1 << num) | ((0x1<<num)<<16)),
			(unsigned int *)(bank->reg_base + 0x1C));
		bank->irqtype |= (0x1 << num);
		bank->irqinten &= ~(0x1 << num);
		bank->irqinten &= ~(0x1 << (num + 16));

		if (type == IRQ_TYPE_LEVEL_HIGH) {
			__raw_writel(__raw_readl
				     ((unsigned int *)(bank->reg_base + 0x1C)) |
				     ((0x1 << num) << 16),
				     (unsigned int *)(bank->reg_base + 0x1C));
			bank->irqinten |= (0x1 << (num + 16));
			return 0;
		}

		if (type == IRQ_TYPE_LEVEL_LOW) {
			__raw_writel(__raw_readl
				((unsigned int *)(bank->reg_base + 0x1C)) |
				(0x1 << num),
				(unsigned int *)(bank->reg_base + 0x1C));
			bank->irqinten |= (0x1 << num);
			return 0;
		}

	} else {
		__raw_writel(__raw_readl
			((unsigned int *)(bank->reg_base + 0x18)) &
			~(0x1 << num),
			(unsigned int *)(bank->reg_base + 0x18));
		bank->irqtype &= ~(0x1 << num);

		if (type & IRQ_TYPE_EDGE_RISING) {
			__raw_writel(__raw_readl
				((unsigned int *)(bank->reg_base +
				0x1C)) | ((0x1 << num) << 16),
				(unsigned int *)(bank->reg_base + 0x1C));
			bank->irqinten |= (0x1 << (num + 16));

		} else {
			__raw_writel(__raw_readl
				((unsigned int *)(bank->reg_base +
				0x1C)) & ~((0x1 << num) << 16),
				(unsigned int *)(bank->reg_base + 0x1C));
			bank->irqinten &= ~(0x1 << (num + 16));
		}

		if (type & IRQ_TYPE_EDGE_FALLING) {
			__raw_writel(__raw_readl
				     ((unsigned int *)(bank->reg_base +
							0x1C)) | (0x1 << num),
				     (unsigned int *)(bank->reg_base + 0x1C));
			bank->irqinten |= (0x1 << num);

		} else {
			__raw_writel(__raw_readl
				     ((unsigned int *)(bank->reg_base +
							0x1C)) & ~(0x1 << num),
				     (unsigned int *)(bank->reg_base + 0x1C));
			bank->irqinten &= ~(0x1 << num);
		}
	}

	return 0;
}

static void nvt_irq_gpio_ack(struct irq_data *d)
{

}

static int nvt_irq_gpio_set_wake(struct irq_data *d, unsigned int on)
{
	return 0;
}

static void nvt_irq_demux_intgroup(struct irq_desc *desc)
{
	unsigned int i, j, isr;
	struct nvt_pin_bank *bank =
	    gpiochip_get_data(irq_desc_get_handler_data(desc));
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	struct irq_domain *irqdomain = bank->chip.irq.domain;

	chained_irq_enter(irqchip, desc);

	isr = __raw_readl(bank->reg_base + GPIO_INTSRC);

	if (isr != 0) {
		__raw_writel(isr, bank->reg_base + (0x40 * i) + GPIO_INTSRC);

		for (j = 0; j < 16; j++) {
			if (isr & 0x1) {
				generic_handle_irq(irq_find_mapping
						   (irqdomain, j));
			}

			isr = isr >> 1;
		}
	}

	chained_irq_exit(irqchip, desc);
}

static int nvt_gpiolib_register(struct platform_device *pdev,
				struct nvt_pinctrl *npctl)
{
	struct nvt_pin_ctrl *ctrl = npctl->ctrl;
	struct nvt_pin_bank *bank = ctrl->pin_banks;
	int ret;
	int i;

	for (i = 0; i < ctrl->nr_banks; ++i, ++bank) {
		if (!bank->valid) {
			dev_warn(&pdev->dev, "bank %s is not valid\n",
				 bank->of_node->name);
			continue;
		}

		dev_info(&pdev->dev, "%s registered %d GPIOs\n",
			bank->of_node->name, bank->nr_pins);
		bank->irqtype = 0;
		bank->irqinten = 0;
		bank->chip.label = bank->name;
		bank->chip.of_gpio_n_cells = 2;
		bank->chip.parent = &pdev->dev;
		bank->chip.request = nvt_gpio_core_to_request;
		bank->chip.free = nvt_gpio_core_to_free;
		bank->chip.direction_input = nvt_gpio_core_direction_in;
		bank->chip.direction_output = nvt_gpio_core_direction_out;
		bank->chip.get = nvt_gpio_core_get;
		bank->chip.set = nvt_gpio_core_set;
		bank->chip.to_irq = nvt_gpio_core_to_irq;
		bank->chip.base = (i * bank->nr_pins);
		bank->chip.ngpio = bank->nr_pins;
		bank->chip.can_sleep = false;
		bank->chip.of_node = bank->of_node;

		if (bank->irq > 0) {
			struct gpio_irq_chip *girq;

			girq = &bank->chip.irq;
			girq->chip = &bank->irqc;
			girq->chip->name = bank->name;
			girq->chip->irq_disable = nvt_irq_gpio_mask;
			girq->chip->irq_enable = nvt_irq_gpio_unmask;
			girq->chip->irq_ack = nvt_irq_gpio_ack;
			girq->chip->irq_mask = nvt_irq_gpio_mask;
			girq->chip->irq_unmask = nvt_irq_gpio_unmask;
			girq->chip->irq_set_type = nvt_irq_irqtype;
			girq->chip->irq_set_wake = nvt_irq_gpio_set_wake;
			girq->chip->flags = IRQCHIP_MASK_ON_SUSPEND;
			girq->parent_handler = nvt_irq_demux_intgroup;
			girq->num_parents = 1;
			girq->parents = devm_kcalloc(&pdev->dev, 1,
						     sizeof(*girq->parents),
						     GFP_KERNEL);

			if (!girq->parents)
				return -ENOMEM;
			girq->parents[0] = bank->irq;
			girq->default_type = IRQ_TYPE_NONE;
			girq->handler = handle_level_irq;
		}

		ret = gpiochip_add_data(&bank->chip, bank);
		if (ret) {
			dev_err(&pdev->dev,
				"failed to register gpio_chip %s, error code: %d\n",
				bank->chip.label, ret);
			goto fail;
		}
	}
	return 0;

fail:
	for (--i, --bank; i >= 0; --i, --bank) {
		if (!bank->valid)
			continue;
		gpiochip_remove(&bank->chip);
	}
	return ret;
}

static int nvt_get_bank_data(struct nvt_pin_bank *bank,
			     struct nvt_pinctrl *npctl)
{
	struct resource res;

	if (of_address_to_resource(bank->of_node, 0, &res)) {
		dev_err(npctl->dev, "cannot find IO resource for bank\n");
		return -ENOENT;
	}

	bank->reg_base = devm_ioremap_resource(npctl->dev, &res);
	if (IS_ERR(bank->reg_base)) {
		dev_err(npctl->dev, "cannot ioremap resource for bank\n");
		return PTR_ERR(bank->reg_base);
	}

	bank->irq = irq_of_parse_and_map(bank->of_node, 0);
	bank->nr_pins = GPIO_OFFSET;

	bank->clk = of_clk_get(bank->of_node, 0);
	if (IS_ERR(bank->clk))
		return PTR_ERR(bank->clk);

	return clk_prepare_enable(bank->clk);
}

static int nvt_pinctrl_get_soc_data(struct nvt_pinctrl *pctl,
				    struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device_node *np;
	struct nvt_pin_ctrl *ctrl;
	struct nvt_pin_bank *bank;
	int i;

	ctrl = pctl->ctrl;
	ctrl->nr_banks = GPIO_PORT_NUM;
	ctrl->pin_banks = devm_kcalloc(&pdev->dev, ctrl->nr_banks,
				       sizeof(*ctrl->pin_banks), GFP_KERNEL);
	if (!ctrl->pin_banks)
		return -ENOMEM;

	for (i = 0; i < ctrl->nr_banks; i++)
		ctrl->pin_banks[i].name = gpio_port_name[i];

	for_each_child_of_node(node, np) {
		if (!of_find_property(np, "gpio-controller", NULL))
			continue;

		bank = ctrl->pin_banks;
		for (i = 0; i < ctrl->nr_banks; ++i, ++bank) {
			if (!strcmp(bank->name, np->name)) {
				bank->of_node = np;

				if (!nvt_get_bank_data(bank, pctl))
					bank->valid = true;

				break;
			}
		}
	}

	return 0;
}

static void nvt_gpio_cla_port(unsigned int gpio_num,
	unsigned int *group, unsigned int *num)
{
	*group = gpio_num / GPIO_OFFSET;
	*num = gpio_num % GPIO_OFFSET;
}

static int nvt_pinconf_set_output(struct nvt_pinctrl *npctl,
				  unsigned int pin_id, bool out)
{

	int port_num, group_num;
	unsigned int value;
	void __iomem *base;
	nvt_gpio_cla_port(pin_id, &group_num, &port_num);

	base = npctl->ctrl->pin_banks[group_num].reg_base;
	value = __raw_readl(base + GPIO_MODE);
	value &= ~GPIO_SET_MODE(port_num, GPIO_MODE_QUASI);
	value |= GPIO_SET_MODE(port_num, GPIO_MODE_OUTPUT);
	__raw_writel(value, base + GPIO_MODE);
	return 0;

}

static int nvt_pinconf_get_output(struct nvt_pinctrl *npctl,
				  unsigned int pin_id)
{

	int port_num, group_num;
	unsigned int value;
	void __iomem *base;

	nvt_gpio_cla_port(pin_id, &group_num, &port_num);
	base = npctl->ctrl->pin_banks[group_num].reg_base;
	value = __raw_readl(base + GPIO_MODE);
	value = (value >> (port_num << 1)) & 0x3;
	if (value == GPIO_MODE_OUTPUT)
		return 1;

	return 0;
}

static int nvt_pinconf_get_pull(struct nvt_pinctrl *npctl,
	unsigned int pin_id)
{

	int port_num, group_num;
	unsigned int value;
	void __iomem *base;

	nvt_gpio_cla_port(pin_id, &group_num, &port_num);
	base = npctl->ctrl->pin_banks[group_num].reg_base;
	value = __raw_readl(base + GPIO_PUSEL);
	value = (value >> (port_num << 1)) & 0x3;
	switch (value) {
	case GPIO_PUSEL_PULL_UP:
		return PIN_CONFIG_BIAS_PULL_UP;
	case GPIO_PUSEL_PULL_DOWN:
		return PIN_CONFIG_BIAS_PULL_DOWN;
	case GPIO_PUSEL_DISABLE:
	default:
		return PIN_CONFIG_BIAS_DISABLE;
	}

	return 0;
}

static int nvt_pinconf_set_pull(struct nvt_pinctrl *npctl,
				unsigned int pin_id, int pull_up)
{

	int port_num, group_num;
	unsigned int value;
	void __iomem *base;

	nvt_gpio_cla_port(pin_id, &group_num, &port_num);
	base = npctl->ctrl->pin_banks[group_num].reg_base;
	value = __raw_readl(base + GPIO_PUSEL);
	value &= ~GPIO_SET_MODE(port_num, GPIO_PUSEL_MASK);
	switch (pull_up) {
	case PIN_CONFIG_BIAS_PULL_UP:
		value |= GPIO_SET_MODE(port_num, GPIO_PUSEL_PULL_UP);
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		value |= GPIO_SET_MODE(port_num, GPIO_PUSEL_PULL_DOWN);
		break;
	case GPIO_PUSEL_DISABLE:
	default:
		value |= GPIO_SET_MODE(port_num, GPIO_PUSEL_DISABLE);
		break;
	}
	__raw_writel(value, base + GPIO_PUSEL);

	return 0;
}

static int nvt_pinconf_get_schmitt_enable(struct nvt_pinctrl *npctl,
					  unsigned int pin_id)
{
	int port_num, group_num;
	unsigned int value;
	void __iomem *base;

	nvt_gpio_cla_port(pin_id, &group_num, &port_num);
	base = npctl->ctrl->pin_banks[group_num].reg_base;
	value = __raw_readl(base + GPIO_SMTEN);
	value = (value>>port_num)&0x1;
	return value;
}

static int nvt_pinconf_set_schmitt(struct nvt_pinctrl *npctl,
					  unsigned int pin_id, int schmitt)
{

	int port_num, group_num;
	unsigned int value;
	void __iomem *base;

	nvt_gpio_cla_port(pin_id, &group_num, &port_num);
	base = npctl->ctrl->pin_banks[group_num].reg_base;
	value = __raw_readl(base + GPIO_SMTEN);
	value &= ~(1<<port_num);
	value |= ((schmitt&0x1)<<port_num);
	__raw_writel(value, base + GPIO_SMTEN);
	return 0;
}

static int nvt_pinconf_get_slew_rate(struct nvt_pinctrl *npctl,
					  unsigned int pin_id)
{
	int port_num, group_num;
	unsigned int value;
	void __iomem *base;

	nvt_gpio_cla_port(pin_id, &group_num, &port_num);
	base = npctl->ctrl->pin_banks[group_num].reg_base;
	value = __raw_readl(base + GPIO_SLEWCTL);
	value = (value >> (port_num << 1)) & 0x3;
	return value;
}

static int nvt_pinconf_set_slew_rate(struct nvt_pinctrl *npctl,
					  unsigned int pin_id, int rate)
{
	int port_num, group_num;
	unsigned int value;
	void __iomem *base;

	nvt_gpio_cla_port(pin_id, &group_num, &port_num);
	base = npctl->ctrl->pin_banks[group_num].reg_base;
	value = __raw_readl(base + GPIO_SLEWCTL);
	value &= ~GPIO_SET_MODE(port_num, 0x3);
	value |= GPIO_SET_MODE(port_num, rate & 0x3);
	__raw_writel(value, base + GPIO_SLEWCTL);
	return 0;
}

static int nvt_pinconf_set_power_source(struct nvt_pinctrl *npctl,
                                          unsigned int pin_id, int src)
{
	int port_num, group_num;
	unsigned int value;
	void __iomem *base;
	int v=0;

	if(src!=1800)
		v=1;
	nvt_gpio_cla_port(pin_id, &group_num, &port_num);
	base = npctl->ctrl->pin_banks[group_num].reg_base;
	value = __raw_readl(base + GPIO_SPW);
	value &= ~(0x1<<port_num);
	value |= (v<<port_num);
	__raw_writel(value, base + GPIO_SPW);
	return 0;
}

static int nvt_pinconf_get_power_source(struct nvt_pinctrl *npctl,
                                          unsigned int pin_id)
{
	int port_num, group_num;
	unsigned int value;
	void __iomem *base;

	nvt_gpio_cla_port(pin_id, &group_num, &port_num);
	base = npctl->ctrl->pin_banks[group_num].reg_base;
	value = __raw_readl(base + GPIO_SPW);
	if((value>>port_num)&0x1)
		return 3300;
	else
		return 1800;
}



static int nvt_pinconf_get_drive_strength(struct nvt_pinctrl *npctl,
					  unsigned int pin_id, u16 *strength)
{

	int port_num, group_num;
	unsigned int u_value, value;
	void __iomem *base;

	nvt_gpio_cla_port(pin_id, &group_num, &port_num);
	base = npctl->ctrl->pin_banks[group_num].reg_base;
	value = __raw_readl(base + GPIO_DS);
	value = ((value & 1<<port_num)>>port_num)|
		((value & 1<<(port_num+16))>>(port_num+16))<<1;
	u_value =
	    (__raw_readl(base + GPIO_UDS) >> port_num) &
	    0x1;
	*strength = (u_value << 2) | value;

	return 0;
}

static int nvt_pinconf_set_drive_strength(struct nvt_pinctrl *npctl,
					  unsigned int pin_id, int strength)
{

	int port_num, group_num;
	unsigned int value, u_value;
	void __iomem *base;

	nvt_gpio_cla_port(pin_id, &group_num, &port_num);
	base = npctl->ctrl->pin_banks[group_num].reg_base;
	value = __raw_readl(base + GPIO_DS);
	value = (value & ~(1<<port_num)) | ((strength&0x1)<<port_num);
	value = (value & ~(1<<(port_num+16))) | (((strength>>1)&0x1)<<(port_num+16));
	__raw_writel(value, base + GPIO_DS);

	u_value = __raw_readl(base + GPIO_UDS);
	u_value &= ~(1 << port_num);
	u_value |= ((strength >> 2) & 0x1) << port_num;
	__raw_writel(u_value, base + GPIO_UDS);

	return 0;
}

static int nvt_pinconf_get(struct pinctrl_dev *pctldev,
			   unsigned int pin_id, unsigned long *config)
{
	struct nvt_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config);
	u16 arg;
	int ret;

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
	case PIN_CONFIG_BIAS_PULL_DOWN:
	case PIN_CONFIG_BIAS_PULL_UP:
		if (nvt_pinconf_get_pull(npctl, pin_id) == param)
			arg = 1;
		else
			return -EINVAL;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		ret = nvt_pinconf_get_drive_strength(npctl, pin_id, &arg);
		if (ret)
			return ret;
		break;
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		arg = nvt_pinconf_get_schmitt_enable(npctl, pin_id);
		break;
	case PIN_CONFIG_SLEW_RATE:
		arg = nvt_pinconf_get_slew_rate(npctl, pin_id);
		break;
	case PIN_CONFIG_OUTPUT_ENABLE:
		arg = nvt_pinconf_get_output(npctl, pin_id);
		break;
	case PIN_CONFIG_POWER_SOURCE:
		 arg = nvt_pinconf_get_power_source(npctl, pin_id);
		break;
	default:
		return -ENOTSUPP;
	}
	*config = pinconf_to_config_packed(param, arg);
	dev_dbg(npctl->dev, "pinconf for pin %u is %lu\n", pin_id, *config);

	return 0;
}

static int nvt_pinconf_set(struct pinctrl_dev *pctldev,
			   unsigned int pin_id, unsigned long *configs,
			   unsigned int num_configs)
{
	struct nvt_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param;
	unsigned int arg = 0;
	int i, ret;

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);
		switch (param) {
		case PIN_CONFIG_BIAS_DISABLE:
		case PIN_CONFIG_BIAS_PULL_UP:
		case PIN_CONFIG_BIAS_PULL_DOWN:
			ret = nvt_pinconf_set_pull(npctl, pin_id, param);
			break;
		case PIN_CONFIG_DRIVE_STRENGTH:
			ret =
			nvt_pinconf_set_drive_strength(npctl, pin_id, arg);
			break;
		case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
			ret =
			nvt_pinconf_set_schmitt(npctl, pin_id, 1);
			break;
		case PIN_CONFIG_INPUT_SCHMITT:
			ret =
			nvt_pinconf_set_schmitt(npctl, pin_id, arg);
			break;
		case PIN_CONFIG_SLEW_RATE:
			ret =
			nvt_pinconf_set_slew_rate(npctl, pin_id, arg);
			break;
		case PIN_CONFIG_OUTPUT_ENABLE:
			ret = nvt_pinconf_set_output(npctl, pin_id, arg);
			break;
		case PIN_CONFIG_POWER_SOURCE:
			ret = nvt_pinconf_set_power_source(npctl, pin_id, arg);
			break;
		default:
		return -ENOTSUPP;
		}
	}
	return 0;
}

static const struct pinconf_ops nvt_pinconf_ops = {
	.pin_config_get = nvt_pinconf_get,
	.pin_config_set = nvt_pinconf_set,
	.is_generic = true,
};

static int nvt_pinctrl_parse_groups(struct device_node *np,
				    struct nvt_pin_group *grp,
				    struct nvt_pinctrl *npctl, u32 index)
{
	struct nvt_pin_setting *pin;
	int size;
	const __be32 *list;
	int i, j, ret;

	dev_dbg(npctl->dev, "group(%d): %s\n", index, np->name);

	/* Initialise group */
	grp->name = np->name;
	/*
	 * the binding format is nuvoton,pins = <bank pin pin-function>,
	 * do sanity check and calculate pins number
	 */
	list = of_get_property(np, "nuvoton,pins", &size);
	/* we do not check return since it's safe node passed down */
	size /= sizeof(*list);
	if (!size || size % 4) {
		dev_err(npctl->dev, "wrong setting!\n");
		return -EINVAL;
	}

	grp->npins = size / 4;

	grp->pins =
	    devm_kzalloc(npctl->dev, grp->npins * sizeof(unsigned int),
			 GFP_KERNEL);
	pin = grp->settings =
	    devm_kzalloc(npctl->dev,
			 grp->npins * sizeof(struct nvt_pin_setting),
			 GFP_KERNEL);
	if (!grp->settings)
		return -ENOMEM;

	for (i = 0, j = 0; i < size; i += 4, j++) {
		const __be32 *phandle;
		struct device_node *np_config;

		pin->offset = be32_to_cpu(*list++);
		pin->shift = be32_to_cpu(*list++);
		pin->muxval = be32_to_cpu(*list++);

		phandle = list++;
		if (!phandle)
			return -EINVAL;

		np_config = of_find_node_by_phandle(be32_to_cpup(phandle));
		ret = pinconf_generic_parse_dt_config(np_config, NULL,
						      &pin->configs,
						      &pin->nconfigs);

		grp->pins[j] =
		    npctl->info->get_pin_num(pin->offset, pin->shift);
		pin++;
	}

	return 0;
}

static int nvt_pinctrl_parse_functions(struct device_node *np,
				       struct nvt_pinctrl *npctl, u32 index)
{
	struct device_node *child;
	struct nvt_pin_func *func;
	struct nvt_pin_group *grp;
	int ret;
	static u32 grp_index;
	u32 i = 0;

	dev_dbg(npctl->dev, "parse function(%d): %s\n", index, np->name);

	func = &npctl->functions[index];

	/* Initialize function */
	func->name = np->name;
	func->ngroups = of_get_child_count(np);

	if (func->ngroups <= 0)   // pin config
		return 0;

	func->groups = devm_kzalloc(npctl->dev,
				    func->ngroups * sizeof(char *), GFP_KERNEL);
	if (!func->groups)
		return -ENOMEM;

	for_each_child_of_node(np, child) {
		func->groups[i] = child->name;
		grp = &npctl->groups[grp_index++];
		ret = nvt_pinctrl_parse_groups(child, grp, npctl, i++);
		if (ret) {
			of_node_put(child);
			return ret;
		}
	}

	return 0;
}

static int nvt_pinctrl_probe_dt(struct platform_device *pdev,
				struct nvt_pinctrl *npctl)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;
	u32 i = 0;
	int ret;

	if (!np)
		return -ENODEV;

	for_each_child_of_node(np, child) {
		if (of_property_read_bool(child, "gpio-controller"))
			continue;
		npctl->nfunctions++;
		npctl->ngroups += of_get_child_count(child);
	}

	npctl->functions = devm_kzalloc(&pdev->dev,
					npctl->nfunctions *
					sizeof(struct nvt_pin_func),
					GFP_KERNEL);
	if (!npctl->functions)
		return -ENOMEM;

	npctl->groups = devm_kzalloc(&pdev->dev,
				     npctl->ngroups *
				     sizeof(struct nvt_pin_group), GFP_KERNEL);
	if (!npctl->groups)
		return -ENOMEM;

	dev_dbg(&pdev->dev, "nfunctions = %d\n", npctl->nfunctions);
	dev_dbg(&pdev->dev, "ngroups = %d\n", npctl->ngroups);

	i = 0;

	for_each_child_of_node(np, child) {
		if (of_property_read_bool(child, "gpio-controller"))
			continue;

		ret = nvt_pinctrl_parse_functions(child, npctl, i++);
		if (ret) {
			dev_err(&pdev->dev, "failed to parse function\n");
			of_node_put(child);
			return ret;
		}
	}

	return 0;
}

int nvt_pinctrl_probe(struct platform_device *pdev,
		      const struct nvt_pinctrl_soc_info *info)
{
	struct device_node *np = pdev->dev.of_node;
	struct pinctrl_desc *nvt_pinctrl_desc;
	struct nvt_pinctrl *npctl;
	int ret;

	if (!info || !info->pins || !info->npins) {
		dev_err(&pdev->dev, "wrong pinctrl info\n");
		return -EINVAL;
	}

	npctl = devm_kzalloc(&pdev->dev, sizeof(*npctl), GFP_KERNEL);
	if (!npctl)
		return -ENOMEM;

	nvt_pinctrl_desc =
	    devm_kzalloc(&pdev->dev, sizeof(*nvt_pinctrl_desc), GFP_KERNEL);
	if (!nvt_pinctrl_desc)
		return -ENOMEM;

	npctl->ctrl =
	    devm_kzalloc(&pdev->dev, sizeof(*npctl->ctrl), GFP_KERNEL);
	if (!npctl->ctrl) {
		dev_err(&pdev->dev, "driver data not available\n");
		return -EINVAL;
	}

	nvt_pinctrl_desc->name = dev_name(&pdev->dev);
	nvt_pinctrl_desc->pins = info->pins;
	nvt_pinctrl_desc->npins = info->npins;
	nvt_pinctrl_desc->pctlops = &nvt_pctrl_ops;
	nvt_pinctrl_desc->pmxops = &nvt_pmx_ops;
	nvt_pinctrl_desc->confops = &nvt_pinconf_ops;
	nvt_pinctrl_desc->owner = THIS_MODULE;

	npctl->info = info;
	npctl->dev = &pdev->dev;
	npctl->regmap = syscon_regmap_lookup_by_phandle(np, "nuvoton,sys");
	nvt_pinctrl_get_soc_data(npctl, pdev);

	ret = nvt_pinctrl_probe_dt(pdev, npctl);
	if (ret) {
		dev_err(&pdev->dev, "fail to probe dt properties\n");
		return ret;
	}

	platform_set_drvdata(pdev, npctl);
	ret = devm_pinctrl_register_and_init(&pdev->dev,
					     nvt_pinctrl_desc, npctl,
					     &npctl->pctl);
	if (ret) {
		dev_err(&pdev->dev, "could not register NVT pinctrl driver\n");
		return ret;
	}

	ret = pinctrl_enable(npctl->pctl);
	if (ret) {
		dev_err(&pdev->dev, "could not enable NVT pinctrl driver\n");
		return ret;
	}

	dev_info(&pdev->dev, "initialized NVT pinctrl driver\n");
	return nvt_gpiolib_register(pdev, npctl);
}

int __maybe_unused nvt_pinctrl_suspend(struct device *dev)
{
	struct nvt_pinctrl *npctl = dev_get_drvdata(dev);

	return pinctrl_force_sleep(npctl->pctl);
}

int __maybe_unused nvt_pinctrl_resume(struct device *dev)
{
	struct nvt_pinctrl *npctl = dev_get_drvdata(dev);

	return pinctrl_force_default(npctl->pctl);
}
