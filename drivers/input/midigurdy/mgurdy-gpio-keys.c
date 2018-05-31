/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Heavily modified for the MidiGurdy to provide three key codes per key:
 * state, short-press and long-press
 *
 * Copyright 2005 Phil Blundell
 * Copyright 2010, 2011 David Jander <david@protonic.nl>
 * Copyright 2018, <marcus@weseloh.cc>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>

struct mg_gpio_keys_button {
	unsigned int state_code;
	unsigned int short_code;
	unsigned int long_code;
	int gpio;
	int active_low;
	const char *desc;
	int debounce_interval;
	unsigned int longpress_delay;
	int value;
	unsigned int irq;
};

struct mg_gpio_keys_platform_data {
	const struct mg_gpio_keys_button *buttons;
	int nbuttons;
	unsigned int poll_interval;
	const char *name;
};

struct gpio_button_data {
	const struct mg_gpio_keys_button *button;
	struct input_dev *input;
	struct gpio_desc *gpiod;

	unsigned short *state_code;
	unsigned short *short_code;
	unsigned short *long_code;

	struct delayed_work work;
	unsigned int software_debounce;	/* in msecs, for GPIO-driven buttons */

	struct timer_list longpress_timer;
	unsigned int longpress_delay;
	bool sent_as_long;

	unsigned int irq;
	spinlock_t lock;
	bool key_pressed;
};

struct gpio_keys_drvdata {
	const struct mg_gpio_keys_platform_data *pdata;
	struct input_dev *input;
	unsigned short *keymap;
	struct gpio_button_data data[0];
};

static void gpio_keys_gpio_report_event(struct gpio_button_data *bdata)
{
	struct input_dev *input = bdata->input;
	int state;

	state = gpiod_get_value_cansleep(bdata->gpiod);
	if (state < 0) {
		dev_err(input->dev.parent,
			"failed to get gpio state: %d\n", state);
		return;
	}

	bdata->key_pressed = state;

	if (bdata->key_pressed) {
		mod_timer(&bdata->longpress_timer,
			  jiffies + bdata->longpress_delay);
	}
	else {
		del_timer_sync(&bdata->longpress_timer);
		if (!bdata->sent_as_long) {
			input_event(input, EV_KEY, *bdata->short_code, 1);
			input_event(input, EV_KEY, *bdata->short_code, 0);
			input_sync(input);
		}
		bdata->sent_as_long = false;
	}

	input_event(input, EV_KEY, *bdata->state_code, bdata->key_pressed);
	input_sync(input);
}

static void gpio_keys_gpio_work_func(struct work_struct *work)
{
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, work.work);

	gpio_keys_gpio_report_event(bdata);
}

static irqreturn_t gpio_keys_gpio_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;

	BUG_ON(irq != bdata->irq);

	mod_delayed_work(system_wq,
			 &bdata->work,
			 msecs_to_jiffies(bdata->software_debounce));

	return IRQ_HANDLED;
}

static void gpio_keys_longpress_timer(unsigned long data)
{
	struct gpio_button_data *bdata = (struct gpio_button_data *)data;
	struct input_dev *input = bdata->input;
	unsigned long flags;

	spin_lock_irqsave(&bdata->lock, flags);
	if (bdata->key_pressed) {
		input_event(input, EV_KEY, *bdata->long_code, 1);
		input_event(input, EV_KEY, *bdata->long_code, 0);
		input_sync(input);

		bdata->sent_as_long = true;

	}
	spin_unlock_irqrestore(&bdata->lock, flags);
}

static int gpio_keys_setup_key(struct platform_device *pdev,
				struct input_dev *input,
				struct gpio_keys_drvdata *ddata,
				const struct mg_gpio_keys_button *button,
				int idx,
				struct fwnode_handle *child)
{
	const char *desc = button->desc ? button->desc : "mgurdy_gpio_keys";
	struct device *dev = &pdev->dev;
	struct gpio_button_data *bdata = &ddata->data[idx];
	unsigned long irqflags;
	int irq;
	int error;

	bdata->input = input;
	bdata->button = button;
	bdata->sent_as_long = false;
	spin_lock_init(&bdata->lock);

	if (child) {
		bdata->gpiod = devm_fwnode_get_gpiod_from_child(dev, NULL,
								child,
								GPIOD_IN,
								desc);
		if (IS_ERR(bdata->gpiod)) {
			error = PTR_ERR(bdata->gpiod);
			if (error != -EPROBE_DEFER)
				dev_err(dev, "failed to get gpio: %d\n", error);
			return error;
		}
	}

	if (!bdata->gpiod) {
		dev_err(dev, "Found button without gpio\n");
		return -EINVAL;
	}

	if (button->debounce_interval) {
		error = gpiod_set_debounce(bdata->gpiod,
				button->debounce_interval * 1000);
		/* use timer if gpiolib doesn't provide debounce */
		if (error < 0)
			bdata->software_debounce =
					button->debounce_interval;
	}

	irq = gpiod_to_irq(bdata->gpiod);
	if (irq < 0) {
		error = irq;
		dev_err(dev,
			"Unable to get irq number for GPIO %d, error %d\n",
			button->gpio, error);
		return error;
	}
	bdata->irq = irq;

	INIT_DELAYED_WORK(&bdata->work, gpio_keys_gpio_work_func);

	setup_timer(&bdata->longpress_timer,
			gpio_keys_longpress_timer, (unsigned long)bdata);

	bdata->state_code = &ddata->keymap[idx * 3];
	bdata->short_code = &ddata->keymap[idx * 3 + 1];
	bdata->long_code = &ddata->keymap[idx * 3 + 2];
	*bdata->state_code = button->state_code;
	*bdata->short_code = button->short_code;
	*bdata->long_code = button->long_code;

	input_set_capability(input, EV_KEY, *bdata->state_code);
	input_set_capability(input, EV_KEY, *bdata->short_code);
	input_set_capability(input, EV_KEY, *bdata->long_code);

	bdata->longpress_delay = button->longpress_delay;

	irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_SHARED;
	error = devm_request_any_context_irq(dev, bdata->irq,
					     gpio_keys_gpio_isr, irqflags,
					     desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			bdata->irq, error);
		return error;
	}

	return 0;
}

static void gpio_keys_report_state(struct gpio_keys_drvdata *ddata)
{
	struct input_dev *input = ddata->input;
	int i;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];
		if (bdata->gpiod)
			gpio_keys_gpio_report_event(bdata);
	}
	input_sync(input);
}

static int gpio_keys_open(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);

	/* Report current state of buttons that are connected to GPIOs */
	gpio_keys_report_state(ddata);

	return 0;
}

/*
 * Translate properties into platform_data
 */
static struct mg_gpio_keys_platform_data *
gpio_keys_get_devtree_pdata(struct device *dev)
{
	struct mg_gpio_keys_platform_data *pdata;
	struct mg_gpio_keys_button *button;
	struct fwnode_handle *child;
	int nbuttons;
	unsigned int longpress_msecs;

	nbuttons = device_get_child_node_count(dev);
	if (nbuttons == 0)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev,
			     sizeof(*pdata) + nbuttons * sizeof(*button),
			     GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	button = (struct mg_gpio_keys_button *)(pdata + 1);

	pdata->buttons = button;
	pdata->nbuttons = nbuttons;

	device_property_read_string(dev, "label", &pdata->name);

	if (device_property_read_u32(dev, "mgurdy,long-press-msecs",
				     &longpress_msecs)) {
		longpress_msecs = 500;
	}

	device_for_each_child_node(dev, child) {
		if (is_of_node(child))
			button->irq =
				irq_of_parse_and_map(to_of_node(child), 0);

		if (fwnode_property_read_u32(child, "mgurdy,state-code",
					     &button->state_code)) {
			dev_err(dev, "Button without state keycode\n");
			fwnode_handle_put(child);
			return ERR_PTR(-EINVAL);
		}

		if (fwnode_property_read_u32(child, "mgurdy,short-code",
					     &button->short_code)) {
			dev_err(dev, "Button without short-press keycode\n");
			fwnode_handle_put(child);
			return ERR_PTR(-EINVAL);
		}

		if (fwnode_property_read_u32(child, "mgurdy,long-code",
					     &button->long_code)) {
			dev_err(dev, "Button without long-press keycode\n");
			fwnode_handle_put(child);
			return ERR_PTR(-EINVAL);
		}

		fwnode_property_read_string(child, "label", &button->desc);

		if (fwnode_property_read_u32(child, "debounce-interval",
					 &button->debounce_interval))
			button->debounce_interval = 5;

		button->longpress_delay = msecs_to_jiffies(longpress_msecs);

		button++;
	}

	return pdata;
}

static const struct of_device_id gpio_keys_of_match[] = {
	{ .compatible = "mgurdy-gpio-keys", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_keys_of_match);

static int gpio_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct mg_gpio_keys_platform_data *pdata = dev_get_platdata(dev);
	struct fwnode_handle *child = NULL;
	struct gpio_keys_drvdata *ddata;
	struct input_dev *input;
	size_t size;
	int i, error;

	if (!pdata) {
		pdata = gpio_keys_get_devtree_pdata(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	size = sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data);
	ddata = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!ddata) {
		dev_err(dev, "failed to allocate state\n");
		return -ENOMEM;
	}

	ddata->keymap = devm_kcalloc(dev,
				     pdata->nbuttons * 3,
				     sizeof(ddata->keymap[0]),
				     GFP_KERNEL);
	if (!ddata->keymap)
		return -ENOMEM;

	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	ddata->pdata = pdata;
	ddata->input = input;

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdata->name ? : pdev->name;
	input->phys = "mgurdy-gpio-keys/input0";
	input->dev.parent = dev;
	input->open = gpio_keys_open;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	input->keycode = ddata->keymap;
	input->keycodesize = sizeof(ddata->keymap[0]);
	input->keycodemax = pdata->nbuttons * 3;

	for (i = 0; i < pdata->nbuttons; i++) {
		const struct mg_gpio_keys_button *button = &pdata->buttons[i];

		if (!dev_get_platdata(dev)) {
			child = device_get_next_child_node(dev, child);
			if (!child) {
				dev_err(dev,
					"missing child device node for entry %d\n",
					i);
				return -EINVAL;
			}
		}

		error = gpio_keys_setup_key(pdev, input, ddata,
					    button, i, child);
		if (error) {
			fwnode_handle_put(child);
			return error;
		}
	}

	fwnode_handle_put(child);

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		return error;
	}

	return 0;
}

static int __maybe_unused gpio_keys_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused gpio_keys_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(gpio_keys_pm_ops, gpio_keys_suspend, gpio_keys_resume);

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.driver		= {
		.name	= "mgurdy-gpio-keys",
		.pm	= &gpio_keys_pm_ops,
		.of_match_table = gpio_keys_of_match,
	}
};

static int __init gpio_keys_init(void)
{
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

late_initcall(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marcus Weseloh <marcus@weseloh.cc>");
MODULE_DESCRIPTION("MidiGurdy GPIO-based keys driver");
MODULE_ALIAS("platform:mgurdy-gpio-keys");

