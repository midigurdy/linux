/*
 * MidiGurdy Keyboard Driver
 *
 * Copyright (C) 2017 Marcus Weseloh
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define DRV_NAME "mgurdy-keys"
/* MG_NUM_KEYS *must* be a multiple of 4 */
#define MG_NUM_KEYS (24)
#define MG_SCANS (MG_NUM_KEYS / 4)
#define MG_SCAN_SIZE MG_NUM_KEYS
#define MG_KEY_DEBOUNCE (2);

#define MG_ADC_IGNORE_BITS (1)
#define MG_ADC_OFFSET (50)

/* Module parameters */
static int clk_div = 3;
static int fs_div = 8;
static int first_delay = 0;
static int tacq = 11;
static int adc_offset = MG_ADC_OFFSET;
static int adc_shift = MG_ADC_IGNORE_BITS;
static int debounce_count = MG_KEY_DEBOUNCE;

static volatile int ISR_ERROR = 0;
static volatile int ISR_ERROR_DATA = 0;

module_param(fs_div, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(fs_div, "FS_DIV");
module_param(clk_div, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(clk_div, "CLK_DIV");
module_param(first_delay, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(first_delay, "FIRST_DELAY");
module_param(tacq, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(tacq, "T_ACQ");
module_param(adc_offset, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(adc_offset, "ADC_OFFSET");
module_param(adc_shift, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(adc_shift, "ADC_SHIFT");
module_param(debounce_count, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(debounce_count, "DEBOUNCE_COUNT");


/* ADC defines */
#define TP_CTRL0		0x00
#define TP_CTRL1		0x04
#define TP_INT_FIFOC		0x10
#define TP_INT_FIFOS		0x14
#define TP_DATA			0x24

/* TP_CTRL0 bits */
#define ADC_FIRST_DLY(x)	((x) << 24) /* 8 bits */
#define ADC_FIRST_DLY_MODE(x)	((x) << 23)
#define ADC_CLK_SEL(x)		((x) << 22)
#define ADC_CLK_DIV(x)		((x) << 20) /* 3 bits */
#define FS_DIV(x)		((x) << 16) /* 4 bits */
#define T_ACQ(x)        ((x) << 0) /* 16 bits */

/* TP_CTRL1 bits */
#define TP_MODE_EN(x)		((x) << 4)
#define TP_ADC_SELECT(x)	((x) << 3)
#define ADC_CHAN_SELECT(x)	((x) << 0)  /* 3 bits */
#define ADC_CHAN_MASK		(0x7)

/* TP_INT_FIFOC irq and fifo mask / control bits */
#define DATA_IRQ_EN(x)		((x) << 16)
#define FIFO_TRIG(x)		((x - 1) << 8)  /* 5 bits */
#define FIFO_FLUSH(x)		((x) << 4)

/* TP_INT_FIFOS irq and fifo status bits */
#define FIFO_DATA_PENDING	BIT(16)
#define FIFO_DATA_CNT_MASK	(0x1F)
#define FIFO_DATA_CNT(x)	(((x) >> 8) & FIFO_DATA_CNT_MASK)


#define KEY_PRESSED		(1)
#define KEY_RELEASED		(2)


/* Private data struct */

struct mgurdy_key {
	int debounce;
	int pressure;
	int prev_pressure;

	int pressed;
	int code;
};

struct mgurdy_keys_data {
	void __iomem		*adc;
	struct gpio_desc	*mux_sel_gpios[3];
	struct gpio_desc	*mux_en_gpio;
	struct input_dev	*input_dev;

	struct mgurdy_key       keys[MG_NUM_KEYS];

	volatile int		scan_round;
	volatile int		scan_ignore;

	struct tasklet_struct	task;
	DECLARE_KFIFO_PTR(fifo, int);
};

/* Linearization of sensor values */
static int _lin_in[11] = {
	0, 800, 1390, 1640, 1720, 1800, 1860, 1890, 1915, 1925, 1930
};

static int _lin_out[11] = {
	0, 300,  600,  900, 1200, 1500, 1800, 2100, 2400, 2700, 3000
};

static int mgurdy_keys_linearize(int val)
{
	int i = 0;

	if (val <= _lin_in[0])
		return _lin_out[0];
	if (val >= _lin_in[10])
		return _lin_out[10];

	while (val > _lin_in[i])
		i++;

	if (val == _lin_in[i])
		return _lin_out[i];

	return ((val - _lin_in[i-1]) * 300 /
		(_lin_in[i] - _lin_in[i-1]) + _lin_out[i-1]);
}

static void mgurdy_setup_keys(struct mgurdy_keys_data *mg)
{
	int i, k;

	memset(mg->keys, 0, sizeof(mg->keys));

	/* used to translate internal key index to real key number */
	for (i=0, k=0; i < MG_SCANS; i++) {
		mg->keys[k++].code = 0 + i;
		mg->keys[k++].code = 12 + i;
		mg->keys[k++].code = 6 + i;
		mg->keys[k++].code = 18 + i;
	}
}

/* Multiplexer control */

static void mgurdy_keys_mux_enable(struct mgurdy_keys_data *mg, bool enable)
{
	gpiod_set_value(mg->mux_en_gpio, enable);
}


static void mgurdy_keys_set_mux(struct mgurdy_keys_data *mg, int val)
{
	int i;
	int values[3];

	for (i=0; i < 3; i++)
		values[i] = !!(val & (1 << i));

	mgurdy_keys_mux_enable(mg, 0);
	gpiod_set_array_value(3, mg->mux_sel_gpios, values);
	mgurdy_keys_mux_enable(mg, 1);
}


/* ADC functions */

static int mgurdy_keys_adc_configure(struct mgurdy_keys_data *mg)
{
	int clk_hz, sample_hz, sample_us, conv_hz, conv_us;
	int tacq_us, delay_us;
	int max_scan_us, max_scan_hz;

	/* always use HOSC as parent clock (24Mhz) */

	switch(clk_div) {
		case 0: clk_hz = 12000000; break;
		case 1: clk_hz = 8000000; break;
		case 2: clk_hz = 4000000; break;
		case 3: clk_hz = 24000000; break;
		default:
			printk("invalid clk_div\n");
			return -1;
	}

	conv_hz = clk_hz / 13;
	conv_us = 1000000 / conv_hz;

	sample_hz = clk_hz / (1 << (20 - fs_div));
	sample_us = 1000000 / sample_hz;

	delay_us = (1000000 / (clk_hz / 16)) * first_delay;
	tacq_us = 1000000 / (clk_hz / (16 * (tacq + 1)));

	max_scan_us = ((tacq_us + conv_us) * 4) + delay_us;
	max_scan_hz = 1000000 / max_scan_us;

	printk("MidiGurdy ADC timing parameters:\n"
	       "   ADC Clock: %dhz (%d)\n"
	       "   First Delay: %dus (%d)\n"
	       "   Single Conversion: %dus\n"
	       "   Aquire Time: %dus (%d)\n"
	       "   => 4 Conversions:   %6dhz, %6dus\n"
	       "   => Sample Freq:     %6dhz, %6dus\n"
	       "   ADC Offset: %d\n"
	       "   ADC Shift Bits: %d\n",
	       clk_hz, clk_div,
	       delay_us, first_delay,
	       conv_us,
	       tacq_us, tacq,
	       max_scan_hz, max_scan_us,
	       sample_hz, sample_us,
	       adc_offset,
	       adc_shift
	);

	if (sample_us <= max_scan_us) {
		printk("error: sample time is less than conversion time\n");
		return -1;
	}

	// Configure clocks, sample rate, sampling time, first delay
	writel(ADC_CLK_SEL(0) | ADC_CLK_DIV(clk_div) | FS_DIV(fs_div) |
	       ADC_FIRST_DLY(first_delay) | T_ACQ(tacq),
	       mg->adc + TP_CTRL0);

	return 0;
}

static void mgurdy_keys_adc_enable(struct mgurdy_keys_data *mg)
{
	// Enable ADC, choose AUX mode and select round-robin channel
	writel(TP_MODE_EN(1) | TP_ADC_SELECT(1) | ADC_CHAN_SELECT(4),
	       mg->adc + TP_CTRL1);
}

static void mgurdy_keys_adc_irq_enable(struct mgurdy_keys_data *mg)
{

	/* Enable data IRQ and set trigger level to 4 */
	writel(DATA_IRQ_EN(1) | FIFO_TRIG(4) | FIFO_FLUSH(1),
	       mg->adc + TP_INT_FIFOC);
}

static void mgurdy_keys_adc_irq_disable(struct mgurdy_keys_data *mg)
{
	/* disable all IRQs */
	writel(0, mg->adc + TP_INT_FIFOC);
}

static void mgurdy_keys_adc_disable(struct mgurdy_keys_data *mg)
{
	/* disable ADC */
	writel(0, mg->adc + TP_CTRL1);
}

static void mgurdy_keys_adc_flush_fifo(struct mgurdy_keys_data *mg)
{
	u32 reg = readl(mg->adc + TP_INT_FIFOC);
	writel(reg | FIFO_FLUSH(1), mg->adc + TP_INT_FIFOC);
}

void mgurdy_keys_adc_bh(unsigned long data)
{
	struct mgurdy_keys_data *mg = (struct mgurdy_keys_data *)data;
	int i, cnt, val, action;
	int raw_vals[MG_SCAN_SIZE];
	struct mgurdy_key *key;
	int do_sync = 0;

	cnt = kfifo_out(&mg->fifo, raw_vals, MG_SCAN_SIZE);
	if (cnt != MG_SCAN_SIZE) {
		dev_err(&mg->input_dev->dev, "Got %d key values, expected %d\n",
			cnt, MG_SCAN_SIZE);
	}

	/* Key values are in groups of four */
	for (i=0; i < cnt; i++) {
		key = &mg->keys[i];

		/* throw away noisy bit(s) and adjust for offset voltage */
		val = (raw_vals[i] >> adc_shift) - adc_offset;
		if (val <= 0) {
			val = 0;
		}
		else {
			val = mgurdy_keys_linearize(val);
		}

		/* Debounce keys and determine if the state of the key changed.
		 * If changing to pressed state, the reported pressure is the
		 * maximum of the debounced pressure values. If changing to
		 * released state, reported pressure is always 0.
		 */
		action = 0;
		if (key->pressed) {
			if (!val) {
				key->debounce++;
			}
			else {
				key->debounce = 0;
			}

			if (key->debounce > debounce_count) {
				action = KEY_RELEASED;
				key->pressed = 0;
				key->pressure = 0;
			}
			else {
				key->pressure = val;
			}
		}
		else {
			if (val) {
				key->debounce++;
				key->pressure = max(key->pressure, val);
			}
			else {
				key->debounce = 0;
				key->pressure = 0;
			}

			if (key->debounce > debounce_count) {
				action = KEY_PRESSED;
				key->pressed = 1;
			}
		}

		if (action || (key->pressed
			       && key->pressure != 0
			       && key->pressure != key->prev_pressure)) {
			input_report_abs(mg->input_dev, key->code,
					 key->pressure);
			do_sync = 1;
			key->prev_pressure = key->pressure;
		}
	}

	if (do_sync) {
		input_sync(mg->input_dev);
	}
}

static irqreturn_t mgurdy_keys_adc_isr(int irq, void *dev_id)
{
	struct mgurdy_keys_data *mg = (struct mgurdy_keys_data *)dev_id;
	u32 reg;

	u32 tmp[4];
	int has_data = 0;
	int scan_complete = 0;
	int cnt;
	int i, written;

	reg = readl(mg->adc + TP_INT_FIFOS);

	if (reg & FIFO_DATA_PENDING) {
		cnt = FIFO_DATA_CNT(reg);

		/* If we have at least one full block of round-robin
		 * data, then switch MUX immediately */
		if ((cnt - mg->scan_ignore) >= 4) {
			mg->scan_round = (mg->scan_round + 1) % 6;
			mgurdy_keys_set_mux(mg, mg->scan_round);
			if (mg->scan_round == 0)
				scan_complete = 1;
		}

		/* Throw away data we need to ignore to reach the next
		 * start of a round-robin block */
		while (mg->scan_ignore > 0 && cnt > 0) {
			readl(mg->adc + TP_DATA);
			cnt--;
			mg->scan_ignore--;
		}

		/* Read all full blocks of data but keep only the last one. The
		 * last one is furthest away from the MUX switch. */
		while (cnt >= 4) {
			for (i=0; i < 4; i++) {
				tmp[i] = readl(mg->adc + TP_DATA);
			}
			cnt -= 4;
			has_data = 1;
		}

		if (has_data) {
			for (i=0; i<4; i++) {
				written = kfifo_put(&mg->fifo, tmp[i]);
				if (written != 1) {
					mgurdy_keys_adc_irq_disable(mg);
					printk("Unable to write data to KFIFO!!\n");
					goto clear_irq;
				}
			}
		}

		/* Check the FIFO contents again. If there is more data,
		 * then throw that away as well. It's either additional
		 * data from the current invocation, or additional data
		 * that as been read while we switched the MUX. */
		reg = readl(mg->adc + TP_INT_FIFOS);
		cnt = FIFO_DATA_CNT(reg);
		for (i=0; i<cnt; i++) {
			readl(mg->adc + TP_DATA);
		}

		/* Make sure we ignore missing data points from the last
		 * round-robin group during next invocation */
		mg->scan_ignore = (4 - (cnt % 4)) % 4;

		if (scan_complete) {
			tasklet_hi_schedule(&mg->task);
		}
	}

clear_irq:
	writel(reg, mg->adc + TP_INT_FIFOS);

	return IRQ_HANDLED;
}


/* Input device handling */

static int mgurdy_keys_open(struct input_dev *dev)
{
	struct mgurdy_keys_data *mg = input_get_drvdata(dev);

	kfifo_reset(&mg->fifo);

	mgurdy_setup_keys(mg);

	mg->scan_round = 0;
	mg->scan_ignore = 0;

	mgurdy_keys_adc_flush_fifo(mg);
	mgurdy_keys_set_mux(mg, 0);

	/* Wait a little until FIFO is flushed. Otherwise we get DATA interrupts
	 * with invalid count numbers */
	udelay(10);

	mgurdy_keys_adc_irq_enable(mg);
	mgurdy_keys_adc_enable(mg);

        return 0;
}

static void mgurdy_keys_close(struct input_dev *dev)
{
	struct mgurdy_keys_data *mg = input_get_drvdata(dev);

	mgurdy_keys_mux_enable(mg, 0);
	mgurdy_keys_adc_irq_disable(mg);
	mgurdy_keys_adc_disable(mg);
}


/* Platform device handling */

static int mgurdy_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct mgurdy_keys_data *mg = dev_get_platdata(dev);
	struct resource *mem;
	int i, error, irq;

	if (!np) {
		dev_err(dev, "device lacks DT data\n");
		return -ENODEV;
	}

	mg = devm_kzalloc(dev, sizeof(struct mgurdy_keys_data), GFP_KERNEL);
	if (!mg)
		return -ENOMEM;
	platform_set_drvdata(pdev, mg);


	/* ADC */

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mg->adc = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(mg->adc)) {
		dev_err(dev, "unable to map adc memory\n");
		return PTR_ERR(mg->adc);
	}

	irq = platform_get_irq(pdev, 0);
	error = devm_request_irq(&pdev->dev, irq, mgurdy_keys_adc_isr,
				 0, dev_name(&pdev->dev), mg);
	if (error) {
		dev_err(&pdev->dev, "failed requesting irq %d\n", irq);
		return error;
	}

	tasklet_init(&mg->task, mgurdy_keys_adc_bh, (unsigned long) mg);

	if (mgurdy_keys_adc_configure(mg)) {
		dev_err(dev, "unable to configure adc timings\n");
		return -ENODEV;
	}


	/* GPIOs */

	mg->mux_en_gpio = devm_gpiod_get(dev, "mux-en", GPIOD_OUT_LOW);
	if (IS_ERR(mg->mux_en_gpio)) {
		dev_err(dev, "invalid mux-en gpio\n");
		return -ENOENT;
	}

	for (i = 0; i < 3; i++) {
		mg->mux_sel_gpios[i] = devm_gpiod_get_index(dev, "mux-sel", i,
							    GPIOD_OUT_LOW);
		if (IS_ERR(mg->mux_sel_gpios[i])) {
			dev_err(dev, "mux-sel gpio %d invalid\n", i);
			return -ENOENT;
		}
	}


	/* Input device */

	mg->input_dev = devm_input_allocate_device(dev);
	if (!mg->input_dev) {
		dev_err(dev, "not enough memory for input device\n");
		return -ENOMEM;
	}
	mg->input_dev->name = pdev->name;
	mg->input_dev->id.bustype = BUS_HOST;
	mg->input_dev->dev.parent = dev;
	mg->input_dev->open = mgurdy_keys_open;
	mg->input_dev->close = mgurdy_keys_close;

	input_set_drvdata(mg->input_dev, mg);

	/* first 24 codes are key pressure, second 24 codes are velocity */
	for(i = 0; i < MG_NUM_KEYS * 2; i++) {
		input_set_capability(mg->input_dev, EV_ABS, i);
	}

	/* allocate enough space for 10 scan rounds, just in case our bottom
	 * half gets delayed */
	error = kfifo_alloc(&mg->fifo, MG_SCAN_SIZE * 10, GFP_KERNEL);
	if (error) {
		dev_err(&pdev->dev, "failed allocating kfifo\n");
		return error;
	}

	error = input_register_device(mg->input_dev);
	if (error) {
		dev_err(dev, "could not register input device\n");
		kfifo_free(&mg->fifo);
		return error;
	}

	dev_info(dev, "mgurdy-keys loaded\n");

	return 0;
}

static int mgurdy_keys_remove(struct platform_device *pdev)
{
	struct mgurdy_keys_data *mg = platform_get_drvdata(pdev);

	input_unregister_device(mg->input_dev);
	kfifo_free(&mg->fifo);

	return 0;
}

static const struct of_device_id mgurdy_keys_of_match[] = {
    { .compatible = "mgurdy,keyboard", },
    { },
};
MODULE_DEVICE_TABLE(of, mgurdy_keys_of_match);

static struct platform_driver mgurdy_keys_device_driver = {
	.probe		= mgurdy_keys_probe,
	.remove		= mgurdy_keys_remove,
	.driver		= {
		.name	= DRV_NAME,
		.of_match_table = of_match_ptr(mgurdy_keys_of_match),
	}
};
module_platform_driver(mgurdy_keys_device_driver);

MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DESCRIPTION("MidiGurdy keyboard driver");
MODULE_AUTHOR("Marcus Weseloh <marcus@weseloh.cc>");
MODULE_LICENSE("GPL v2");
