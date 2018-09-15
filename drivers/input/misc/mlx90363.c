/*
 * MLX90363 Rotary Position Sensor IC
 *
 * Copyright (C) 2016 Marcus Weseloh
 * Licensed under the GPL-2 or later.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/string.h>

#define DRV_NAME "mlx90363"

#define MLX90363_DELAY_US		(920)
#define MLX90363_SPI_FREQ_HZ		(1300000)
#define MLX90363_SPI_FREQ_MAX_HZ	(2000000)
#define MLX90363_RES			(16384)
#define MLX90363_FILTER			(0)
#define MLX90363_HYST			(3)

#define ABS_POS (0)
#define ABS_DIST (1)
#define ABS_GAIN (2)
#define REL_TIME  (1)

/* for eeprom reading and writing */
#define READ16(X, Y) ((u16)(Y << 8) | X)
#define HB(X)  ((X >> 8) & 0xFF)
#define LB(X)  (X & 0xFF)

#define MSG_MEMORY_READ		(0x01)
#define MSG_MEMORY_READ_RET	(0x02)

#define MSG_EEWRITE		(0x03)
#define MSG_EEWRITE_CHAL	(0x04)
#define MSG_EEWRITE_CHAL_ANS	(0x05)

#define MSG_EEREAD_RET		(0x28)
#define MSG_EEREAD_CHAL		(0x0F)
#define MSG_EEWRITE_STAT	(0x0E)

#define MSG_NOP			(0x10)
#define MSG_NOP_RET		(0x11)

#define CMD_MARKER		(0xC0)


/* module parameters */
static int read_delay_us = MLX90363_DELAY_US;
static int spi_freq_hz = MLX90363_SPI_FREQ_HZ;
static int mlx_filter = MLX90363_FILTER;
static int mlx_hyst = MLX90363_HYST;

module_param(read_delay_us, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(read_delay_us, "READ_DELAY_US");
module_param(spi_freq_hz, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(spi_freq_hz, "SPI_FREQ_HZ");
module_param(mlx_filter, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(mlx_filter, "MLX_FILTER");
module_param(mlx_hyst, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(mlx_hyst, "MLX_HYST");


struct mlx90363 {
	struct input_dev	*input;

	struct spi_device	*spi;
	struct spi_message      msg;
	struct spi_transfer     tfr;
	u8			tx[8];
	u8			rx[8];

	s64			times[2];

	int			angle;
	int			dist;
	int                     gain;

	int			should_stop;
	struct hrtimer		timer;
};

/* CRC calculations according to MLX90363 datasheet */

static u8 mlx90363_crc[] = {
	0x00, 0x2f, 0x5e, 0x71, 0xbc, 0x93, 0xe2, 0xcd,
	0x57, 0x78, 0x09, 0x26, 0xeb, 0xc4, 0xb5, 0x9a,
	0xae, 0x81, 0xf0, 0xdf, 0x12, 0x3d, 0x4c, 0x63,
	0xf9, 0xd6, 0xa7, 0x88, 0x45, 0x6a, 0x1b, 0x34,
	0x73, 0x5c, 0x2d, 0x02, 0xcf, 0xe0, 0x91, 0xbe,
	0x24, 0x0b, 0x7a, 0x55, 0x98, 0xb7, 0xc6, 0xe9,
	0xdd, 0xf2, 0x83, 0xac, 0x61, 0x4e, 0x3f, 0x10,
	0x8a, 0xa5, 0xd4, 0xfb, 0x36, 0x19, 0x68, 0x47,
	0xe6, 0xc9, 0xb8, 0x97, 0x5a, 0x75, 0x04, 0x2b,
	0xb1, 0x9e, 0xef, 0xc0, 0x0d, 0x22, 0x53, 0x7c,
	0x48, 0x67, 0x16, 0x39, 0xf4, 0xdb, 0xaa, 0x85,
	0x1f, 0x30, 0x41, 0x6e, 0xa3, 0x8c, 0xfd, 0xd2,
	0x95, 0xba, 0xcb, 0xe4, 0x29, 0x06, 0x77, 0x58,
	0xc2, 0xed, 0x9c, 0xb3, 0x7e, 0x51, 0x20, 0x0f,
	0x3b, 0x14, 0x65, 0x4a, 0x87, 0xa8, 0xd9, 0xf6,
	0x6c, 0x43, 0x32, 0x1d, 0xd0, 0xff, 0x8e, 0xa1,
	0xe3, 0xcc, 0xbd, 0x92, 0x5f, 0x70, 0x01, 0x2e,
	0xb4, 0x9b, 0xea, 0xc5, 0x08, 0x27, 0x56, 0x79,
	0x4d, 0x62, 0x13, 0x3c, 0xf1, 0xde, 0xaf, 0x80,
	0x1a, 0x35, 0x44, 0x6b, 0xa6, 0x89, 0xf8, 0xd7,
	0x90, 0xbf, 0xce, 0xe1, 0x2c, 0x03, 0x72, 0x5d,
	0xc7, 0xe8, 0x99, 0xb6, 0x7b, 0x54, 0x25, 0x0a,
	0x3e, 0x11, 0x60, 0x4f, 0x82, 0xad, 0xdc, 0xf3,
	0x69, 0x46, 0x37, 0x18, 0xd5, 0xfa, 0x8b, 0xa4,
	0x05, 0x2a, 0x5b, 0x74, 0xb9, 0x96, 0xe7, 0xc8,
	0x52, 0x7d, 0x0c, 0x23, 0xee, 0xc1, 0xb0, 0x9f,
	0xab, 0x84, 0xf5, 0xda, 0x17, 0x38, 0x49, 0x66,
	0xfc, 0xd3, 0xa2, 0x8d, 0x40, 0x6f, 0x1e, 0x31,
	0x76, 0x59, 0x28, 0x07, 0xca, 0xe5, 0x94, 0xbb,
	0x21, 0x0e, 0x7f, 0x50, 0x9d, 0xb2, 0xc3, 0xec,
	0xd8, 0xf7, 0x86, 0xa9, 0x64, 0x4b, 0x3a, 0x15,
	0x8f, 0xa0, 0xd1, 0xfe, 0x33, 0x1c, 0x6d, 0x42,
};

/* "secret" keys for eeprom write operations */
static u16 mlx90363_eeprom_keys[32] = {
	17485, 31053, 57190, 57724, 7899, 53543, 26763, 12528,
	38105, 51302, 16209, 24847, 13134, 52339, 14530, 18350,
	55636, 64477, 40905, 45498, 24411, 36677, 4213, 48843,
	6368, 5907, 31384, 63325, 3562, 19816, 6995, 3147,
};

static u8 mlx90363_calc_crc(u8 msg[8])
{
	u8 crc = 0xFF;
	int i;

	for (i=0; i<7; i++)
		crc = mlx90363_crc[msg[i] ^ crc];

	return ~crc;
}

static void mlx90363_trigger_read(struct mlx90363 *mlx)
{
	int err;

	err = spi_async(mlx->spi, &mlx->msg);
	if (err) {
		dev_err(&mlx->spi->dev, "unable to send message\n");
		return;
	}
}


static enum hrtimer_restart mlx90363_timer_callback(struct hrtimer *tmr)
{
	struct mlx90363 *mlx = container_of(tmr, struct mlx90363, timer);

	mlx90363_trigger_read(mlx);

	return HRTIMER_NORESTART;
}


/* called as SPI complete callback */
static void mlx90363_report_angle(void *context)
{
	struct mlx90363 *mlx = context;
	u8 crc, marker, gain;
	s64 time, delay_ns;
	ktime_t delay_period;
	u16 angle;
	int dist = 0;
	int delta_us = 0;
	int do_sync = 0;

	time = ktime_get_ns();

	crc = mlx90363_calc_crc(mlx->rx);
	if (crc != mlx->rx[7]) {
		dev_err(&mlx->spi->dev, "CRC error!\n");
		goto timer_restart;
	}

	/* If this is not a GET1 response, just ignore it */
	marker = (mlx->rx[6] >> 6) & 0x03;
	if (marker != 0) {
		dev_dbg(&mlx->spi->dev, "Got response with marker %d\n",
			marker);
		goto timer_restart;
	}

	angle = mlx->rx[1] & 0x3F;
	angle <<= 8;
	angle |= mlx->rx[0];

	gain = mlx->rx[4];

	/* Prepare to send angle and/or distance if the angle has changed or the
	 * last reported distance is not zero. Otherwise we don't report
	 * if the distance falls to zero. */
	if (mlx->angle != angle || mlx->dist != 0) {
		/* Calculate the elapsed time between the last and the current
		 * angle.  We need to compare the times of the previous two
		 * readings, as the reported angle is the one from the previous
		 * request */

		if (mlx->times[0] && mlx->times[1]) {
			delta_us = (int)(mlx->times[1] - mlx->times[0]) / 1000;
		}

		/* If we have a time delta, then calculate the travelled distance
		 * since last reading */
		if (delta_us > 0) {
			dist = (angle - mlx->angle);

			/* Handle zero crossing */
			if (dist < -MLX90363_RES/2)
				dist += MLX90363_RES;
			else if (dist > MLX90363_RES/2)
				dist -= MLX90363_RES;
		}

		if (mlx->dist != dist) {
			input_report_abs(mlx->input, ABS_DIST, dist);
			mlx->dist = dist;
			do_sync = 1;
		}

		if (mlx->angle != angle) {
			input_report_abs(mlx->input, ABS_POS, angle);
			mlx->angle = angle;
			do_sync = 1;
		}

		/* If we send angle and/or distance, we always send duration
		 * time as well */
		if (do_sync)
			input_event(mlx->input, EV_MSC, REL_TIME, delta_us);
	}

	if (mlx->gain != gain) {
		input_report_abs(mlx->input, ABS_GAIN, gain);
		do_sync = 1;
	}

	if (do_sync)
		input_sync(mlx->input);

	// push current time onto list, removing the oldest time
	mlx->times[0] = mlx->times[1];
	mlx->times[1] = time;

timer_restart:
	if (mlx->should_stop)
		return;

	// check how much time has passed while processing the results and
	// adjust the next wait period accordingly
	delay_ns = read_delay_us * 1000 - (ktime_get_ns() - time);

	// trigger next spi transfer immediately if we're late already
	if (delay_ns <= 0) {
		mlx90363_trigger_read(mlx);
	}
	else {
		delay_period = ktime_set(0, delay_ns);
		hrtimer_start(&mlx->timer, delay_period, HRTIMER_MODE_REL);
	}
}

static void mlx90363_setup_spi_message(struct mlx90363 *mlx)
{
	spi_message_init(&mlx->msg);
	mlx->msg.context = mlx;
	mlx->msg.complete = mlx90363_report_angle;
	mlx->tfr.speed_hz = spi_freq_hz;
	mlx->tfr.len = 8;
	mlx->tfr.tx_buf = mlx->tx;
	mlx->tfr.rx_buf = mlx->rx;

	memset(mlx->tx, 0, sizeof(mlx->tx));
	memset(mlx->rx, 0, sizeof(mlx->rx));

	/* pre-create GET1 message with maximum timeout */
	mlx->tx[2] = 0xFF;
	mlx->tx[3] = 0xFF;
	mlx->tx[6] = 0x13;
	mlx->tx[7] = mlx90363_calc_crc(mlx->tx);

	spi_message_add_tail(&mlx->tfr, &mlx->msg);
}


/* EEPROM reading and writing */
static int mlx90363_spi_cmd(struct mlx90363 *mlx, u8 cmd, u8 tx[6], u8 rx[6],
			    u8 chk_opcode)
{
	struct spi_message msg;
	struct spi_transfer tfr;
	u8 opcode;
	u8 tx_buf[8], rx_buf[8];
	u16 crc;
	int err, i;

	spi_message_init(&msg);

	memset(&tfr, 0, sizeof(tfr));
	memset(rx_buf, 0, sizeof(rx_buf));

	tfr.speed_hz = spi_freq_hz;
	tfr.tx_buf = tx_buf;
	tfr.rx_buf = rx_buf;
	tfr.len = 8;
	spi_message_add_tail(&tfr, &msg);

	for (i=0; i<6; i++)
		tx_buf[i] = tx[i];
	tx_buf[6] = CMD_MARKER | cmd;
	tx_buf[7] = mlx90363_calc_crc(tx_buf);

	err = spi_sync(mlx->spi, &msg);
	if (err) {
		dev_err(&mlx->spi->dev, "spi error sending %X command\n", cmd);
		return err;
	}

	crc = mlx90363_calc_crc(rx_buf);
	if (crc != rx_buf[7]) {
		dev_err(&mlx->spi->dev,
			"CRC error in %X command response\n", cmd);
		return -1;
	}

	/* optionally check response opcode */
	if (chk_opcode) {
		opcode = rx_buf[6] & 0x3F;
		if (opcode != chk_opcode) {
			dev_err(&mlx->spi->dev,
				"%X command got invalid response opcode: %X\n",
				cmd, opcode);
			return -1;
		}
	}

	for (i=0; i<6; i++)
		rx[i] = rx_buf[i];

	return 0;
}


static int mlx90363_eeread(struct mlx90363 *mlx, u16 addr)
{
	u8 tx[6], rx[6];

	/* do a NOP first, just to be on the safe side */
	memset(tx, 0, sizeof(tx));
	if (mlx90363_spi_cmd(mlx, MSG_NOP, tx, rx, 0))
		return -1;
	usleep_range(120, 500);

	/* MEMORY_READ command */
	memset(tx, 0, sizeof(tx));
	/* read command can read two addresses, we only need one so set both
	 * to same address */
	tx[0] = tx[2] = LB(addr);
	tx[1] = tx[3] = HB(addr);
	if (mlx90363_spi_cmd(mlx, MSG_MEMORY_READ, tx, rx, 0))
		return -1;

	/* chip needs 120us to fetch data */
	usleep_range(120, 500);

	/* NOP command, returns value from read command */
	memset(tx, 0, sizeof(tx));
	if (mlx90363_spi_cmd(mlx, MSG_NOP, tx, rx, MSG_MEMORY_READ_RET))
		return -1;

	usleep_range(120, 500);

	return READ16(rx[0], rx[1]);
}

static int mlx90363_eewrite(struct mlx90363 *mlx, u16 addr, u16 val)
{
	u8 tx[6], rx[6];
	u8 reg;
	u16 key, chal, chal_inv;
	int status, key_row, key_col;

	/* do a NOP first, just to be on the safe side */
	memset(tx, 0, sizeof(tx));
	if (mlx90363_spi_cmd(mlx, MSG_NOP, tx, rx, 0))
		return -1;
	usleep_range(120, 500);

	/* EepromWrite Command */
	reg = addr & 0x3F;
	key_row = (reg >> 4) & 0x3;
	key_col = (reg >> 1) & 0x7;
	key = mlx90363_eeprom_keys[(key_row * 8) + key_col];

	memset(tx, 0, sizeof(tx));
	tx[1] = reg;
	tx[2] = LB(key);
	tx[3] = HB(key);
	tx[4] = LB(val);
	tx[5] = HB(val);

	if (mlx90363_spi_cmd(mlx, MSG_EEWRITE, tx, rx, 0))
		return -1;

	usleep_range(120, 500);

	memset(tx, 0, sizeof(tx));

	if (mlx90363_spi_cmd(mlx, MSG_EEREAD_CHAL, tx, rx, MSG_EEWRITE_CHAL))
		return -1;

	chal = READ16(rx[2], rx[3]);
	chal ^= 0x1234;
	chal_inv = ~chal;

	usleep_range(120, 500);

	/* calculate challenge answer */
	memset(tx, 0, sizeof(tx));
	tx[2] = LB(chal);
	tx[3] = HB(chal);
	tx[4] = LB(chal_inv);
	tx[5] = HB(chal_inv);

	if (mlx90363_spi_cmd(mlx, MSG_EEWRITE_CHAL_ANS, tx, rx, MSG_EEREAD_RET))
		return -1;

	usleep_range(37000, 50000); // yes, eeprom needs 37ms!

	memset(tx, 0, sizeof(tx));
	if (mlx90363_spi_cmd(mlx, MSG_NOP, tx, rx, MSG_EEWRITE_STAT))
		return -1;

	usleep_range(120, 500);

	status = rx[0] & 0xF;
	if (status != 1) {
		dev_err(&mlx->spi->dev,
			"eeprom write failed, error code: %d\n", status);
		return -1;
	}

	return 0;
}

static int mlx90363_reg_write(struct mlx90363 *mlx, u16 addr, u16 val,
			      u16 shift, u16 mask)
{
	u16 reg;
	int ret;

	ret = mlx90363_eeread(mlx, addr);
	if (ret < 0) return -1;

	reg = (u16)ret;
	reg &= ~((u16) mask << shift);
	reg |= (u16)((val & mask) << shift);

	if (reg != (u16)ret)
		return mlx90363_eewrite(mlx, addr, reg);

	printk("Nothing to do!\n");
	return 0;
}


/* Input device handling */

static int mlx90363_open(struct input_dev *dev)
{
	struct mlx90363 *mlx = input_get_drvdata(dev);

	mlx->should_stop = 0;
	memset(mlx->times, 0, sizeof(mlx->times));

	mlx90363_trigger_read(mlx);

        return 0;
}

static void mlx90363_close(struct input_dev *dev)
{
	struct mlx90363 *mlx = input_get_drvdata(dev);

	mlx->should_stop = 1;
	hrtimer_cancel(&mlx->timer);
}


/* Module probing and setup */

static const struct of_device_id mlx90363_dt_ids[] = {
    { .compatible = "mlx90363", },
    { },
};
MODULE_DEVICE_TABLE(of, mlx90363_dt_ids);


static int mlx90363_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mlx90363	*mlx;
	struct input_dev *input_dev;
	struct device_node *node = dev->of_node;
	const struct of_device_id *match;
	int err;
	int retry;

	if (!node) {
		dev_err(dev, "Device does not have associated DT data\n");
		return -EINVAL;
	}

	match = of_match_device(mlx90363_dt_ids, dev);
	if (!match) {
		dev_err(dev, "Unknown device model\n");
		return -EINVAL;
	}

	/* don't exceed max specified SPI CLK frequency */
	if (spi->max_speed_hz > MLX90363_SPI_FREQ_MAX_HZ) {
		dev_dbg(dev, "Max allowed frequency is %d Hz!\n",
			spi->max_speed_hz);
		return -EINVAL;
	}

	spi->mode = SPI_MODE_1;
	spi->bits_per_word = 8;

	err = spi_setup(spi);
	if (err) {
		dev_dbg(dev, "SPI device setup failed\n");
		return err;
	}

	mlx = devm_kzalloc(dev, sizeof(struct mlx90363), GFP_KERNEL);
	if (!mlx)
		return -ENOMEM;
	memset(mlx, 0, sizeof(mlx));
	spi_set_drvdata(spi, mlx);
	mlx->spi = spi;

	input_dev = devm_input_allocate_device(dev);
	if (!input_dev)
		return -ENOMEM;


	// update filter and hysteresis parameters if necessary
	retry = 10;
	while (1) {
		if (mlx90363_reg_write(mlx, 0x102A, mlx_filter, 4, 0x3)) {
			retry--;
			if (retry == 0) {
				dev_err(dev,
					"Unable to set filter parameter!\n");
				return -EINVAL;
			}
			else {
				dev_err(dev, "Unable to set filter parameter, retrying...!\n");
			}
		} else {
			break;
		}
	}

	if (mlx90363_reg_write(mlx, 0x1028, mlx_hyst, 8, 0xFF)) {
		dev_err(dev, "Unable to set hysteresis parameter!\n");
		return -EINVAL;
	}

	input_dev->name = "MLX90363 Rotary Sensor";
	input_dev->dev.parent = dev;
	input_dev->id.bustype = BUS_HOST;
	input_dev->open = mlx90363_open;
	input_dev->close = mlx90363_close;

	input_set_drvdata(input_dev, mlx);
	mlx->input = input_dev;

	input_set_capability(input_dev, EV_ABS, ABS_POS);
        input_set_abs_params(input_dev, ABS_POS, 0, MLX90363_RES - 1, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_DIST);
        input_set_abs_params(input_dev, ABS_DIST,
			     -MLX90363_RES/2, MLX90363_RES/2, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_GAIN);
        input_set_abs_params(input_dev, ABS_GAIN, 0, 255, 0, 0);
	input_set_capability(input_dev, EV_MSC, REL_TIME);

	err = input_register_device(input_dev);
	if (err) {
		dev_err(dev, "unable to register input device\n");
		return err;
	}

	mlx90363_setup_spi_message(mlx);

	hrtimer_init(&mlx->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	mlx->timer.function = mlx90363_timer_callback;

	dev_info(dev, "MLX90363 driver loaded\n");

	return 0;
}

static int mlx90363_remove(struct spi_device *spi)
{
	struct mlx90363 *mlx = spi_get_drvdata(spi);

	hrtimer_cancel(&mlx->timer);
	input_unregister_device(mlx->input);

	dev_info(&spi->dev, "Unregistered MLX90363\n");

	return 0;
}

static struct spi_driver mlx90363_driver = {
	.driver = {
		.name	= DRV_NAME,
		.of_match_table = of_match_ptr(mlx90363_dt_ids),
	},
	.probe		= mlx90363_probe,
	.remove		= mlx90363_remove,
};

module_spi_driver(mlx90363_driver);

MODULE_ALIAS("spi:" DRV_NAME);
MODULE_AUTHOR("Marcus Weseloh <marcus@weseloh.cc>");
MODULE_DESCRIPTION("MLX90363 Rotary Position Sensor Driver");
MODULE_LICENSE("GPL v2");
