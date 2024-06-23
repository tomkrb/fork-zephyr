/* seesaw.c - Driver for the adafruit Seesaw boards
 */

/*
 * Copyright (c) Trada AS
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Based on doc from:
 * https://learn.adafruit.com/adafruit-seesaw-atsamd09-breakout/reading-and-writing-data
 */

#define DT_DRV_COMPAT adafruit_seasaw

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "seesaw.h"

LOG_MODULE_REGISTER(SEESAW, CONFIG_SENSOR_LOG_LEVEL);

static inline int seesaw_reboot(const struct device *dev)
{
	const struct seesaw_config *config = dev->config;

	// if (i2c_reg_update_byte_dt(&config->i2c, LSM6DS0_REG_CTRL_REG8,
	// 			   LSM6DS0_MASK_CTRL_REG8_BOOT,
	// 			   1 << LSM6DS0_SHIFT_CTRL_REG8_BOOT) < 0) {
	// 	return -EIO;
	// }

	// k_busy_wait(USEC_PER_MSEC * 50U);

	return 0;
}


static int seesaw_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	// __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL ||
	// 		chan == SENSOR_CHAN_ACCEL_XYZ ||
	// 		chan == SENSOR_CHAN_GYRO_XYZ);

	// switch (chan) {
	// case SENSOR_CHAN_ACCEL_XYZ:
	// 	lsm6ds0_sample_fetch_accel(dev);
	// 	break;
	// case SENSOR_CHAN_GYRO_XYZ:
	// 	lsm6ds0_sample_fetch_gyro(dev);
	// 	break;
	// case SENSOR_CHAN_ALL:
	// 	lsm6ds0_sample_fetch_accel(dev);
	// 	lsm6ds0_sample_fetch_gyro(dev);
	// 	break;
	// default:
	// 	return -ENOTSUP;
	// }

	return 0;
}

static inline void seesaw_accel_convert(struct sensor_value *val, int raw_val,
					 float scale)
{
	double dval;

	dval = (double)(raw_val) * (double)scale / 32767.0;
	val->val1 = (int32_t)dval;
	val->val2 = ((int32_t)(dval * 1000000)) % 1000000;
}

static int seesaw_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	// struct seesaw_data *data = dev->data;

	// switch (chan) {
	// case SENSOR_CHAN_ACCEL_X:
	// case SENSOR_CHAN_ACCEL_Y:
	// case SENSOR_CHAN_ACCEL_Z:
	// case SENSOR_CHAN_ACCEL_XYZ:
	// 	lsm6dso_accel_channel_get(chan, val, data);
	// 	break;
	// case SENSOR_CHAN_GYRO_X:
	// case SENSOR_CHAN_GYRO_Y:
	// case SENSOR_CHAN_GYRO_Z:
	// case SENSOR_CHAN_GYRO_XYZ:
	// 	lsm6dso_gyro_channel_get(chan, val, data);
	// 	break;
	// default:
	// 	return -ENOTSUP;
	// }

	return 0;
}


static const struct sensor_driver_api seesaw_api_funcs = {
	.sample_fetch = seesaw_sample_fetch,
	.channel_get = seesaw_channel_get,
};

static int lsm6ds0_init_chip(const struct device *dev)
{
	// const struct seesaw_config *config = dev->config;
	// uint8_t chip_id;

	// if (seesaw_reboot(dev) < 0) {
	// 	LOG_DBG("failed to reboot device");
	// 	return -EIO;
	// }

	// if (i2c_reg_read_byte_dt(&config->i2c, LSM6DS0_REG_WHO_AM_I, &chip_id) < 0) {
	// 	LOG_DBG("failed reading chip id");
	// 	return -EIO;
	// }
	// if ((chip_id != LSM6DS0_VAL_WHO_AM_I) && (chip_id != LSM6DSOX_VAL_WHO_AM_I)) {
	// 	LOG_DBG("invalid chip id 0x%x", chip_id);
	// 	return -EIO;
	// }
	// LOG_DBG("chip id 0x%x", chip_id);

	return 0;
}

static int seesaw_init(const struct device *dev)
{
	const struct seesaw_config * const config = dev->config;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}

	if (seesaw_init_chip(dev) < 0) {
		LOG_DBG("failed to initialize chip");
		return -EIO;
	}

	return 0;
}

#define SEESAW_DEFINE(inst)								\
	static struct seesaw_data seesaw_data_##inst;					\
											\
	static const struct seesaw_config seesaw_config_##inst = {			\
		.i2c = I2C_DT_SPEC_INST_GET(inst),					\
	};										\
											\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, seesaw_init, NULL,				\
			      &seesaw_data_##inst, &seesaw_config_##inst, POST_KERNEL,\
			      CONFIG_SENSOR_INIT_PRIORITY, &seesaw_api_funcs);		\

DT_INST_FOREACH_STATUS_OKAY(SEESAW_DEFINE)
