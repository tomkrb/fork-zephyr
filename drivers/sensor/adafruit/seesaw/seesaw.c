/* seesaw.c - Driver for the adafruit Seesaw boards
 */

/*
 * Copyright (c) 2024 Trada AS
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Based on doc from:
 * https://learn.adafruit.com/adafruit-seesaw-atsamd09-breakout/reading-and-writing-data
 */

#define DT_DRV_COMPAT adafruit_seesaw

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "seesaw.h"

LOG_MODULE_REGISTER(seesaw_sensor, CONFIG_SENSOR_LOG_LEVEL);

static int read_register(const struct device *dev, uint8_t channel, uint8_t reg, void *data, size_t size)
{
	const struct seesaw_config *config = dev->config;
	
	if (i2c_reg_write_byte_dt(&config->i2c, channel, reg)) {
		LOG_ERR("Failed to write channel 0x%02X, register 0x%02X", channel, reg);
		return -EIO;
	}
	k_sleep(K_USEC(50));
	if (i2c_read_dt(&config->i2c, (uint8_t *)data, size) < 0) {
		LOG_ERR("Failed to read %zu bytes from channel 0x%02X, register 0x%02X", size, channel, reg);
		return -EIO;
	}

	if (size == 4) {
		LOG_DBG("Read channel 0x%02X, register 0x%02X: 0x%08X", channel, reg, *(uint32_t *)data);
	} else if (size == 1) {
		LOG_DBG("Read channel 0x%02X, register 0x%02X: 0x%02X", channel, reg, *(uint8_t *)data);
	}
	return 0;
}

static int read_uint32_register(const struct device *dev, uint8_t channel, uint8_t reg, uint32_t *data)
{
	return read_register(dev, channel, reg, data, sizeof(uint32_t));
}

static int read_uint8_register(const struct device *dev, uint8_t channel, uint8_t reg, uint8_t *data)
{
	return read_register(dev, channel, reg, data, sizeof(uint8_t));
}

static inline int seesaw_init_chip(const struct device *dev)
{
	const struct seesaw_config *config = dev->config;
	struct seesaw_data *data = dev->data;

	LOG_DBG("Initializing seesaw device %s", config->i2c.bus->name);
	
	/* Set all registers to default values */
	if (i2c_reg_write_byte_dt(&config->i2c, SEESAW_STATUS_SWRST, 0xFF)) {
		LOG_ERR("Failed to reset seesaw device");
		return -EIO;
	}

	// Add 100 msec delay
 	if( read_uint32_register(dev, SEESAW_CHAN_STATUS, SEESAW_STATUS_OPTIONS, &data->options) < 0) {
		return -EIO;
	}

	if( read_uint32_register(dev, SEESAW_CHAN_STATUS, SEESAW_STATUS_VERSION, &data->version) < 0) {
		return -EIO;
	}	

	if( read_uint8_register(dev, SEESAW_CHAN_STATUS, SEESAW_STATUS_HW_ID, &data->hw_id) < 0) {
		return -EIO;
	}

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

uint32_t seesaw_temperature_get(const struct device *dev)
{
	uint32_t temp;

	if (read_uint32_register(dev, SEESAW_CHAN_STATUS, SEESAW_STATUS_TEMP, &temp) < 0) {
		return -EIO;
	}
	LOG_DBG("Temperature: %d", temp);
	return temp;
}

uint16_t seesaw_adc_channel_read(const struct device *dev, uint8_t channel)
{
	const struct seesaw_config *config = dev->config;
	uint8_t adc[2], reg;

	reg = SEESAW_ADC_FUNCTION_CHANNEL_0 + channel;

	if (channel >= SEESAW_ADC_NUMBER_OF_CHANNELS) {
		return -EINVAL;
	}

	if (i2c_write_read_dt(&config->i2c, &reg, 1, adc, 2) < 0) {
		return -EIO;
	}

	return sys_get_le16(adc);	
}

void i2c_write_32bit_register( const struct device *dev, uint8_t reg, uint32_t data)
{
	uint8_t buf[5];
	const struct seesaw_config *config = dev->config;

	buf[0] = reg;
	sys_put_le32(data, &buf[1]);

	i2c_write_dt(&config->i2c, buf, 5);
}

void seesaw_gpio_pin_set_input(const struct device *dev, uint8_t pin)
{
	uint32_t data = 1 << pin;

	i2c_write_32bit_register(dev, SEESAW_GPIO_FUNCTION_DIRCLR, data);
}

void seesaw_gpio_pin_set_output(const struct device *dev, uint8_t pin)
{
	const struct seesaw_config *config = dev->config;
	uint32_t data = 1 << pin;

	i2c_write_32bit_register(dev, SEESAW_GPIO_FUNCTION_DIRSET, data);
}

void seesaw_gpio_pin_set(const struct device *dev, uint8_t pin)
{
	const struct seesaw_config *config = dev->config;
	uint32_t data = 1 << pin;

	i2c_write_32bit_register(dev, SEESAW_GPIO_FUNCTION_SET, data);
}

void seesaw_gpio_pin_clear(const struct device *dev, uint8_t pin)
{
	const struct seesaw_config *config = dev->config;
	uint32_t data = 1 << pin;

	i2c_write_32bit_register(dev, SEESAW_GPIO_FUNCTION_CLR, data);
}

void seesaw_gpio_pin_toggle(const struct device *dev, uint8_t pin)
{
	const struct seesaw_config *config = dev->config;
	uint32_t data = 1 << pin;

	i2c_write_32bit_register(dev, SEESAW_GPIO_FUNCTION_TOGGLE, data);
}

uint32_t seesaw_gpio_sample_fetch(const struct device *dev)
{
	const struct seesaw_config *config = dev->config;
	struct seesaw_data *data = dev->data;
	uint8_t gpio[4], reg;

	reg = SEESAW_GPIO_FUNCTION_GPIO;

	if (i2c_write_read_dt(&config->i2c, &reg, 1, gpio, 4) < 0) {
		return -EIO;
	}

	data->sample_gpio = sys_get_le32(gpio);

	return data->sample_gpio;
}

bool seesaw_gpio_pin_get(const struct device *dev, uint8_t pin)
{
	const struct seesaw_data *data = dev->data;

	return data->sample_gpio & BIT(pin);
}

void seesaw_gpio_pin_pullup_enable(const struct device *dev, uint8_t pin)
{
	const struct seesaw_config *config = dev->config;
	uint32_t data = 1 << pin;

	i2c_write_32bit_register(dev, SEESAW_GPIO_FUNCTION_PULLENSET, data);
}

void seesaw_gpio_pin_pulldown_enable(const struct device *dev, uint8_t pin)
{
	const struct seesaw_config *config = dev->config;
	uint32_t data = 1 << pin;

	i2c_write_32bit_register(dev, SEESAW_GPIO_FUNCTION_PULLENCLR, data);
}

void seesaw_gpio_pin_pull_disable(const struct device *dev, uint8_t pin)
{
	const struct seesaw_config *config = dev->config;
	uint32_t data = 1 << pin;

	i2c_write_32bit_register(dev, SEESAW_GPIO_FUNCTION_PULLENCLR, data);
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
			      CONFIG_SENSOR_INIT_PRIORITY, NULL);		\

 DT_INST_FOREACH_STATUS_OKAY(SEESAW_DEFINE)
