/* sensor_lsm6ds0.h - header file for LSM6DS0 accelerometer, gyroscope and
 * temperature sensor driver
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_SEESAW_SEESAW_H_
#define ZEPHYR_DRIVERS_SENSOR_SEESAW_SEESAW_H_

#include <zephyr/types.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>

enum seesaw_channel {
	SEESAW_CHAN_STATUS = 0x00,
	SEESAW_CHAN_GPIO = 0x01,
	SEESAW_CHAN_SERCOM1 = 0x02,
	SEESAW_CHAN_SERCOM2 = 0x03,
	SEESAW_CHAN_SERCOM3 = 0x04,
	SEESAW_CHAN_SERCOM4 = 0x05,
	SEESAW_CHAN_SERCOM5 = 0x06,
	SEESAW_CHAN_SERCOM6 = 0x07,
	SEESAW_CHAN_PWM = 0x08,
	SEESAW_CHAN_ADC = 0x09,
	SEESAW_CHAN_DAC = 0x0A,
	SEESAW_CHAN_INTERRUPT = 0x0B,
	SEESAW_CHAN_DAP = 0x0C,
	SEESAW_CHAN_EEPROM = 0x0D,
	SEESAW_CHAN_NEOPIXEL = 0x0E,
	SEESAW_CHAN_TOUCH = 0x0F,
	SEESAW_CHAN_KEYPAD = 0x10,
	SEESAW_CHAN_ENCODER = 0x11
};

enum seesaw_status_function {
	SEESAW_STATUS_HW_ID = 0x01,
	SEESAW_STATUS_VERSION = 0x02,
	SEESAW_STATUS_OPTIONS = 0x03,
	SEESAW_STATUS_TEMP = 0x04,
	SEESAW_STATUS_SWRST = 0x7F
};

enum seesaw_gpio_function {
	SEESAW_GPIO_FUNCTION_DIRSET = 0x02,
	SEESAW_GPIO_FUNCTION_DIRCLR = 0x03,
	SEESAW_GPIO_FUNCTION_GPIO = 0x04,
	SEESAW_GPIO_FUNCTION_SET = 0x05,
	SEESAW_GPIO_FUNCTION_CLR = 0x06,
	SEESAW_GPIO_FUNCTION_TOGGLE = 0x07,
	SEESAW_GPIO_FUNCTION_INTENSET = 0x08,
	SEESAW_GPIO_FUNCTION_INTENCLR = 0x09,
	SEESAW_GPIO_FUNCTION_INTFLAG = 0x0A,
	SEESAW_GPIO_FUNCTION_PULLENSET = 0x0B,
	SEESAW_GPIO_FUNCTION_PULLENCLR = 0x0C
};

enum seesaw_adc_function {
	SEESAW_ADC_FUNCTION_STATUS = 0x00,
	SEESAW_ADC_FUNCTION_INTENSET = 0x02,
	SEESAW_ADC_FUNCTION_INTENCLR = 0x03,
	SEESAW_ADC_FUNCTION_WINMODE = 0x04,
	SEESAW_ADC_FUNCTION_WINTHRESH = 0x05,
	SEESAW_ADC_FUNCTION_CHANNEL_0 = 0x07,
	SEESAW_ADC_FUNCTION_CHANNEL_1 = 0x08,
	SEESAW_ADC_FUNCTION_CHANNEL_2 = 0x09,
	SEESAW_ADC_FUNCTION_CHANNEL_3 = 0x0A
};

#define SEESAW_ADC_NUMBER_OF_CHANNELS 4
struct seesaw_config {
	struct i2c_dt_spec i2c;
};

struct seesaw_data {
	uint16_t sample_adc[SEESAW_ADC_NUMBER_OF_CHANNELS];
	uint32_t sample_gpio;
	uint8_t hw_id;
	uint32_t version;
	uint32_t options;
};

static int seesaw_init(const struct device *dev);

uint32_t seesaw_temperature_get(const struct device *dev);

uint16_t seesaw_adc_channel_read(const struct device *dev, uint8_t channel);

void seesaw_gpio_pin_set_input(const struct device *dev, uint8_t pin);
void seesaw_gpio_pin_set_output(const struct device *dev, uint8_t pin);

void seesaw_gpio_pin_set(const struct device *dev, uint8_t pin);
void seesaw_gpio_pin_clear(const struct device *dev, uint8_t pin);
void seesaw_gpio_pin_toggle(const struct device *dev, uint8_t pin);

uint32_t seesaw_gpio_sample_get(const struct device *dev);
bool seesaw_gpio_pin_get(const struct device *dev, uint8_t pin);

void seesaw_gpio_pin_pullup_enable(const struct device *dev, uint8_t pin);
void seesaw_gpio_pin_pulldown_enable(const struct device *dev, uint8_t pin);

void seesaw_gpio_pin_pull_disable(const struct device *dev, uint8_t pin);



#endif /* ZEPHYR_DRIVERS_SENSOR_JOYWING_JOYWING_H_ */
