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

uint32_t seesaw_temperature_get(const struct device *dev);

uint16_t seesaw_adc_channel_read(const struct device *dev, uint8_t channel, uint16_t *val);

void seesaw_gpio_pin_set_input(const struct device *dev, uint8_t pin);
void seesaw_gpio_pin_set_output(const struct device *dev, uint8_t pin);

void seesaw_gpio_pin_set(const struct device *dev, uint8_t pin);
void seesaw_gpio_pin_clear(const struct device *dev, uint8_t pin);
void seesaw_gpio_pin_toggle(const struct device *dev, uint8_t pin);

uint32_t seesaw_gpio_sample_fetch(const struct device *dev);
uint32_t seesaw_gpio_sample_get(const struct device *dev);
bool seesaw_gpio_pin_get(const struct device *dev, uint8_t pin);

void seesaw_gpio_pin_pullup_enable(const struct device *dev, uint8_t pin);
void seesaw_gpio_pin_pulldown_enable(const struct device *dev, uint8_t pin);

void seesaw_gpio_pin_pull_disable(const struct device *dev, uint8_t pin);

#endif /* ZEPHYR_DRIVERS_SENSOR_JOYWING_JOYWING_H_ */
