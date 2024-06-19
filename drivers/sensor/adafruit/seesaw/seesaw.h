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


/* 
Base Register Address	Module
0x00		Status
0x01		GPIO
0x02 - 0x07	SERCOM
0x08		PWM
0x09		ADC
0x0A		DAC
0x0B		Interrupt
0x0C		DAP
0x0D		EEPROM
0x0E		NeoPixel
0x0F		Touch
0x10		Keypad
0x11		Encoder
*/


#define JOYWING_REG_X_AXIS 0x03	/* Horizontal */
#define JOYWING_REG_Y_AXIS 0x02 /* Vertical */

#define JOYWING_REG_BUTTONS 0x04

#define JOYWING_BUTTON_RIGHT	6
#define JOYWING_BUTTON_DOWN		7
#define JOYWING_BUTTON_LEFT		9
#define JOYWING_BUTTON_UP		10
#define JOYWING_BUTTON_SEL		14


struct seesaw_config {
	struct i2c_dt_spec i2c;
};

struct seesaw_data {
	int sample_x_axis;
	int sample_y_axis;

	int buttons;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_JOYWING_JOYWING_H_ */
