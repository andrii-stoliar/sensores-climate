/*
 * LPS25HB.h
 *
 *  Created on: Nov 3, 2024
 *      Author: andre
 */

#include "main.h"

#ifndef LPS25HB_H_
#define LPS25HB_H_

#define 	LPS25HB_DEVICE_ADDRESS_0		0xB8
#define 	LPS25HB_DEVICE_ADDRESS_1		0xBA

#define 	LPS25HB_WHO_AM_I_VALUE			0xBD
#define 	LPS25HB_WHO_AM_I_ADDRESS		0x0F

#define 	LPS25HB_ADDRESS_CTRL1			0x20

#define 	LPS25HB_ADDRESS_PRESS_OUT_XL	0x28
#define 	LPS25HB_ADDRESS_PRESS_OUT_L		0x29
#define 	LPS25HB_ADDRESS_PRESS_OUT_H		0x2A

#define		LPS25HB_ADDRESS_RPDS_L			0x39
#define		LPS25HB_ADDRESS_RPDS_H			0x3A

void LPS25HB_read_byte(uint8_t reg_addr);
void LPS25HB_write_byte(uint8_t reg_addr, uint8_t value);
void LPS25HB_init(void);

void LPS25HB_get_pressure_calibration(void);
#endif /* LPS25HB_H_ */

