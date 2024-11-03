/*
 * LPS25HB.c
 *
 *  Created on: Nov 3, 2024
 *      Author: andre
 */

#include "LPS25HB.h"

uint8_t LPS25HB_address = LPS25HB_DEVICE_ADDRESS_0;
uint16_t LPS25HB_PressureOffset;

void LPS25HB_read_bytes(uint8_t reg_addr, uint8_t* values, size_t length) {
    i2c_master_read(values, length, reg_addr, LPS25HB_address, 1);
    // Added a delay or a check for 'end_of_read_flag' to ensure data has been read before proceeding

}

void LPS25HB_write_bytes(uint8_t reg_addr, uint8_t* values, size_t length) {
    i2c_master_write_multi(values, length, reg_addr, LPS25HB_address, 0);
}


void LPS25HB_init() {
    uint8_t whoAmI = 0;
    LPS25HB_read_bytes(LPS25HB_WHO_AM_I_ADDRESS, &whoAmI, 1);

    if (!(whoAmI == LPS25HB_WHO_AM_I_VALUE)) {
        LPS25HB_address = LPS25HB_DEVICE_ADDRESS_1;

        LPS25HB_read_bytes(LPS25HB_WHO_AM_I_ADDRESS, &whoAmI, 1);
        if (whoAmI == LPS25HB_WHO_AM_I_VALUE) {
            uint8_t ctrl1 = 148;
            LPS25HB_write_bytes(LPS25HB_ADDRESS_CTRL1, &ctrl1, 1);
            LPS25HB_get_pressure_calibration();
        }
    }
}


void LPS25HB_get_pressure_calibration(void){
    uint8_t buffer[2];
    // Read two bytes from RPDS_L register with auto-increment to RPDS_H
    LPS25HB_read_bytes(LPS25HB_ADDRESS_RPDS_L | 0x80, buffer, 2);

    // Combine the two bytes into a single 16-bit value
    LPS25HB_PressureOffset = (int16_t)(buffer[0] | (buffer[1] << 8));
}

void LPS25HB_get_pressure(float* pressure) {
    uint8_t buffer[3];
    // Read three bytes from PRESS_OUT_XL register with auto-increment to PRESS_OUT_H
    LPS25HB_read_bytes(LPS25HB_ADDRESS_PRESS_OUT_XL | 0x80, buffer, 3);

    // Combine the three bytes into a single 24-bit value
    uint32_t p_out = (uint32_t)(buffer[2] << 16) | (buffer[1] << 8) | buffer[0];

    // Calculate the actual pressure value
    *pressure = p_out / 4096.0f;
}



