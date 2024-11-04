/*
 * HTS221.c
 *
 *  Created on: Nov 3, 2024
 *      Author: andre
 */

#include "HTS221.h"

uint8_t HTS221_address = HTS221_DEVICE_ADDRESS_0;
float HTS221_HumiditySlope;
float HTS221_HumidityZero;
float HTS221_TemperatureSlope;
float HTS221_TemperatureZero;
//uint8_t HTS221_address = YOUR_DEFAULT_ADDRESS; // Replace with the actual default address

// Read multiple bytes
void HTS221_read_bytes(uint8_t reg_addr, uint8_t* values, size_t length) {
    i2c_master_read(values, length, reg_addr, HTS221_address, 1);
    // Added a delay or a check for 'end_of_read_flag' to ensure data has been read before proceeding

}

// Write multiple bytes
void HTS221_write_bytes(uint8_t reg_addr, uint8_t* values, size_t length) {
    i2c_master_write_multi(values, length, reg_addr, HTS221_address, 0);
}

void HTS221_init(void) {
    uint8_t whoAmI;

    HTS221_read_bytes(HTS221_WHO_AM_I_ADDRESS, &whoAmI, 1);

    if (!(whoAmI == HTS221_WHO_AM_I_VALUE)) {
    	HTS221_address = HTS221_DEVICE_ADDRESS_1;
    	HTS221_read_bytes(HTS221_WHO_AM_I_ADDRESS, &whoAmI, 1);
    }

    uint8_t ctrl_setting = 134;
    HTS221_write_bytes(HTS221_CTRL1, &ctrl_setting, 1);

    HTS221_get_humidity_calibration();
    HTS221_get_temperature_calibration();


}

void HTS221_get_humidity_calibration(void) {
    uint8_t calibration_data[2]; // Buffer to hold H0_rH and H1_rH
    uint8_t h0T0Out_data[2], h1T0Out_data[2]; // Buffers to hold the H0_T0_OUT and H1_T0_OUT
    int16_t h0_t0_out, h1_t0_out;

    // Read H0_rH and H1_rH together with auto-increment
    HTS221_read_bytes(HTS221_H0_rH_x2 | 0x80, calibration_data, 2);

    // Extract the H0_rH and H1_rH values
    uint8_t h0_rH = calibration_data[0] >> 1; // H0_rH is stored as (value / 2)
    uint8_t h1_rH = calibration_data[1] >> 1; // H1_rH is stored as (value / 2)

    // Read H0_T0_OUT and H1_T0_OUT together with auto-increment
    HTS221_read_bytes(HTS221_H0_T0_OUT_H | 0x80, h0T0Out_data, 2);
    HTS221_read_bytes(HTS221_H0_T0_OUT_L | 0x80, h1T0Out_data, 2);

    // Combine the bytes into the H0_T0_OUT and H1_T0_OUT values
    h0_t0_out = (int16_t)((uint16_t)h0T0Out_data[1] << 8 | (uint16_t)h0T0Out_data[0]);
    h1_t0_out = (int16_t)((uint16_t)h1T0Out_data[1] << 8 | (uint16_t)h1T0Out_data[0]);

    // Calculate the humidity calibration slope and zero intercept
    HTS221_HumiditySlope = (float)(h1_rH - h0_rH) / (h1_t0_out - h0_t0_out);
    HTS221_HumidityZero = h0_rH - HTS221_HumiditySlope * h0_t0_out;
}

void HTS221_get_humidity(float* humidity_out) {
    uint8_t h_out_data[2];
    int16_t h_out;

    HTS221_read_bytes(HTS221_HUMIDITY_OUT_L, h_out_data, 2);
    h_out = h_out_data[0] | (h_out_data[1] << 8);

    *humidity_out = (h_out * HTS221_HumiditySlope + HTS221_HumidityZero);
}




void HTS221_get_temperature_calibration(void) {
    uint8_t calibration_data[4]; // Buffer to hold T0/T1 calibration values
    uint8_t t_out_data[4]; // Buffer to hold T0_OUT/T1_OUT calibration values
    int16_t t0_out, t1_out;
    uint16_t t0_degC, t1_degC;

    // Read T0_degC, T1_degC and MSB bits together with auto-increment
    HTS221_read_bytes(HTS221_T0_degC_x8 | 0x80, calibration_data, 4);

    // Extract the calibration values
    t0_degC = ((uint16_t)(calibration_data[0]) | ((uint16_t)(calibration_data[2] & 0x03) << 8));
        t1_degC = ((uint16_t)(calibration_data[1]) | ((uint16_t)(calibration_data[2] & 0x0C) << 8));

    // Read T0_OUT and T1_OUT together with auto-increment
    HTS221_read_bytes(HTS221_T0_OUT_L | 0x80, t_out_data, 4);

    // Combine the bytes into the T0_OUT and T1_OUT values
    t0_out = (int16_t)(t_out_data[1] << 8 | t_out_data[0]);
    t1_out = (int16_t)(t_out_data[3] << 8 | t_out_data[2]);

    // Calculate the temperature calibration slope and intercept using the calibration values
    HTS221_TemperatureSlope = (t1_degC - t0_degC) / (8.0 * t1_out - t0_out);
    HTS221_TemperatureZero = (t0_degC / 8.0) - HTS221_TemperatureSlope * t0_out;
LL_mDelay(100);
}


void HTS221_get_temperature(float* temperature_out) {
    uint8_t t_out_buffer[2];
    int16_t t_out;

    // Read both temperature registers at once
    HTS221_read_bytes(HTS221_TEMP_OUT_L | 0x80, t_out_buffer, 2);

    // Combine the two bytes into a single 16-bit value
    t_out = (int16_t)((uint16_t)t_out_buffer[1] << 8 | t_out_buffer[0]);

    // Calculate the temperature in Celsius
    *temperature_out = (t_out * -HTS221_TemperatureSlope) + HTS221_TemperatureZero;
}
