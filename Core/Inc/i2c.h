/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
/* Additional includes can be added here */
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
/* Private defines can be added here */
/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */
void i2c_master_write_multiple(uint8_t *data, size_t length, uint8_t regAddr, uint8_t deviceAddr, uint8_t readFlag);
uint8_t *i2c_master_read_data(uint8_t *buffer, uint8_t length, uint8_t regAddr, uint8_t deviceAddr, uint8_t readFlag);
void I2C1_Reception_Callback(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

