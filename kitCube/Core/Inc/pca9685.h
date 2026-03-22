#ifndef PCA9685_H
#define PCA9685_H

#include "main.h"
#include <stdint.h>

#define PCA9685_OSC_CLOCK      25000000UL

// 7-bit base address is 0x40 when A0..A5 are all LOW.
// STM32 HAL uses the 8-bit address form (7-bit << 1).
#define PCA9685_ADDR           (0x40 << 1)

// Registers
#define PCA9685_MODE1          0x00
#define PCA9685_MODE2          0x01
#define PCA9685_SUBADR1        0x02
#define PCA9685_SUBADR2        0x03
#define PCA9685_SUBADR3        0x04
#define PCA9685_ALLCALLADR     0x05

#define PCA9685_LED0_ON_L      0x06
#define PCA9685_LED0_ON_H      0x07
#define PCA9685_LED0_OFF_L     0x08
#define PCA9685_LED0_OFF_H     0x09

#define PCA9685_ALL_LED_ON_L   0xFA
#define PCA9685_ALL_LED_ON_H   0xFB
#define PCA9685_ALL_LED_OFF_L  0xFC
#define PCA9685_ALL_LED_OFF_H  0xFD
#define PCA9685_PRE_SCALE      0xFE

// MODE1 bits
#define PCA9685_MODE1_ALLCALL  0x01
#define PCA9685_MODE1_SUB3     0x02
#define PCA9685_MODE1_SUB2     0x04
#define PCA9685_MODE1_SUB1     0x08
#define PCA9685_MODE1_SLEEP    0x10
#define PCA9685_MODE1_AI       0x20
#define PCA9685_MODE1_EXTCLK   0x40
#define PCA9685_MODE1_RESTART  0x80

// MODE2 bits
#define PCA9685_MODE2_OUTNE_0  0x01
#define PCA9685_MODE2_OUTNE_1  0x02
#define PCA9685_MODE2_OUTDRV   0x04
#define PCA9685_MODE2_OCH      0x08
#define PCA9685_MODE2_INVRT    0x10

HAL_StatusTypeDef PCA9685_IsReady(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef PCA9685_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data);
HAL_StatusTypeDef PCA9685_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data);

HAL_StatusTypeDef PCA9685_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef PCA9685_SetPWMFreq(I2C_HandleTypeDef *hi2c, float freq_hz);
HAL_StatusTypeDef PCA9685_SetPWM(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t on_count, uint16_t off_count);

// helpful for servos
HAL_StatusTypeDef PCA9685_SetServoPulseCounts(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t off_count);

#endif
