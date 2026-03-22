#include "pca9685.h"
#include <math.h>

#define PCA9685_I2C_TIMEOUT_MS 100

HAL_StatusTypeDef PCA9685_IsReady(I2C_HandleTypeDef *hi2c)
{
    return HAL_I2C_IsDeviceReady(hi2c, PCA9685_ADDR, 3, PCA9685_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef PCA9685_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(
        hi2c,
        PCA9685_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        PCA9685_I2C_TIMEOUT_MS
    );
}

HAL_StatusTypeDef PCA9685_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Read(
        hi2c,
        PCA9685_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        1,
        PCA9685_I2C_TIMEOUT_MS
    );
}

HAL_StatusTypeDef PCA9685_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;

    // MODE1:
    // AI = 1  -> auto-increment enabled
    // ALLCALL = 0 -> disable all-call to keep things simple
    // SLEEP = 0 -> normal mode
    uint8_t mode1 = PCA9685_MODE1_AI;

    // MODE2:
    // OUTDRV = 1 -> totem pole
    // OCH = 0 -> outputs change on STOP
    // INVRT = 0 -> not inverted
    uint8_t mode2 = PCA9685_MODE2_OUTDRV;

    status = PCA9685_WriteReg(hi2c, PCA9685_MODE1, mode1);
    if (status != HAL_OK) return status;

    status = PCA9685_WriteReg(hi2c, PCA9685_MODE2, mode2);
    if (status != HAL_OK) return status;

    HAL_Delay(1);

    // Turn all channels fully OFF at startup
    status = PCA9685_WriteReg(hi2c, PCA9685_ALL_LED_ON_L, 0x00);
    if (status != HAL_OK) return status;
    status = PCA9685_WriteReg(hi2c, PCA9685_ALL_LED_ON_H, 0x00);
    if (status != HAL_OK) return status;
    status = PCA9685_WriteReg(hi2c, PCA9685_ALL_LED_OFF_L, 0x00);
    if (status != HAL_OK) return status;
    status = PCA9685_WriteReg(hi2c, PCA9685_ALL_LED_OFF_H, 0x10); // full OFF bit
    if (status != HAL_OK) return status;

    return HAL_OK;
}

HAL_StatusTypeDef PCA9685_SetPWMFreq(I2C_HandleTypeDef *hi2c, float freq_hz)
{
    HAL_StatusTypeDef status;
    uint8_t old_mode;
    uint8_t sleep_mode;
    uint8_t wake_mode;

    // prescale = round(25MHz / (4096 * freq)) - 1
    float prescale_f = ((float)PCA9685_OSC_CLOCK / (4096.0f * freq_hz)) - 1.0f;
    uint8_t prescale = (uint8_t)(prescale_f + 0.5f);

    status = PCA9685_ReadReg(hi2c, PCA9685_MODE1, &old_mode);
    if (status != HAL_OK) return status;

    // Must set SLEEP=1 before writing PRE_SCALE
    sleep_mode = (old_mode & (uint8_t)~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
    status = PCA9685_WriteReg(hi2c, PCA9685_MODE1, sleep_mode);
    if (status != HAL_OK) return status;

    status = PCA9685_WriteReg(hi2c, PCA9685_PRE_SCALE, prescale);
    if (status != HAL_OK) return status;

    // Wake up, keep AI and other bits
    wake_mode = old_mode & (uint8_t)~PCA9685_MODE1_SLEEP;
    status = PCA9685_WriteReg(hi2c, PCA9685_MODE1, wake_mode);
    if (status != HAL_OK) return status;

    HAL_Delay(1); // >500 us oscillator startup time

    // Optional restart
    status = PCA9685_WriteReg(hi2c, PCA9685_MODE1, wake_mode | PCA9685_MODE1_RESTART);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

HAL_StatusTypeDef PCA9685_SetPWM(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t on_count, uint16_t off_count)
{
    if (channel > 15 || on_count > 4095 || off_count > 4095) {
        return HAL_ERROR;
    }

    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    uint8_t data[4];

    data[0] = (uint8_t)(on_count & 0xFF);         // ON_L
    data[1] = (uint8_t)((on_count >> 8) & 0x0F);  // ON_H
    data[2] = (uint8_t)(off_count & 0xFF);        // OFF_L
    data[3] = (uint8_t)((off_count >> 8) & 0x0F); // OFF_H

    return HAL_I2C_Mem_Write(
        hi2c,
        PCA9685_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        4,
        PCA9685_I2C_TIMEOUT_MS
    );
}

HAL_StatusTypeDef PCA9685_SetServoPulseCounts(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t off_count)
{
    if (off_count > 4095) return HAL_ERROR;

    // For servo control we usually start pulse at count 0 and end at off_count
    return PCA9685_SetPWM(hi2c, channel, 0, off_count);
}
