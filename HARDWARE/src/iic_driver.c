#include "iic_driver.h"
#include "stm32f4xx_hal.h"
#include <stddef.h>

extern I2C_HandleTypeDef hi2c1;

#define I2C_DRIVER_TIMEOUT_MS    20U

static int hal_status_to_error(HAL_StatusTypeDef status)
{
    switch (status)
    {
        case HAL_OK:
            return 0;

        case HAL_BUSY:
            return -2;

        case HAL_TIMEOUT:
            return -3;

        case HAL_ERROR:
        default:
            return -4;
    }
}

int i2c_write(uint8_t dev_addr, const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    if ((data == NULL) || (len == 0U))
    {
        return -1;
    }

    status = HAL_I2C_Master_Transmit(
        &hi2c1,
        (uint16_t)(dev_addr << 1),
        (uint8_t *)data,
        len,
        I2C_DRIVER_TIMEOUT_MS);

    return hal_status_to_error(status);
}

int i2c_read(uint8_t dev_addr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    if ((data == NULL) || (len == 0U))
    {
        return -1;
    }

    status = HAL_I2C_Master_Receive(
        &hi2c1,
        (uint16_t)(dev_addr << 1),
        data,
        len,
        I2C_DRIVER_TIMEOUT_MS);

    return hal_status_to_error(status);
}

int i2c_is_ready(uint8_t dev_addr, uint32_t trials, uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_IsDeviceReady(
        &hi2c1,
        (uint16_t)(dev_addr << 1),
        trials,
        timeout_ms);
    return hal_status_to_error(status);
}