/**
 * @file spi_driver.c
 * @brief SPI communication driver implementation for ICM-42688
 * @author Yusuf Karaböcek
 * @date July 2025
 */

#include "spi_driver.h"
#include "stm32f4xx_hal.h"
extern void MX_SPI1_Init(void);
extern SPI_HandleTypeDef hspi1;

#define READ_FLAG 0x80 /**< SPI read flag bit */

void spi_cs_enable(void) {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET); /* Pin selection can be done in main.c */
}

void spi_cs_disable(void) {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}
/*
int spi_read_wrapper(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx[32] = {0};
    uint8_t rx[32] = {0};
    tx[0] = reg | READ_FLAG;

    spi_cs_enable();
    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(&hspi1, tx, rx, len + 1, 10);
    spi_cs_disable();

    if (ret != HAL_OK) return -1;

    for (int i = 0; i < len; i++) data[i] = rx[i + 1];
    return 0;
}
*/
int spi_read_wrapper(uint8_t reg, uint8_t *data, uint16_t len) {
    uint8_t tx_data = reg | READ_FLAG;
    spi_cs_enable();
    if(HAL_SPI_Transmit(&hspi1, &tx_data, 1, 100) != HAL_OK) { spi_cs_disable(); return -1; }
    if(HAL_SPI_Receive(&hspi1, data, len, 100) != HAL_OK) { spi_cs_disable(); return -2; }
    spi_cs_disable();
    return 0;
}
int spi_write_wrapper(uint8_t reg, uint8_t *data, uint16_t len) {
    uint8_t tx_data = reg & 0x7F;
    spi_cs_enable();

    if(HAL_SPI_Transmit(&hspi1, &tx_data, 1, 1000) != HAL_OK) {
        spi_cs_disable();
        return -1;
    }

    if(HAL_SPI_Transmit(&hspi1, data, len, 1000) != HAL_OK) {
        spi_cs_disable();
        return -2;
    }

    spi_cs_disable();
    return 0;
}
