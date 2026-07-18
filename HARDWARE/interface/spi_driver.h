/**
 * @file spi_driver.h
 * @brief SPI communication driver for ICM-42688
 * @author Yusuf Karaböcek
 * @date July 2025
 */

#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include <stdint.h>
#include "main.h"
/**
 * @brief SPI read wrapper function
 * @param reg Register address
 * @param data Data buffer
 * @param len Data length
 * @return 0 on success, negative value on error
 */
int spi_read_wrapper(uint8_t reg, uint8_t *data, uint16_t len);

/**
 * @brief SPI write wrapper function
 * @param reg Register address
 * @param data Data buffer
 * @param len Data length
 * @return 0 on success, negative value on error
 */
int spi_write_wrapper(uint8_t reg, uint8_t *data, uint16_t len);

/**
 * @brief Enable chip select (CS)
 */
void spi_cs_enable(void);

/**
 * @brief Disable chip select (CS)
 */
void spi_cs_disable(void);

#endif // SPI_DRIVER_H