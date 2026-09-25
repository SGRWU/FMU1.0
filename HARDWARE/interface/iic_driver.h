#ifndef IIC_DRIVER_H
#define IIC_DRIVER_H

#include <stdint.h>



/*
 * dev_addr使用7位I2C地址。
 * 例如MS5837传入0x76，底层调用HAL时再左移一位。
 */
int i2c_write(uint8_t dev_addr, const uint8_t *data, uint16_t len);
int i2c_read(uint8_t dev_addr, uint8_t *data, uint16_t len);

/*
 * 用于初始化前检查设备是否应答。
 * trials为尝试次数，timeout_ms为单次超时时间。
 */
int i2c_is_ready(uint8_t dev_addr, uint32_t trials, uint32_t timeout_ms);


#endif