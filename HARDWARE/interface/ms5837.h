#ifndef MS5837_H
#define MS5837_H

#include <stdint.h>
#include <stdbool.h>


#define MS5837_I2C_ADDRESS               0x76U

#define MS5837_CMD_RESET                 0x1EU
#define MS5837_CMD_ADC_READ              0x00U
#define MS5837_CMD_PROM_READ_BASE        0xA0U

#define MS5837_CMD_CONVERT_D1_OSR4096    0x48U
#define MS5837_CMD_CONVERT_D2_OSR4096    0x58U

#define MS5837_CMD_CONVERT_D1_OSR8192    0x4AU
#define MS5837_CMD_CONVERT_D2_OSR8192    0x5AU

/*
 * MS5837-02BA在OSR4096下的最大转换时间是8.61ms。
 * 取9ms保证转换完成。
 */
#define MS5837_CONVERSION_TIME_MS        20U

#define MS5837_PROM_WORD_COUNT           8U

typedef int (*ms5837_write_fn)(
    uint8_t dev_addr,
    const uint8_t *data,
    uint16_t len);

typedef int (*ms5837_read_fn)(
    uint8_t dev_addr,
    uint8_t *data,
    uint16_t len);

typedef void (*ms5837_delay_fn)(uint32_t delay_ms);

typedef struct
{
    ms5837_write_fn write;
    ms5837_read_fn read;
    ms5837_delay_fn delay_ms;
} ms5837_bus_t;

typedef struct
{
    ms5837_bus_t bus;

    uint8_t dev_addr;
    uint16_t prom[MS5837_PROM_WORD_COUNT];

    uint32_t raw_pressure;
    uint32_t raw_temperature;

    float pressure_mbar;
    float temperature_c;

    float base_pressure_mbar;
    float fluid_density_kg_m3;

    bool initialized;
    bool data_ready;

    int last_error;
} ms5837_t;

int ms5837_init(
    ms5837_t *dev,
    const ms5837_bus_t *bus,
    uint8_t dev_addr);

int ms5837_read(ms5837_t *dev);

int ms5837_calculate_depth(
    const ms5837_t *dev,
    float *depth_m);

void ms5837_set_base_pressure(
    ms5837_t *dev,
    float pressure_mbar);

int ms5837_set_fluid_density(
    ms5837_t *dev,
    float density_kg_m3);

float ms5837_get_pressure(
    const ms5837_t *dev);

float ms5837_get_temperature(
    const ms5837_t *dev);

bool ms5837_is_ready(
    const ms5837_t *dev);

int ms5837_get_last_error(
    const ms5837_t *dev);


#endif