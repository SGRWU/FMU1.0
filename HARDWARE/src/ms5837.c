#include "ms5837.h"
#include <stddef.h>
#include <string.h>

#define MS5837_GRAVITY_M_S2          9.80665f
#define MS5837_DEFAULT_DENSITY       997.0f
#define MS5837_DEFAULT_PRESSURE      1013.25f

enum
{
    MS5837_OK = 0,
    MS5837_ERROR_ARGUMENT = -1,
    MS5837_ERROR_BUS = -2,
    MS5837_ERROR_PROM = -3,
    MS5837_ERROR_CRC = -4,
    MS5837_ERROR_ADC = -5,
    MS5837_ERROR_NOT_READY = -6,
    MS5837_ERROR_RANGE = -7
};

static uint8_t ms5837_crc4(const uint16_t prom[MS5837_PROM_WORD_COUNT])
{
    uint16_t prom_copy[MS5837_PROM_WORD_COUNT];
    uint16_t remainder = 0U;
    uint8_t bit;
    uint8_t cnt;

    memcpy(prom_copy, prom, sizeof(prom_copy));

    /*
     * PROM word 0的高4位存放CRC。
     * 计算前清除CRC字段，word 7按手册置零。
     */
    prom_copy[0] &= 0x0FFFU;
    prom_copy[7] = 0U;

    for (cnt = 0U; cnt < 16U; cnt++)
    {
        if ((cnt & 1U) != 0U)
        {
            remainder ^= (uint16_t)(
                prom_copy[cnt >> 1U] & 0x00FFU);
        }
        else
        {
            remainder ^= (uint16_t)(
                prom_copy[cnt >> 1U] >> 8U);
        }

        for (bit = 8U; bit > 0U; bit--)
        {
            if ((remainder & 0x8000U) != 0U)
            {
                remainder =
                    (uint16_t)((remainder << 1U) ^ 0x3000U);
            }
            else
            {
                remainder =
                    (uint16_t)(remainder << 1U);
            }
        }
    }

    return (uint8_t)((remainder >> 12U) & 0x0FU);
}

static bool ms5837_prom_is_reasonable(
    const uint16_t prom[MS5837_PROM_WORD_COUNT])
{
    uint8_t i;
    bool all_zero = true;
    bool all_ones = true;

    for (i = 0U; i < MS5837_PROM_WORD_COUNT; i++)
    {
        if (prom[i] != 0U)
        {
            all_zero = false;
        }

        if (prom[i] != 0xFFFFU)
        {
            all_ones = false;
        }
    }

    if (all_zero || all_ones)
    {
        return false;
    }

    /*
     * C1～C6不能为0或0xFFFF。
     */
    for (i = 1U; i <= 6U; i++)
    {
        if ((prom[i] == 0U) || (prom[i] == 0xFFFFU))
        {
            return false;
        }
    }

    return true;
}

static int ms5837_send_command(
    ms5837_t *dev,
    uint8_t command)
{
    if ((dev == NULL) || (dev->bus.write == NULL))
    {
        return MS5837_ERROR_ARGUMENT;
    }

    if (dev->bus.write(
            dev->dev_addr,
            &command,
            1U) != 0)
    {
        return MS5837_ERROR_BUS;
    }

    return MS5837_OK;
}

static int ms5837_read_prom(ms5837_t *dev)
{
    uint8_t i;
    uint8_t buffer[2];
    uint8_t command;
    uint8_t stored_crc;
    uint8_t calculated_crc;

    for (i = 0U; i < 7; i++)
    {
        command = (uint8_t)(
            MS5837_CMD_PROM_READ_BASE + (i << 1U));

        if (ms5837_send_command(dev, command) != MS5837_OK)
        {
            return MS5837_ERROR_BUS;
        }

        if (dev->bus.read(
                dev->dev_addr,
                buffer,
                2U) != 0)
        {
            return MS5837_ERROR_BUS;
        }

        dev->prom[i] =
            ((uint16_t)buffer[0] << 8U) |
            (uint16_t)buffer[1];
    }
	
	dev->prom[7] = 0;
	
    if (!ms5837_prom_is_reasonable(dev->prom))
    {
        return MS5837_ERROR_PROM;
    }

    stored_crc =
        (uint8_t)((dev->prom[0] >> 12U) & 0x0FU);

    calculated_crc = ms5837_crc4(dev->prom);

    if (stored_crc != calculated_crc)
    {
        return MS5837_ERROR_CRC;
    }

    return MS5837_OK;
}

static int ms5837_read_adc(
    ms5837_t *dev,
    uint32_t *adc_value)
{
    uint8_t buffer[3];

    if ((dev == NULL) || (adc_value == NULL))
    {
        return MS5837_ERROR_ARGUMENT;
    }

    if (ms5837_send_command(
            dev,
            MS5837_CMD_ADC_READ) != MS5837_OK)
    {
        return MS5837_ERROR_BUS;
    }

    if (dev->bus.read(
            dev->dev_addr,
            buffer,
            3U) != 0)
    {
        return MS5837_ERROR_BUS;
    }

    *adc_value =
        ((uint32_t)buffer[0] << 16U) |
        ((uint32_t)buffer[1] << 8U) |
        (uint32_t)buffer[2];

    if ((*adc_value == 0U) ||
        (*adc_value == 0x00FFFFFFUL))
    {
        return MS5837_ERROR_ADC;
    }

    return MS5837_OK;
}

static int ms5837_convert_and_read(
    ms5837_t *dev,
    uint8_t conversion_command,
    uint32_t *adc_value)
{
    int result;

    result = ms5837_send_command(
        dev,
        conversion_command);

    if (result != MS5837_OK)
    {
        return result;
    }

    dev->bus.delay_ms(MS5837_CONVERSION_TIME_MS);

    return ms5837_read_adc(dev, adc_value);
}

static void ms5837_compensate_02ba(
    ms5837_t *dev,
    uint32_t d1,
    uint32_t d2)
{
    int32_t dT;
    int32_t temperature;
    int64_t offset;
    int64_t sensitivity;

    int64_t temperature_second_order = 0;
    int64_t offset_second_order = 0;
    int64_t sensitivity_second_order = 0;

    int64_t temperature_difference;
    int64_t pressure;

    /*
     * 一阶温度补偿：
     * dT = D2 - C5 * 2^8
     * TEMP = 2000 + dT * C6 / 2^23
     */
    dT =
        (int32_t)d2 -
        (int32_t)((uint32_t)dev->prom[5] << 8U);

    temperature =
        2000 +
        (int32_t)(
            ((int64_t)dT * dev->prom[6]) /
            8388608LL);

    /*
     * MS5837-02BA一阶压力补偿：
     *
     * OFF  = C2 * 2^17 + C4 * dT / 2^6
     * SENS = C1 * 2^16 + C3 * dT / 2^7
     */
    offset =
        ((int64_t)dev->prom[2] * 131072LL) +
        (((int64_t)dev->prom[4] * dT) / 64LL);

    sensitivity =
        ((int64_t)dev->prom[1] * 65536LL) +
        (((int64_t)dev->prom[3] * dT) / 128LL);

    /*
     * MS5837-02BA二阶低温补偿。
     * 数据手册在TEMP < 20°C时要求执行。
     */
    if (temperature < 2000)
    {
        temperature_difference =
            (int64_t)temperature - 2000LL;

        temperature_second_order =
            (11LL * (int64_t)dT * (int64_t)dT) /
            34359738368LL;

        offset_second_order =
            (31LL *
             temperature_difference *
             temperature_difference) /
            8LL;

        sensitivity_second_order =
            (63LL *
             temperature_difference *
             temperature_difference) /
            32LL;
    }

    temperature -= (int32_t)temperature_second_order;
    offset -= offset_second_order;
    sensitivity -= sensitivity_second_order;

    /*
     * P = (D1 * SENS / 2^21 - OFF) / 2^15
     *
     * 得到的P单位为0.01mbar。
     */
    pressure =
        ((((int64_t)d1 * sensitivity) / 2097152LL) -
         offset) /
        32768LL;

    dev->temperature_c =
        (float)temperature / 100.0f;

    dev->pressure_mbar =
        (float)pressure / 100.0f;
}

int ms5837_init(
    ms5837_t *dev,
    const ms5837_bus_t *bus,
    uint8_t dev_addr)
{
    int result;

    if ((dev == NULL) ||
        (bus == NULL) ||
        (bus->write == NULL) ||
        (bus->read == NULL) ||
        (bus->delay_ms == NULL))
    {
        return MS5837_ERROR_ARGUMENT;
    }

    memset(dev, 0, sizeof(*dev));

    dev->bus = *bus;
    dev->dev_addr = dev_addr;

    dev->fluid_density_kg_m3 =
        MS5837_DEFAULT_DENSITY;

    dev->base_pressure_mbar =
        MS5837_DEFAULT_PRESSURE;

    result = ms5837_send_command(
        dev,
        MS5837_CMD_RESET);

    if (result != MS5837_OK)
    {
        dev->last_error = result;
        return result;
    }

    /*
     * 手册要求复位后等待至少2.8ms。
     * 这里使用4ms。
     */
    dev->bus.delay_ms(10U);

    result = ms5837_read_prom(dev);
	//printf("%d\r\n", result);
    if (result != MS5837_OK)
    {
        dev->last_error = result;
        return result;
    }

    dev->initialized = true;
    dev->data_ready = false;
    dev->last_error = MS5837_OK;

    return MS5837_OK;
}

int ms5837_read(ms5837_t *dev)
{
    int result;
    uint32_t d1;
    uint32_t d2;

    if ((dev == NULL) || !dev->initialized)
    {
        return MS5837_ERROR_NOT_READY;
    }

    /*
     * 本次读取尚未完成，旧数据不能继续被当成新数据。
     */
    dev->data_ready = false;

    result = ms5837_convert_and_read(
        dev,
        MS5837_CMD_CONVERT_D1_OSR8192,
        &d1);

    if (result != MS5837_OK)
    {
        dev->last_error = result;
        return result;
    }

    result = ms5837_convert_and_read(
        dev,
        MS5837_CMD_CONVERT_D2_OSR8192,
        &d2);

    if (result != MS5837_OK)
    {
        dev->last_error = result;
        return result;
    }

    ms5837_compensate_02ba(dev, d1, d2);

    /*
     * 使用扩展线性范围做基本异常判断。
     * 如果只接受手册标称工作范围，可改成300～1200mbar。
     */
    if ((dev->pressure_mbar < 10.0f) ||
        (dev->pressure_mbar > 2000.0f) ||
        (dev->temperature_c < -40.0f) ||
        (dev->temperature_c > 85.0f))
    {
        dev->last_error = MS5837_ERROR_RANGE;
        return MS5837_ERROR_RANGE;
    }

    dev->raw_pressure = d1;
    dev->raw_temperature = d2;

    dev->data_ready = true;
    dev->last_error = MS5837_OK;

    return MS5837_OK;
}

int ms5837_calculate_depth(
    const ms5837_t *dev,
    float *depth_m)
{
    float pressure_difference_pa;

    if ((dev == NULL) || (depth_m == NULL))
    {
        return MS5837_ERROR_ARGUMENT;
    }

    if (!dev->initialized || !dev->data_ready)
    {
        return MS5837_ERROR_NOT_READY;
    }

    if (dev->fluid_density_kg_m3 <= 0.0f)
    {
        return MS5837_ERROR_RANGE;
    }

    pressure_difference_pa =
        (dev->pressure_mbar -
         dev->base_pressure_mbar) *
        100.0f;

    *depth_m =
        pressure_difference_pa /
        (dev->fluid_density_kg_m3 *
         MS5837_GRAVITY_M_S2);

    return MS5837_OK;
}

void ms5837_set_base_pressure(
    ms5837_t *dev,
    float pressure_mbar)
{
    if ((dev != NULL) && (pressure_mbar > 0.0f))
    {
        dev->base_pressure_mbar = pressure_mbar;
    }
}

int ms5837_set_fluid_density(
    ms5837_t *dev,
    float density_kg_m3)
{
    if ((dev == NULL) || (density_kg_m3 <= 0.0f))
    {
        return MS5837_ERROR_ARGUMENT;
    }

    dev->fluid_density_kg_m3 = density_kg_m3;

    return MS5837_OK;
}

float ms5837_get_pressure(
    const ms5837_t *dev)
{
    if (dev == NULL)
    {
        return 0.0f;
    }

    return dev->pressure_mbar;
}

float ms5837_get_temperature(
    const ms5837_t *dev)
{
    if (dev == NULL)
    {
        return 0.0f;
    }

    return dev->temperature_c;
}

bool ms5837_is_ready(
    const ms5837_t *dev)
{
    return
        (dev != NULL) &&
        dev->initialized &&
        dev->data_ready;
}

int ms5837_get_last_error(
    const ms5837_t *dev)
{
    if (dev == NULL)
    {
        return MS5837_ERROR_ARGUMENT;
    }

    return dev->last_error;
}