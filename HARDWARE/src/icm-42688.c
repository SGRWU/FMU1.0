/**
 * @file icm-42688.c
 * @brief ICM-42688 6-axis IMU sensor driver implementation
 * @author Yusuf Karaböcek
 * @date July 2025
 */

#include "icm-42688.h"
/**
 * @brief Write single register
 * @param dev Pointer to sensor context
 * @param reg Register address
 * @param value Value to write
 * @return 0 on success, negative value on error
 */
static int write_register(icm42688_t *dev, uint8_t reg, uint8_t value) {
    return dev->bus.write(reg, &value, 1);
}

/**
 * @brief Read single register
 * @param dev Pointer to sensor context
 * @param reg Register address
 * @param value Pointer to store read value
 * @return 0 on success, negative value on error
 */
static int read_register(icm42688_t *dev, uint8_t reg, uint8_t *value) {
    return dev->bus.read(reg, value, 1);
}

int icm42688_init(icm42688_t *dev) {
    if(!dev) return -1;

    uint8_t who_am_i = 0;

    /* Read device ID */
    if (read_register(dev, WHO_AM_I_REG, &who_am_i) != 0) {
        return -2;
    }

    /* Verify device ID */
    if(who_am_i != 0x47){
        return -3;
    }

    /* Reset device */
    write_register(dev, ICM42688_REG_DEVICE_CONFIG, 0x01);
    /* Simple delay loop */
    HAL_Delay(10);  /* wait for reset */

    /* Power management: enable accelerometer and gyroscope */
    if (write_register(dev, ICM42688_PWR_MGMT0, 0x0F) != 0) {
        return -4;
    }
        HAL_Delay(10);

    return 0;
}

int icm42688_read_temp(icm42688_t *dev, int16_t *temp) {
    if(!dev || !temp) return -1;

    uint8_t buf[2];
    if(dev->bus.read(ICM42688_REG_TEMP_DATA1, buf, 2) != 0) return -2;
    
    *temp = (int16_t)((buf[0] << 8) | buf[1]);
    return 0;
}

int icm42688_read_accel(icm42688_t *dev, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z) {
    if(!dev || !accel_x || !accel_y || !accel_z) return -1;

    uint8_t buf[6];
    if(dev->bus.read(ICM42688_REG_ACCEL_DATA_X1, buf, 6) != 0) return -2;

    *accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    *accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    *accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    return 0;
}

int icm42688_read_gyro(icm42688_t *dev, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    if(!dev || !gyro_x || !gyro_y || !gyro_z) return -1;

    uint8_t buf[6];
    if(dev->bus.read(ICM42688_REG_GYRO_DATA_X1, buf, 6) != 0) return -2;

    *gyro_x = (int16_t)((buf[0] << 8) | buf[1]);
    *gyro_y = (int16_t)((buf[2] << 8) | buf[3]);
    *gyro_z = (int16_t)((buf[4] << 8) | buf[5]);
    return 0;
}

int icm42688_read_all(icm42688_t *dev, icm42688_data_t *data) {
		
    if(!dev || !data) return -1;

    uint8_t buf[14];
    
    if(dev->bus.read(ICM42688_REG_TEMP_DATA1, buf, 14) != 0) return -2;

    data->temp = (int16_t)((buf[0] << 8) | buf[1]);

    data->accel_x = (int16_t)((buf[2] << 8) | buf[3]);
    data->accel_y = (int16_t)((buf[4] << 8) | buf[5]);
    data->accel_z = (int16_t)((buf[6] << 8) | buf[7]);

    data->gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
    data->gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
    data->gyro_z = (int16_t)((buf[12] << 8) | buf[13]);

    return 0;
}
/**
 * @brief 将ADC原始数据转换为物理量
 * @param raw_data 原始ADC数据
 * @param phy_data 转换后的物理量数据
 *
 * 默认使用标准灵敏度:
 * - 加速度: ±16g (2048 LSB/g)
 * - 陀螺仪: ±2000dps (16.384 LSB/(°/s))
 */
void icm42688_convert_adc_to_physical(icm42688_data_t *raw_data, icm42688_physical_data_t *phy_data) {
    if (!raw_data || !phy_data) return;

    // 温度转换: °C = (ADC值 / 132.48) + 25
    phy_data->temp_c = raw_data->temp / 132.48f + 25.0f;

    // 加速度转换: g = ADC值 / 2048
    phy_data->accel_x_g = raw_data->accel_x / 2048.0f;
    phy_data->accel_y_g = raw_data->accel_y / 2048.0f;
    phy_data->accel_z_g = raw_data->accel_z / 2048.0f;

    // 陀螺仪转换: °/s = ADC值 / 16.384
    phy_data->gyro_x_dps = raw_data->gyro_x / 16.384f;
    phy_data->gyro_y_dps = raw_data->gyro_y / 16.384f;
    phy_data->gyro_z_dps = raw_data->gyro_z / 16.384f;
}
