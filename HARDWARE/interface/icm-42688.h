/**
 * @file icm-42688.h
 * @brief ICM-42688 6-axis IMU sensor driver header
 * @author Yusuf Karaböcek
 * @date July 2025
 */

#ifndef ICM_42688_H
#define ICM_42688_H

#include <stdint.h>
#include <stdbool.h>

/* Device Configuration */
#define ICM_42688_I2C_ADDRESS    0x68    /**< Default I2C address */
#define WHO_AM_I_REG             0x75    /**< Device identification register */
#define ICM42688_REG_DEVICE_CONFIG 0x11  /**< Device configuration register */
#define ICM42688_PWR_MGMT0       0x4E    /**< Power management register */

/* Temperature Data Registers */
#define ICM42688_REG_TEMP_DATA1  0x1D    /**< Temperature data MSB */
#define ICM42688_REG_TEMP_DATA2  0x1E    /**< Temperature data LSB */

/* Accelerometer Data Registers */
#define ICM42688_REG_ACCEL_DATA_X1 0x1F  /**< Accelerometer X-axis MSB */
#define ICM42688_REG_ACCEL_DATA_X0 0x20  /**< Accelerometer X-axis LSB */
#define ICM42688_REG_ACCEL_DATA_Y1 0x21  /**< Accelerometer Y-axis MSB */
#define ICM42688_REG_ACCEL_DATA_Y0 0x22  /**< Accelerometer Y-axis LSB */
#define ICM42688_REG_ACCEL_DATA_Z1 0x23  /**< Accelerometer Z-axis MSB */
#define ICM42688_REG_ACCEL_DATA_Z0 0x24  /**< Accelerometer Z-axis LSB */

/* Gyroscope Data Registers */
#define ICM42688_REG_GYRO_DATA_X1 0x25   /**< Gyroscope X-axis MSB */
#define ICM42688_REG_GYRO_DATA_X0 0x26   /**< Gyroscope X-axis LSB */
#define ICM42688_REG_GYRO_DATA_Y1 0x27   /**< Gyroscope Y-axis MSB */
#define ICM42688_REG_GYRO_DATA_Y0 0x28   /**< Gyroscope Y-axis LSB */
#define ICM42688_REG_GYRO_DATA_Z1 0x29   /**< Gyroscope Z-axis MSB */
#define ICM42688_REG_GYRO_DATA_Z0 0x2A   /**< Gyroscope Z-axis LSB */

/* Configuration Registers */
#define ICM42688_REG_ACCEL_CONFIG0 0x50  /**< Accelerometer configuration */
#define ICM42688_REG_GYRO_CONFIG0  0x4F  /**< Gyroscope configuration 这儿要了解清楚 量程设置的是多少*/

/**
 * @brief Sensor data structure
 */
typedef struct {
    int16_t accel_x;    /**< Accelerometer X-axis data */
    int16_t accel_y;    /**< Accelerometer Y-axis data */
    int16_t accel_z;    /**< Accelerometer Z-axis data */
    int16_t gyro_x;     /**< Gyroscope X-axis data */
    int16_t gyro_y;     /**< Gyroscope Y-axis data */
    int16_t gyro_z;     /**< Gyroscope Z-axis data */
    int16_t temp;       /**< Temperature data */
} icm42688_data_t;

/**
 * @brief Communication bus abstraction structure
 */
typedef struct {
    int (*read)(uint8_t reg, uint8_t *data, uint16_t len);   /**< Read function pointer */
    int (*write)(uint8_t reg, uint8_t *data, uint16_t len);  /**< Write function pointer */
} icm42688_bus_t;

/**
 * @brief Main sensor context structure
 */
typedef struct {
    icm42688_bus_t bus; /**< Communication bus interface */
} icm42688_t;

// 在 icm-42688.h 中添加
typedef struct {
    float temp_c;               // 摄氏度
    float accel_x_g, accel_y_g, accel_z_g;  // g
    float gyro_x_dps, gyro_y_dps, gyro_z_dps; // °/s
} icm42688_physical_data_t;
//改1
// 添加ADC转换函数声明
void icm42688_convert_adc_to_physical(icm42688_data_t *raw_data, icm42688_physical_data_t *phy_data);
//改2
/**
 * @brief Initialize ICM-42688 sensor
 * @param dev Pointer to sensor context
 * @return 0 on success, negative value on error
 */
int icm42688_init(icm42688_t *dev);

/**
 * @brief Read all sensor data (accelerometer, gyroscope, temperature)
 * @param dev Pointer to sensor context
 * @param data Pointer to data structure
 * @return 0 on success, negative value on error
 */
int icm42688_read_all(icm42688_t *dev, icm42688_data_t *data);

/**
 * @brief Read temperature data
 * @param dev Pointer to sensor context
 * @param temp Pointer to temperature value
 * @return 0 on success, negative value on error
 */
int icm42688_read_temp(icm42688_t *dev, int16_t *temp);

/**
 * @brief Read accelerometer data
 * @param dev Pointer to sensor context
 * @param accel_x Pointer to X-axis acceleration
 * @param accel_y Pointer to Y-axis acceleration
 * @param accel_z Pointer to Z-axis acceleration
 * @return 0 on success, negative value on error
 */
int icm42688_read_accel(icm42688_t *dev, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z);

/**
 * @brief Read gyroscope data
 * @param dev Pointer to sensor context
 * @param gyro_x Pointer to X-axis angular velocity
 * @param gyro_y Pointer to Y-axis angular velocity
 * @param gyro_z Pointer to Z-axis angular velocity
 * @return 0 on success, negative value on error
 */
int icm42688_read_gyro(icm42688_t *dev, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

#endif // ICM_42688_H