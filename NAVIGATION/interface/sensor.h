#ifndef __SENSORS_H
#define __SENSORS_H

#include "stabilizer_types.h"
#include "stm32f4xx.h" 

#define ACCEL_LPF_CUTOFF_FREQ     30
#define GYRO_LPF_CUTOFF_FREQ      80

#define SENSOR_UPDATE_RATE    RATE_500_HZ   /* 500Hz read IMU */
#define SENSOR_UPDATE_DT      (1.0f / RATE_500_HZ)

/*scale of imu, for the purpose of turning raw imu data to physical data*/
#define GYRO_SCALE    (1.0f / 16.384f)    /* ±2000dps: LSB -> dps */
#define ACCEL_SCALE   (1.0f / 2048.0f)    /* ±16g: LSB -> g */


#define SENSORS_NBR_OF_BIAS_SAMPLES		1024	/* Number of sampling samples for calculating variance */
#define GYRO_VARIANCE_BASE				40000	/* 陀螺仪零偏方差阈值 */
#define SENSORS_ACC_SCALE_SAMPLES  		200		/* 加速计采样个数 */

typedef struct
{
	Axis3f     bias;
	bool       isBiasValueFound;
	bool       isBufferFilled;
	Axis3i16*  bufHead;
	Axis3i16   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
}BiasObj;

void sensorsAcquire(sensorData_t *sensors, const uint32_t tick);
void sensorsTask(void *param);
bool sensorsAreCalibrated(void);

#endif
