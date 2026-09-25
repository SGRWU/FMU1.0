#include <math.h>
#include <stdio.h>
#include "imu_sensor.h"
#include "icm-42688.h"
#include "spi_driver.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "depth_sensor.h"

#include "task.h"
#include "filter.h"

static icm42688_t imu_sensor;
static bool gyroBiasFound = false;
static xQueueHandle imuDataQueue;
static imuData_t imuData;/*pass to stabilizerTask */
static icm42688_data_t raw_imu_data;/**/


static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];

BiasObj	gyroBiasRunning;

static Axis3f gyroBias = {0};

static float accScaleSum = 0;
static float accScale = 1;


static void sensorsBiasObjInit(BiasObj* bias);


void imuDeviceInit(void){

	imu_sensor.bus.read = spi_read_wrapper;
    imu_sensor.bus.write = spi_write_wrapper;
	int ret = icm42688_init(&imu_sensor);
}

/* initialize all the sensors */
void imuInit(void){

		//if(isInit) return;
		
		imuDataQueue= xQueueCreate(1, sizeof(imuData_t));
		sensorsBiasObjInit(&gyroBiasRunning);
		imuDeviceInit();	/*传感器器件初始化*/
		for (uint8_t i = 0; i < 3; i++)// 初始化加速计和陀螺二阶低通滤波
		{
			lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
			lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
		}
		//isInit = true;
	
}
/*Sensor bias initialization*/
static void sensorsBiasObjInit(BiasObj* bias)
{
	bias->isBufferFilled = false;
	bias->bufHead = bias->buffer;
}

/**
 * 往方差缓冲区（循环缓冲区）添加一个新值，缓冲区满后，替换旧的的值
 */
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z)
{
	bias->bufHead->x = x;
	bias->bufHead->y = y;
	bias->bufHead->z = z;
	bias->bufHead++;

	if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
	{
		bias->bufHead = bias->buffer;
		bias->isBufferFilled = true;
	}
}

/*Calculate variance and mean*/
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut)
{
	uint64_t i;
	int64_t sum[3] = {0};
	int64_t sumsq[3] = {0};

	for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
	{
		sum[0] += bias->buffer[i].x;
		sum[1] += bias->buffer[i].y;
		sum[2] += bias->buffer[i].z;
		sumsq[0] += bias->buffer[i].x * bias->buffer[i].x;
		sumsq[1] += bias->buffer[i].y * bias->buffer[i].y;
		sumsq[2] += bias->buffer[i].z * bias->buffer[i].z;
	}

	varOut->x = (sumsq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->y = (sumsq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->z = (sumsq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

	meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}
/*Sensor searches for bias value*/
static bool sensorsFindBiasValue(BiasObj* bias)
{
	bool foundbias = false;

	if (bias->isBufferFilled)
	{
		
		Axis3f mean;
		Axis3f variance;
		sensorsCalculateVarianceAndMean(bias, &variance, &mean);
		
		if (variance.x < GYRO_VARIANCE_BASE && variance.y < GYRO_VARIANCE_BASE && variance.z < GYRO_VARIANCE_BASE)
			
		{

			bias->bias.x = mean.x;
			bias->bias.y = mean.y;
			bias->bias.z = mean.z;
			foundbias = true;
			bias->isBiasValueFound= true;
		}else
			bias->isBufferFilled=false;
	}
	return foundbias;
}
/**
 * Calculate gyroscope variance
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
	sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

	if (!gyroBiasRunning.isBiasValueFound)
	{
		sensorsFindBiasValue(&gyroBiasRunning);
	}

	gyroBiasOut->x = gyroBiasRunning.bias.x;
	gyroBiasOut->y = gyroBiasRunning.bias.y;
	gyroBiasOut->z = gyroBiasRunning.bias.z;

	return gyroBiasRunning.isBiasValueFound;
}
/*Second order low-pass filter*/
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in)
{
	for (uint8_t i = 0; i < 3; i++) 
	{
		in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
	}
}
/**/
bool sensorsAreCalibrated(void){
	
		return gyroBiasFound;
	
}

bool imuReadLatest(imuData_t *sensors){
	
		return (pdTRUE == xQueueReceive(imuDataQueue, sensors, 0));
	
}


void sensorsAcquire(sensorData_t *sensors, const uint32_t tick)
{
		
    depthData_t latest_depth;
	imuData_t latest_imu;

    if(imuReadLatest(&latest_imu)){
		sensors->acc = latest_imu.acc;
		sensors->gyro = latest_imu.gyro;
	}
	
	if(depthReadLatest(&latest_depth)){
		sensors->depth = latest_depth;
	}
}


/**
 * 根据样本计算重力加速度缩放因子
 */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
	static bool accBiasFound = false;
	static uint32_t accScaleSumCount = 0;

	if (!accBiasFound)
	{
		accScaleSum += sqrtf((ax*ACCEL_SCALE)*(ax*ACCEL_SCALE) + (ay*ACCEL_SCALE)*(ay*ACCEL_SCALE) + (az*ACCEL_SCALE)*(az*ACCEL_SCALE) );
		accScaleSumCount++;

		if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
		{
			accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
			accBiasFound = true;
		}
	}

	return accBiasFound;
}

void processAccGyroMeasurements(void){

    float gx = raw_imu_data.gyro_x - gyroBias.x;
    float gy = raw_imu_data.gyro_y - gyroBias.y;
    float gz = raw_imu_data.gyro_z - gyroBias.z;
	
	  /* Gyro bias calibration - interface placeholder */
    gyroBiasFound = processGyroBias(raw_imu_data.gyro_x, raw_imu_data.gyro_y, raw_imu_data.gyro_z, &gyroBias);
	  
		if (gyroBiasFound)
		{
			processAccScale(raw_imu_data.accel_x, raw_imu_data.accel_y, raw_imu_data.accel_z);	/*calcultate the accScale*/
		}
		
    imuData.gyro.x =  (gx) * GYRO_SCALE;    /* IMU_X -> Vehicle X */
    imuData.gyro.y = -(gy) * GYRO_SCALE;    /* IMU_Y -> Vehicle -Y */
    imuData.gyro.z = -(gz) * GYRO_SCALE;    /* IMU_Z -> Vehicle -Z (down) */
    applyAxis3fLpf(gyroLpf, &imuData.gyro);

    /* Accelerometer: same transform */
    imuData.acc.x =  (raw_imu_data.accel_x) * ACCEL_SCALE / accScale;
    imuData.acc.y = -(raw_imu_data.accel_y) * ACCEL_SCALE / accScale;
    imuData.acc.z = -(raw_imu_data.accel_z) * ACCEL_SCALE / accScale;
    applyAxis3fLpf(accLpf, &imuData.acc);
			
}
void imuTask(void *param){
	
		TickType_t lastWakeTime;
	
		const TickType_t period = pdMS_TO_TICKS(1U);
	
		int read_result;
	
		imuInit();
	
		lastWakeTime = xTaskGetTickCount();
	
		while(1){

        read_result = icm42688_read_all(&imu_sensor, &raw_imu_data);
				
				if(read_result == 0){
					
						processAccGyroMeasurements();
						
						xQueueOverwrite(imuDataQueue, &imuData);	
					
				}
			
				vTaskDelayUntil(&lastWakeTime, period);
		}
}
