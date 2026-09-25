#include "stabilizer.h"
#include "imu_sensor.h"
#include "system.h"
#include "FreeRTOS.h"		 
#include "task.h"
#include "stabilizer_types.h"
#include "sensfusion6.h"
#include "power_control.h"
#include "thrusters.h"
#include "jetson_link.h"
#include "state_control.h"
#include "position_estimator.h"

static setpoint_t 	setpoint;		
static volatile control_t control;
static sensorData_t sensorData;
static state_t state;

extern TIM_HandleTypeDef htim1;

void stabilizerInit(void){
	
	
	
		//if (isInit) return;
    /* 后续在这里初始化 PID、控制参数等 */
		thrustersInit();
    //isInit = true;

}

void getAttitudeData(attitude_t* att_get)
{
    att_get->roll  = state.attitude.roll;
    att_get->pitch = state.attitude.pitch;
    att_get->yaw   = state.attitude.yaw;
}

void getAngleRateData(Axis3f* gyro_get){
		gyro_get->x = sensorData.gyro.x;
		gyro_get->y = sensorData.gyro.y;
		gyro_get->z = sensorData.gyro.z;
}

void stabilizerTask(void* param){
	
	
	uint32_t tick = 0;//count of ticks
	uint32_t lastWakeTime = xTaskGetTickCount();
	
	if (!positionEstimatorInit())
    {
        vTaskDelete(NULL);
        return;
    }
	
	while(!sensorsAreCalibrated())
	{
		//vTaskDelayUntil(&lastWakeTime, MAIN_LOOP_DT);
		vTaskDelay(1);
	}
	lastWakeTime = xTaskGetTickCount();
	while(1){
		
		//vTaskDelay(1);
		vTaskDelayUntil(&lastWakeTime, 1);/* loop per 1 ms */
		
		/* acquire all the sensors data */
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			sensorsAcquire(&sensorData, tick);
		}
		
		/* calculate quanterion and euler's angle */
		/* estimate the position and velocity */
		if (RATE_DO_EXECUTE(RATE_250_HZ, tick))
		{
			imuUpdate(sensorData.acc, sensorData.gyro, &state, ATTITUDE_ESTIMAT_DT);
			positionEstimate(&sensorData, &state, POSITION_ESTIMATOR_DT_S);
		}
		
		/* calculate the output according to input and feedback */
		stateControl(&control, &sensorData, &state, &setpoint, tick);
		
		/* Control motor output */
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))                
		{
			powerControl(&control);	
		}
		
		tick++;
	}
}
