#ifndef __STABILIZER_TYPES_H
#define __STABILIZER_TYPES_H
#include <stdbool.h>
#include "sensors_types.h"
#include "stm32f4xx.h" 

#if defined(__CC_ARM) 
	#pragma anon_unions
#endif

typedef struct
{
    uint32_t timestamp_ms;

    float x;
    float y;
    float z;

    float vx;
    float vy;
    float vz;

    uint8_t valid_flags;
} localPositionNed_t;

/* 深度传感器数据 */
typedef struct
{
    uint32_t timestamp;

    float pressure_mbar;
    float temperature_c;

    float depth_raw_m;
    float depth_filtered_m;

    uint16_t error_count;

    bool valid;
    bool calibrated;
} depthData_t;


/* 陀螺仪数据类型 */
typedef struct
{
	
	Axis3f acc;
	Axis3f gyro;
	
} imuData_t;


/* 传感器数据类型 */
typedef struct
{
    Axis3f acc;
    Axis3f gyro;

    depthData_t depth;
} sensorData_t;

/* 3×1向量 */
struct  vec3_s 
{
	uint32_t timestamp;

	float x;
	float y;
	float z;
};

typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;


typedef struct  
{
	uint32_t timestamp;	
	
	float roll;
	float pitch;
	float yaw;
} attitude_t;


typedef struct
{
	attitude_t attitude;
	point_t position;
	velocity_t velocity;
	acc_t acc;
	uint8_t local_position_valid_flags;
} state_t;

/*contorl value of joystick output*/
typedef struct
{
	float pitch;
	float yaw;
	float thrust;
	float heave;

} control_t;

/*destination of the controller*/
typedef struct
{
	attitude_t attitude;		// deg	
	point_t position;         	// m
	velocity_t velocity;      	// m/s
	float heave;
	float thrust;
} setpoint_t;


#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#define MAIN_LOOP_RATE 	RATE_1000_HZ
#define MAIN_LOOP_DT	(uint32_t)(1000/MAIN_LOOP_RATE)	/*ms*/

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)/*turn Hz to ticks counter*/


#endif
