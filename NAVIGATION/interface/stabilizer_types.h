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
	Axis3f acc;
	Axis3f gyro;
} sensorData_t;

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


/* Orientation as a quaternion */
typedef struct quaternion_s 
{
	uint32_t timestamp;

	union 
	{
		struct 
		{
			float q0;
			float q1;
			float q2;
			float q3;
		};
		struct 
		{
			float x;
			float y;
			float z;
			float w;
		};
	};
} quaternion_t;


typedef struct
{
	attitude_t attitude;
	quaternion_t attitudeQuaternion;
	point_t position;
	velocity_t velocity;
	acc_t acc;
	bool isRCLocked;
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
	attitude_t attitudeRate;	// deg/s
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
