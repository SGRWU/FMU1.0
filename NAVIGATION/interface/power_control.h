#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H
#include "stabilizer_types.h"

#define PWM_CENTER  1500
#define PWM_RANGE   500
#define PWM_MIN     1000
#define PWM_MAX     2000
#define DEADBAND    20   // ¡À20¦Ìs ËÀÇø

typedef struct 
{
	uint32_t t1;
	uint32_t t2;
	uint32_t t3;
	uint32_t t4;
	
}thrusterPWM_t;


void powerControl(volatile control_t *control);


#endif 