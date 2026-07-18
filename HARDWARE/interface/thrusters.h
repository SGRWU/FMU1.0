#ifndef __THRUSTERS_H
#define __THRUSTERS_H
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/* 96M主频下 8位精度输出375K PWM */
#define TIM_CLOCK_HZ 				96000000
#define thrusters_PWM_BITS           	8
#define thrusters_PWM_PERIOD         	((1<<thrusters_PWM_BITS) - 1)
#define thrusters_PWM_PRESCALE       	0


#define PWM_CENTER  1500 /*pwm when motor is stationary*/
#define PWM_RANGE   500  /*motor pwm range*/

#define ENABLE_THRUST_BAT_COMPENSATED	/*使能电池油门补偿*/

#define NBR_OF_thrusters 	4
#define THRUSTER_T1  		0
#define THRUSTER_T2  		1
#define THRUSTER_T3  		2
#define THRUSTER_T4  		3

#define thrusters_TEST_RATIO         (uint16_t)(0.2*(1<<16))	//20%
#define thrusters_TEST_ON_TIME_MS    50
#define thrusters_TEST_DELAY_TIME_MS 150


void thrustersInit(void);		/*电机初始化*/
void thrustersTest(void);		/*电机测试*/
void thrustersSetRatio(uint32_t id, uint16_t ithrust);	/*设置电机占空比*/

#endif