#include "thrusters.h"
#include "FreeRTOS.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

static int pos = 0;

uint32_t motor_ratios[] = {0, 0, 0, 0};
static const uint32_t MOTORS[] = { THRUSTER_T1, THRUSTER_T2, THRUSTER_T3, THRUSTER_T4 };

void thrustersInit(void){
	    
		
		
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);


}


void thrustersTest(void){
    static uint32_t tick = 0;
		pos = 0;
    static int dir = 1;

		tick++;
		if (tick < 100) return;      /* 每 10 步更新一次（1ms 调用一次） */
		tick = 0;

		pos += dir * 5;              /* 每次移 5 个计数值 */

		if (pos >= PWM_RANGE) {          /* 正转顶 → 往回 */
				pos = PWM_RANGE;
				dir = -1;
		} else if (pos <= -PWM_RANGE) {  /* 反转顶 → 往回 */
				pos = -PWM_RANGE;
				dir = 1;
		} else if (pos == 0 && dir == -1) {
				/* 经过中位继续向下 */
		}

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_CENTER + pos);


}


void thrustersSetRatio(uint32_t id, uint16_t ithrust)
{
								/*电池补偿暂不启用*/
//		#ifdef ENABLE_THRUST_BAT_COMPENSATED		
//			float thrust = ((float)ithrust / 65536.0f) * 60;
//			float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
//			float supply_voltage = pmGetBatteryVoltage();
//			float percentage = volts / supply_voltage;
//			percentage = percentage > 1.0f ? 1.0f : percentage;
//			ratio = percentage * UINT16_MAX;
//			motor_ratios[id] = ratio;
//		#endif
		
		switch(id)
		{
			case 0:		/*THRUSTER_M1*/
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ithrust);
				break;
			case 1:		/*THRUSTER_M2*/
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ithrust);
				break;
			case 2:		/*THRUSTER_M3*/
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ithrust);
				break;
			case 3:		/*THRUSTER_M4*/	
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ithrust);
				break;
			default: break;
		}	
}