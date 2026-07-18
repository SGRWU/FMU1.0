#ifndef __STABILIZER_H
#define __STABILIZER_H

#include <stdbool.h>
#include <stdint.h>
#include "stabilizer_types.h"

#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)


#define MAIN_LOOP_RATE 	RATE_1000_HZ
#define MAIN_LOOP_DT	(uint32_t)(1000/MAIN_LOOP_RATE)	/*ms*/

#define ATTITUDE_ESTIMAT_RATE	RATE_250_HZ	//◊ÀÃ¨Ω‚À„ÀŸ¬ 
#define ATTITUDE_ESTIMAT_DT		(1.0/RATE_250_HZ)

#define DEG2RAD		0.017453293f	/*¶–/180 */
#define RAD2DEG		57.29578f		/* 180/¶– */
void stabilizerTask(void* param);
void stabilizerInit(void);
void getAttitudeData(attitude_t* get);
void getAngleRateData(Axis3f* gyro_get);

#endif
