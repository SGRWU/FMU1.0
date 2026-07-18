#ifndef ATTITUDE_ESTIMATION_H
#define ATTITUDE_ESTIMATION_H
#include "sensors_types.h"
#include "stabilizer_types.h"


void imuUpdate(Axis3f acc, Axis3f gyro, state_t *state, float dt);
bool getIsCalibrated(void);
float invSqrt(float x);

#endif
