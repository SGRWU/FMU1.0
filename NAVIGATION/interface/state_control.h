#ifndef __STATE_CONTROL_H
#define __STATE_CONTROL_H

#include "stabilizer_types.h"


#define ANGEL_PID_RATE 									RATE_500_HZ
#define ANGEL_RATE_PID_RATE 						RATE_250_HZ


void stateControl(volatile control_t *control, sensorData_t *sensors, state_t *state, setpoint_t *setpoint, const uint32_t tick);



#endif