#ifndef __ATTITUDE_PID_H
#define __ATTITUDE_PID_H

#include "pid.h"


extern PidObject pidRateRoll;    // unused (no roll loop)
extern PidObject pidRatePitch;
extern PidObject pidRateYaw;

extern PidObject pidRoll;    // unused (no roll loop)
extern PidObject pidPitch;
extern PidObject pidYaw;


#define pidRatePitch_OUTPUT_LIMIT  25000.0f   // Pitch
#define pidRateYaw_OUTPUT_LIMIT    40000.0f    // Yaw 

#define pidPitch_OUTPUT_LIMIT      1200.0f  // Pitch 
#define pidYaw_OUTPUT_LIMIT        120.0f   // Yaw 


void angleControlInit(float velocityPidDt, float posPidDt);

#endif
