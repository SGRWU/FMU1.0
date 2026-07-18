#include "attitude_pid.h"
#include "config_param.h"


PidObject pidRateRoll;      // unused
PidObject pidRatePitch;
PidObject pidRateYaw;

PidObject pidRoll;          // unused
PidObject pidPitch;
PidObject pidYaw;

/* ???? */
#define PID_RATE_PITCH_INTEGRAL_LIMIT  500.0f
#define PID_RATE_YAW_INTEGRAL_LIMIT    50.0f

#define PID_PITCH_INTEGRAL_LIMIT       30.0f
#define PID_YAW_INTEGRAL_LIMIT         180.0f

void angleControlInit(float velocityPidDt, float posPidDt)
{

    pidInit(&pidRatePitch, 0, configParam.pidRate.pitch, velocityPidDt);
    pidSetIntegralLimit(&pidRatePitch, PID_RATE_PITCH_INTEGRAL_LIMIT);
    pidSetOutputLimit(&pidRatePitch, pidRatePitch_OUTPUT_LIMIT);

    pidInit(&pidRateYaw, 0, configParam.pidRate.yaw, velocityPidDt);
    pidSetIntegralLimit(&pidRateYaw, PID_RATE_YAW_INTEGRAL_LIMIT);
    pidSetOutputLimit(&pidRateYaw, pidRateYaw_OUTPUT_LIMIT);


    pidInit(&pidPitch, 0, configParam.pidAngle.pitch, posPidDt);
    pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRAL_LIMIT);
    pidSetOutputLimit(&pidPitch, pidPitch_OUTPUT_LIMIT);

    pidInit(&pidYaw, 0, configParam.pidAngle.yaw, posPidDt);
    pidSetIntegralLimit(&pidYaw, PID_YAW_INTEGRAL_LIMIT);
    pidSetOutputLimit(&pidYaw, pidYaw_OUTPUT_LIMIT);
}
