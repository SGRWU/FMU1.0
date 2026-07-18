#include <math.h>
#include "state_control.h"
#include "stabilizer.h"
#include "attitude_pid.h"
#include "config_param.h"
#include "jetson_link.h"
#include "pid.h"


extern PidObject pidPitch;
extern PidObject pidRatePitch;

void stateControl(volatile control_t *control, sensorData_t *sensors, state_t *state, setpoint_t *setpoint, const uint32_t tick)
{
    getSetpoint(setpoint);   

		/* 角度环 (只用 pitch) */
		if (RATE_DO_EXECUTE(ANGEL_PID_RATE, tick))
		{
				float error = setpoint->attitude.pitch - state->attitude.pitch;

				if (fabsf(error) < 10.0f)       // ±10° 以内不纠正
						error = 0.0f;

				float desRatePitch = pidUpdate(&pidPitch, error);
				pidSetDesired(&pidRatePitch, desRatePitch);
		}

		/* 角速度环 */
		if (RATE_DO_EXECUTE(ANGEL_RATE_PID_RATE, tick))
		{
				float pitch_rate_err = pidRatePitch.desired - sensors->gyro.y;
				control->pitch = pidUpdate(&pidRatePitch, pitch_rate_err);

				pidSetDesired(&pidRateYaw, setpoint->attitude.yaw);     
				float yaw_rate_err = pidRateYaw.desired - sensors->gyro.z;
				control->yaw = pidUpdate(&pidRateYaw, yaw_rate_err);

				control->heave = setpoint->heave;
				control->thrust = setpoint->thrust;
		}
}
	