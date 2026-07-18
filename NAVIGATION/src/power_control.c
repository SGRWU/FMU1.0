#include "power_control.h"
#include "thrusters.h"


static thrusterPWM_t thrusterPWM;
static thrusterPWM_t thrusterPWMset = {0, 0, 0, 0};

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

static int clamp(int val, int min, int max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

void powerControl(volatile control_t *control)
{

    /* 缩放 PID 输出到 PWM 偏移量 ±500 */
    int pitch_off = control->pitch * 0.02f;        // ±32768 → ±512
    int yaw_off   = control->yaw * 0.02f;          // ±32768 → ±512
    int heave_off = (int)(control->heave);      // 已在 jetson_link 里映射为 ±500
    int thrust_off = (int)(control->thrust);  // 0~1 → -500~+500

    /* 混控 */
    int t1 = PWM_CENTER + heave_off + pitch_off;
    int t2 = PWM_CENTER - heave_off + pitch_off;
    int t3 = PWM_CENTER + thrust_off + yaw_off;
    int t4 = PWM_CENTER + thrust_off - yaw_off;

    /* 限幅 1000~2000 */
    thrustersSetRatio(THRUSTER_T1, clamp(t1, PWM_MIN, PWM_MAX));
    thrustersSetRatio(THRUSTER_T2, clamp(t2, PWM_MIN, PWM_MAX));
    thrustersSetRatio(THRUSTER_T3, clamp(t3, PWM_MIN, PWM_MAX));
    thrustersSetRatio(THRUSTER_T4, clamp(t4, PWM_MIN, PWM_MAX));
}