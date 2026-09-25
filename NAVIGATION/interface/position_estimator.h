#ifndef __POSITION_ESTIMATOR_H
#define __POSITION_ESTIMATOR_H

#include <stdbool.h>
#include "stabilizer_types.h"


/* 位置估计器运行频率 */
#define POSITION_ESTIMATOR_RATE_HZ          250.0f
#define POSITION_ESTIMATOR_DT_S             (1.0f / POSITION_ESTIMATOR_RATE_HZ)

/* 传感器数据超时时间 */
#define POSITION_XY_TIMEOUT_MS              300U
#define POSITION_Z_TIMEOUT_MS               200U

/* 测量差分允许的时间范围 */
#define POSITION_MIN_MEASURE_DT_S           0.005f
#define POSITION_MAX_MEASURE_DT_S           0.500f

/* 速度二阶低通滤波器截止频率 */
#define VELOCITY_XY_LPF_CUTOFF_HZ           2.0f
#define VELOCITY_Z_LPF_CUTOFF_HZ            2.0f

#define UWB_TAG_DEPTH_OFFSET_M 0.0f/* 记得修改！！！！！ */

/* 位置状态有效标志 */
#define LOCAL_POSITION_XY_VALID             (1U << 0)
#define LOCAL_POSITION_Z_VALID              (1U << 1)
#define LOCAL_VELOCITY_XY_VALID             (1U << 2)
#define LOCAL_VELOCITY_Z_VALID              (1U << 3)

#define LOCAL_POSITION_ALL_VALID            (LOCAL_POSITION_XY_VALID | LOCAL_POSITION_Z_VALID | LOCAL_VELOCITY_XY_VALID | LOCAL_VELOCITY_Z_VALID)


bool positionEstimatorInit(void);
void positionEstimate(const sensorData_t *sensor_data, state_t *state, float dt_s);
bool positionReadLatest(localPositionNed_t *position);

#endif