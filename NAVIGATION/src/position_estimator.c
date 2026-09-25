#include "position_estimator.h"
#include "uwb.h"
#include "filter.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <string.h>


/* 滤波器相关 */
static lpf2pData velocityXLpf;
static lpf2pData velocityYLpf;
static lpf2pData velocityZLpf;

static float rawVxHold;
static float rawVyHold;
static float rawVzHold;

/* 位置估计相关 */
static QueueHandle_t localPositionQueue = NULL;
static localPositionNed_t localPosition;

static uint32_t previousUwbTimestampMs;
static uint32_t previousDepthTimestampMs;

static bool xyInitialized;
static bool zInitialized;

static float previousMeasureX;
static float previousMeasureY;
static float previousMeasureZ;

static bool positionMeasureDtValid(float dt_s)
{
    return isfinite(dt_s) && dt_s >= POSITION_MIN_MEASURE_DT_S && dt_s <= POSITION_MAX_MEASURE_DT_S;
}

static void positionResetXyVelocity(void)
{
    rawVxHold = 0.0f;
    rawVyHold = 0.0f;

    localPosition.vx = 0.0f;
    localPosition.vy = 0.0f;

    lpf2pReset(&velocityXLpf, 0.0f);
    lpf2pReset(&velocityYLpf, 0.0f);

    localPosition.valid_flags &= (uint8_t)(~LOCAL_VELOCITY_XY_VALID);
}

static void positionResetZVelocity(void)
{
    rawVzHold = 0.0f;
    localPosition.vz = 0.0f;

    lpf2pReset(&velocityZLpf, 0.0f);

    localPosition.valid_flags &= (uint8_t)(~LOCAL_VELOCITY_Z_VALID);
}

/* 如果两次检测时间间隔太大，重置 */
static void positionCheckTimeout(uint32_t now_ms)
{
    if (xyInitialized && (now_ms - previousUwbTimestampMs) > POSITION_XY_TIMEOUT_MS)
    {
        xyInitialized = false;

        localPosition.valid_flags &= (uint8_t)(~LOCAL_POSITION_XY_VALID);

        positionResetXyVelocity();
    }

    if (zInitialized && (now_ms - previousDepthTimestampMs) > POSITION_Z_TIMEOUT_MS)
    {
        zInitialized = false;

        localPosition.valid_flags &= (uint8_t)(~LOCAL_POSITION_Z_VALID);

        positionResetZVelocity();
    }
}

/* 通过估计，提高state的更新频率 */
static void positionPredict(float dt_s)
{
	/* 没有用卡尔曼滤波 */
    if ((localPosition.valid_flags & (LOCAL_POSITION_XY_VALID | LOCAL_VELOCITY_XY_VALID)) == (LOCAL_POSITION_XY_VALID | LOCAL_VELOCITY_XY_VALID))
    {
        localPosition.x += localPosition.vx * dt_s;
        localPosition.y += localPosition.vy * dt_s;
    }

    if ((localPosition.valid_flags & (LOCAL_POSITION_Z_VALID | LOCAL_VELOCITY_Z_VALID)) == (LOCAL_POSITION_Z_VALID | LOCAL_VELOCITY_Z_VALID))
    {
        localPosition.z += localPosition.vz * dt_s;
    }
}


/* 读取UWB，并计算速度 */
static void positionUpdateUwbMeasurement(void)
{
    uwbPosition_t uwbPosition;
    float measureDtS;
    float newRawVx;
    float newRawVy;

    if (!uwbGetPosition(&uwbPosition))
    {
        return;
    }

    if (uwbPosition.timestamp_ms == previousUwbTimestampMs)
    {
        return;
    }

    if (!isfinite(uwbPosition.x_m) || !isfinite(uwbPosition.y_m))
    {
        return;
    }

    if (xyInitialized)
    {
        measureDtS = (uwbPosition.timestamp_ms - previousUwbTimestampMs) * 0.001f;

        if (positionMeasureDtValid(measureDtS))
        {
            newRawVx = (uwbPosition.x_m - previousMeasureX) / measureDtS;
            newRawVy = (uwbPosition.y_m - previousMeasureY) / measureDtS;

            if (isfinite(newRawVx) && isfinite(newRawVy))
            {
                rawVxHold = newRawVx;
                rawVyHold = newRawVy;

                localPosition.valid_flags |= LOCAL_VELOCITY_XY_VALID;
            }
            else
            {
                positionResetXyVelocity();
            }
        }
        else
        {
            positionResetXyVelocity();
        }
    }
    else
    {
        positionResetXyVelocity();
    }

    /*
     * 当前版本直接使用UWB解算结果修正位置。
     * 后续如果实测位置跳动明显，可在此处增加位置滤波。
     */
    localPosition.x = uwbPosition.x_m;
    localPosition.y = uwbPosition.y_m;

    previousMeasureX = uwbPosition.x_m;
    previousMeasureY = uwbPosition.y_m;
    previousUwbTimestampMs = uwbPosition.timestamp_ms;

    xyInitialized = true;

    localPosition.valid_flags |= LOCAL_POSITION_XY_VALID;
}

/* 读取uwb_position_queue，并计算速度 */
static void positionUpdateDepthMeasurement(const sensorData_t *sensor_data)
{
    float measureDtS;
    float newDepthM;
    float newRawVz;
    float uwbTagDepthM;

    if (!sensor_data->depth.valid || !sensor_data->depth.calibrated)
    {
        return;
    }

    if (sensor_data->depth.timestamp == previousDepthTimestampMs)
    {
        return;
    }

    newDepthM = sensor_data->depth.depth_filtered_m;

    if (!isfinite(newDepthM))
    {
        return;
    }

    if (zInitialized)
    {
        measureDtS = (sensor_data->depth.timestamp - previousDepthTimestampMs) * 0.001f;

        if (positionMeasureDtValid(measureDtS))
        {
            newRawVz = (newDepthM - previousMeasureZ) / measureDtS;

            if (isfinite(newRawVz))
            {
                rawVzHold = newRawVz;

                localPosition.valid_flags |= LOCAL_VELOCITY_Z_VALID;
            }
            else
            {
                positionResetZVelocity();
            }
        }
        else
        {
            positionResetZVelocity();
        }
    }
    else
    {
        positionResetZVelocity();
    }

    /*
     * NED坐标中z轴向下为正，因此深度值可以直接作为z。
     */
    localPosition.z = newDepthM;

    previousMeasureZ = newDepthM;
    previousDepthTimestampMs = sensor_data->depth.timestamp;

    zInitialized = true;

    localPosition.valid_flags |= LOCAL_POSITION_Z_VALID;

    /*
     * 向UWB二维解算提供标签的当前深度。
     * 该公式要求UWB基站Z轴同样向下为正，并与深度计共用水面零点。
     */
    uwbTagDepthM = newDepthM + UWB_TAG_DEPTH_OFFSET_M;

    uwbSetTagZ(uwbTagDepthM);
}

/* 对速度进行二阶低通滤波 */
static void positionUpdateVelocityFilter(void)
{
    if ((localPosition.valid_flags & LOCAL_VELOCITY_XY_VALID) != 0U)
    {
        localPosition.vx = lpf2pApply(&velocityXLpf, rawVxHold);
        localPosition.vy = lpf2pApply(&velocityYLpf, rawVyHold);
    }
    else
    {
        localPosition.vx = 0.0f;
        localPosition.vy = 0.0f;
    }

    if ((localPosition.valid_flags & LOCAL_VELOCITY_Z_VALID) != 0U)
    {
        localPosition.vz = lpf2pApply(&velocityZLpf, rawVzHold);
    }
    else
    {
        localPosition.vz = 0.0f;
    }
}


bool positionEstimatorInit(void)
{
	/* 防止localPositionQueue被反复创建 */
	if (localPositionQueue != NULL)
	{
		return true;
	}

    memset(&localPosition, 0, sizeof(localPosition));

    localPositionQueue = xQueueCreate(1U, sizeof(localPositionNed_t));
	
	/* 滤波器初始化 */
    lpf2pInit(&velocityXLpf, POSITION_ESTIMATOR_RATE_HZ, VELOCITY_XY_LPF_CUTOFF_HZ);
    lpf2pInit(&velocityYLpf, POSITION_ESTIMATOR_RATE_HZ, VELOCITY_XY_LPF_CUTOFF_HZ);
    lpf2pInit(&velocityZLpf, POSITION_ESTIMATOR_RATE_HZ, VELOCITY_Z_LPF_CUTOFF_HZ);

    lpf2pReset(&velocityXLpf, 0.0f);
    lpf2pReset(&velocityYLpf, 0.0f);
    lpf2pReset(&velocityZLpf, 0.0f);

    return localPositionQueue != NULL;
}


/* 核心函数，更新控制器位置信息，更新mavlink发送队列 */
void positionEstimate(const sensorData_t *sensor_data, state_t *state, float dt_s)
{
	uint32_t nowMs;
	
	if (sensor_data == NULL || state == NULL || localPositionQueue == NULL)
    {
        return;
    }

    if (!isfinite(dt_s) || dt_s <= 0.0f)
    {
        dt_s = POSITION_ESTIMATOR_DT_S;
    }

    nowMs = HAL_GetTick();
	
	
	positionCheckTimeout(nowMs);/* 检查函数有效性 */
    positionPredict(dt_s);
    positionUpdateUwbMeasurement();
    positionUpdateDepthMeasurement(sensor_data);
    positionUpdateVelocityFilter();
	
	localPosition.timestamp_ms = nowMs;
	/* 更新状态送给stateControl */
    state->position.timestamp = nowMs;
    state->position.x = localPosition.x;
    state->position.y = localPosition.y;
    state->position.z = localPosition.z;

    state->velocity.timestamp = nowMs;
    state->velocity.x = localPosition.vx;
    state->velocity.y = localPosition.vy;
    state->velocity.z = localPosition.vz;
	state->local_position_valid_flags = localPosition.valid_flags;
	
	/* 写入localPositionQueue，后续由jetsonLinkTask读取 */
	xQueueOverwrite(localPositionQueue, &localPosition);
}

bool positionReadLatest(localPositionNed_t *position)
{
    if (position == NULL || localPositionQueue == NULL)
    {
        return false;
    }

    return xQueuePeek(localPositionQueue, position, 0U) == pdTRUE;
}
