#include "depth_sensor.h"
#include "ms5837.h"
#include "iic_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <string.h>
#include <stdio.h>
#define DEPTH_TASK_PERIOD_MS             50U
#define DEPTH_FILTER_ALPHA               0.20f

#define DEPTH_ZERO_DISCARD_SAMPLES       10U
#define DEPTH_ZERO_AVERAGE_SAMPLES       40U

#define DEPTH_MAX_CONSECUTIVE_ERRORS     3U

static QueueHandle_t depthDataQueue = NULL;

static ms5837_t depthSensor;
static depthData_t depthData;

static void depth_delay_ms(uint32_t delay_ms)
{
    TickType_t ticks;

    ticks = pdMS_TO_TICKS(delay_ms);

    if (ticks == 0U)
    {
        ticks = 1U;
    }

    vTaskDelay(ticks);
}

static int depth_sensor_device_init(void)
{
    ms5837_bus_t bus;

    bus.write = i2c_write;
    bus.read = i2c_read;
    bus.delay_ms = depth_delay_ms;

    if (i2c_is_ready(
            MS5837_I2C_ADDRESS,
            3U,
            100U) != 0)
    {
        return -1;
    }

    if (ms5837_init(
            &depthSensor,
            &bus,
            MS5837_I2C_ADDRESS) != 0)
    {
        return -2;
    }

    /*
     * 淡水密度。
     * 海水可根据实际盐度改成约1025kg/m3。
     */
    if (ms5837_set_fluid_density(
            &depthSensor,
            997.0f) != 0)
    {
        return -3;
    }

    return 0;
}

/*
 * 此函数假设ROV启动时，传感器处于水面零深度位置。
 * 如果ROV可能在水下启动，不应自动执行，应改成收到校零命令后执行。
 */
static int depth_sensor_zero_calibrate(void)
{
    uint16_t count;
    float pressure_sum = 0.0f;

    for (count = 0U;
         count < DEPTH_ZERO_DISCARD_SAMPLES;
         count++)
    {
        if (ms5837_read(&depthSensor) != 0)
        {
            return -1;
        }

        vTaskDelay(pdMS_TO_TICKS(
            DEPTH_TASK_PERIOD_MS));
    }

    for (count = 0U;
         count < DEPTH_ZERO_AVERAGE_SAMPLES;
         count++)
    {
        if (ms5837_read(&depthSensor) != 0)
        {
            return -2;
        }

        pressure_sum +=
            ms5837_get_pressure(&depthSensor);

        vTaskDelay(pdMS_TO_TICKS(
            DEPTH_TASK_PERIOD_MS));
    }

    ms5837_set_base_pressure(
        &depthSensor,
        pressure_sum /
        (float)DEPTH_ZERO_AVERAGE_SAMPLES);

    return 0;
}

bool depthReadLatest(depthData_t *data)
{
    if ((data == NULL) || (depthDataQueue == NULL))
    {
        return false;
    }

    return
        xQueueReceive(
            depthDataQueue,
            data,
            0U) == pdTRUE;
}

void depthTask(void *param)
{
    TickType_t lastWakeTime;
    float depth_raw;
    uint16_t consecutive_errors = 0U;
    int result;

    (void)param;

    memset(&depthData, 0, sizeof(depthData));

    depthDataQueue = xQueueCreate(1U, sizeof(depthData_t));
        
    if (depthDataQueue == NULL)
    {
        vTaskDelete(NULL);
        return;
    }
    while (depth_sensor_device_init() != 0)
    {
        depthData.valid = false;
        depthData.calibrated = false;
        depthData.error_count++;

        xQueueOverwrite(
            depthDataQueue,
            &depthData);

        vTaskDelay(pdMS_TO_TICKS(1000U));
    }

    /*
     * 自动零点校准要求启动时位于水面。
     */
    if (depth_sensor_zero_calibrate() == 0)
    {
        depthData.calibrated = true;
    }
    else
    {
        depthData.calibrated = false;
    }

    lastWakeTime = xTaskGetTickCount();

    while (1)
    {
        result = ms5837_read(&depthSensor);

        if (result == 0)
        {
            result = ms5837_calculate_depth(
                &depthSensor,
                &depth_raw);
        }

        if (result == 0)
        {
            depthData.pressure_mbar =
                ms5837_get_pressure(&depthSensor);

            depthData.temperature_c =
                ms5837_get_temperature(&depthSensor);

            depthData.depth_raw_m = depth_raw;

            if (!depthData.valid)
            {
                /*
                 * 第一个有效样本直接初始化滤波器，
                 * 避免从0缓慢收敛。
                 */
                depthData.depth_filtered_m =
                    depth_raw;
            }
            else
            {
                depthData.depth_filtered_m +=
                    DEPTH_FILTER_ALPHA *
                    (depth_raw -
                     depthData.depth_filtered_m);
            }

            depthData.timestamp =
                (uint32_t)xTaskGetTickCount();

            depthData.valid =
                depthData.calibrated;

            consecutive_errors = 0U;
        }
        else
        {
            if (consecutive_errors < 0xFFFFU)
            {
                consecutive_errors++;
            }

            if (depthData.error_count < 0xFFFFU)
            {
                depthData.error_count++;
            }

            if (consecutive_errors >=
                DEPTH_MAX_CONSECUTIVE_ERRORS)
            {
                depthData.valid = false;
            }
        }
		//printf("%f\r\n", depthData.depth_raw_m);
        xQueueOverwrite(depthDataQueue, &depthData);

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(DEPTH_TASK_PERIOD_MS));
    }
}