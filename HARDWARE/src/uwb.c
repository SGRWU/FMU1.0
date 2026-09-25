#include "uwb.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* CubeMX生成的UART6句柄 */
extern UART_HandleTypeDef huart6;


/* ==================== 内部变量 ==================== */

static const uwbAnchor_t uwb_anchors[UWB_ANCHOR_NUM] =
{
    {UWB_A0_X_M, UWB_A0_Y_M, UWB_A0_Z_M},
    {UWB_A1_X_M, UWB_A1_Y_M, UWB_A1_Z_M},
    {UWB_A2_X_M, UWB_A2_Y_M, UWB_A2_Z_M},
    {UWB_A3_X_M, UWB_A3_Y_M, UWB_A3_Z_M}
};

/* UART6每次接收一个字节 */
static uint8_t uwb_rx_byte;

/* UART6中断向uwbTask传递字节 */
static QueueHandle_t uwb_rx_queue = NULL;

/* 保存最新测距数据 */
static QueueHandle_t uwb_range_queue = NULL;

/* 保存最新二维定位结果 */
static QueueHandle_t uwb_position_queue = NULL;

/* 当前标签Z坐标 */
static volatile float uwb_tag_z_m = UWB_DEFAULT_TAG_Z_M;
/*调试用！！！*/
#define UWB_DEBUG_RX_BUFFER_SIZE 128U

static volatile uint8_t uwb_debug_rx_buffer[UWB_DEBUG_RX_BUFFER_SIZE];
static volatile uint16_t uwb_debug_rx_count = 0U;

/* ==================== 内部函数 ==================== */

static bool uwbParseMcFrame(const char *line, uwbRangeData_t *data)
{
    unsigned int mask;
    unsigned int range_count;
    unsigned int sequence;
    unsigned int tag_id;
    unsigned int report_anchor_id;

    unsigned long range0;
    unsigned long range1;
    unsigned long range2;
    unsigned long range3;
    unsigned long debug_value;

    char node_type;

    int parsed;

    if ((line == NULL) || (data == NULL))
    {
        return false;
    }

    /*
     * 示例：
     * mc 0f 00000663 000005a3 00000512 000004cb
     *    095f c1 00024c24 t1:0
     */
    parsed = sscanf(line, "mc %x %lx %lx %lx %lx %x %x %lx %c%u:%u", &mask, &range0, &range1, &range2, &range3, &range_count, &sequence, &debug_value, &node_type, &tag_id, &report_anchor_id);

    /*
     * 转换参数数量：
     *
     * 1  mask
     * 2  range0
     * 3  range1
     * 4  range2
     * 5  range3
     * 6  range_count
     * 7  sequence
     * 8  debug_value
     * 9  node_type
     * 10 tag_id
     * 11 report_anchor_id
     */
    if (parsed != 11)
    {
        return false;
    }

    /* 当前只处理标签输出 */
    if (node_type != 't')
    {
        return false;
    }

    memset(data, 0, sizeof(*data));

    data->timestamp_ms = HAL_GetTick();

    data->mask = (uint8_t)(mask & UWB_VALID_MASK);

    data->range_mm[0] = (uint32_t)range0;
    data->range_mm[1] = (uint32_t)range1;
    data->range_mm[2] = (uint32_t)range2;
    data->range_mm[3] = (uint32_t)range3;

    data->range_count = (uint16_t)range_count;
    data->sequence = (uint8_t)sequence;

    data->tag_id = (uint8_t)tag_id;
    data->report_anchor_id = (uint8_t)report_anchor_id;

    /*
     * 固定四基站模式：
     * 1. MASK必须为0x0F；
     * 2. 四个距离都必须非零。
     */
    data->valid =
        (data->mask == UWB_VALID_MASK) &&
        (data->range_mm[0] != 0U) &&
        (data->range_mm[1] != 0U) &&
        (data->range_mm[2] != 0U) &&
        (data->range_mm[3] != 0U);

    return true;
}


static float uwbCalculateResidual(const uwbRangeData_t *ranges, float x_m, float y_m, float tag_z_m)
{
    float error_square_sum = 0.0f;
    uint8_t i;

    for (i = 0U; i < UWB_ANCHOR_NUM; i++)
    {
        float dx;
        float dy;
        float dz;
        float predicted_range_m;
        float measured_range_m;
        float error_m;

        dx = x_m - uwb_anchors[i].x_m;
        dy = y_m - uwb_anchors[i].y_m;
        dz = tag_z_m - uwb_anchors[i].z_m;

        predicted_range_m = sqrtf(dx * dx + dy * dy + dz * dz);
        measured_range_m = (float)ranges->range_mm[i] * 0.001f;

        error_m = predicted_range_m - measured_range_m;
        error_square_sum += error_m * error_m;
    }

    return sqrtf(error_square_sum / (float)UWB_ANCHOR_NUM);
}


static bool uwbSolvePosition2D(const uwbRangeData_t *ranges, float tag_z_m, uwbPosition_t *position)
{
    float range_m[UWB_ANCHOR_NUM];

    float ata00 = 0.0f;
    float ata01 = 0.0f;
    float ata11 = 0.0f;

    float atb0 = 0.0f;
    float atb1 = 0.0f;

    float determinant;

    uint8_t i;

    if ((ranges == NULL) || (position == NULL))
    {
        return false;
    }

    memset(position, 0, sizeof(*position));

    position->timestamp_ms = ranges->timestamp_ms;
    position->tag_z_m = tag_z_m;
    position->valid = false;

    /*
     * 必须四个基站全部有效。
     */
    if (!ranges->valid || (ranges->mask != UWB_VALID_MASK))
    {
        return false;
    }

    for (i = 0U; i < UWB_ANCHOR_NUM; i++)
    {
        float dz;

        range_m[i] = (float)ranges->range_mm[i] * 0.001f;
        dz = tag_z_m - uwb_anchors[i].z_m;

        /*
         * UWB测量的是空间距离。
         * 如果空间距离小于已知高度差，当前测量不合理。
         */
        if ((range_m[i] * range_m[i]) <= (dz * dz))
        {
            return false;
        }
    }

    /*
     * 以A0作为参考基站。
     *
     * 对A1、A2、A3分别构造：
     *
     * a0*x + a1*y = b
     *
     * 三条方程、两个未知量，使用最小二乘：
     *
     * position = inverse(A^T A) * A^T b
     */
    for (i = 1U; i < UWB_ANCHOR_NUM; i++)
    {
        float dx;
        float dy;

        float dz_i;
        float dz_0;

        float a0;
        float a1;
        float b;

        dx = uwb_anchors[i].x_m - uwb_anchors[0].x_m;
        dy = uwb_anchors[i].y_m - uwb_anchors[0].y_m;

        dz_i = tag_z_m - uwb_anchors[i].z_m;
        dz_0 = tag_z_m - uwb_anchors[0].z_m;

        a0 = 2.0f * dx;
        a1 = 2.0f * dy;

        b =
            range_m[0] * range_m[0] -
            range_m[i] * range_m[i] +
            uwb_anchors[i].x_m * uwb_anchors[i].x_m -
            uwb_anchors[0].x_m * uwb_anchors[0].x_m +
            uwb_anchors[i].y_m * uwb_anchors[i].y_m -
            uwb_anchors[0].y_m * uwb_anchors[0].y_m +
            dz_i * dz_i -
            dz_0 * dz_0;

        ata00 += a0 * a0;
        ata01 += a0 * a1;
        ata11 += a1 * a1;

        atb0 += a0 * b;
        atb1 += a1 * b;
    }

    determinant = ata00 * ata11 - ata01 * ata01;

    /*
     * 行列式接近0表示基站几何关系无法解算，
     * 例如四个基站近似位于一条直线上。
     */
    if (fabsf(determinant) < 1.0e-6f)
    {
        return false;
    }

    position->x_m = (atb0 * ata11 - atb1 * ata01) / determinant;
    position->y_m = (ata00 * atb1 - ata01 * atb0) / determinant;

    /*
     * 用解算出的X/Y和已知Z重新计算四个三维距离，
     * 并与UWB测量距离比较。
     */
    position->residual_m = uwbCalculateResidual(ranges, position->x_m, position->y_m, tag_z_m);

    if (position->residual_m > UWB_MAX_RESIDUAL_M)
    {
        return false;
    }

    position->valid = true;

    return true;
}


/* ==================== 对外函数 ==================== */

bool uwbInit(void)
{
    if (uwb_rx_queue == NULL)
    {
        uwb_rx_queue = xQueueCreate(UWB_RX_QUEUE_LENGTH, sizeof(uint8_t));
    }

    if (uwb_range_queue == NULL)
    {
        uwb_range_queue = xQueueCreate(1U, sizeof(uwbRangeData_t));
    }

    if (uwb_position_queue == NULL)
    {
        uwb_position_queue = xQueueCreate(1U, sizeof(uwbPosition_t));
    }

    if ((uwb_rx_queue == NULL) || (uwb_range_queue == NULL) || (uwb_position_queue == NULL))
    {
        return false;
    }

    /*
     * 开启UART6第一次单字节接收。
     * 后续接收在HAL_UART_RxCpltCallback中重新启动。
     */
    if (HAL_UART_Receive_IT(&huart6, &uwb_rx_byte, 1U) != HAL_OK)
    {
        return false;
    }

    return true;
}


void uwbSetTagZ(float tag_z_m)
{
    uwb_tag_z_m = tag_z_m;
}


bool uwbGetPosition(uwbPosition_t *position)
{
    if ((position == NULL) || (uwb_position_queue == NULL))
    {
        return false;
    }

    if (xQueuePeek(uwb_position_queue, position, 0U) != pdTRUE)
    {
        return false;
    }

    if (!position->valid)
    {
        return false;
    }

    /*
     * 超时保护：
     * 超过规定时间没有新位置，则返回无效。
     */
    if ((HAL_GetTick() - position->timestamp_ms) > UWB_DATA_TIMEOUT_MS)
    {
        return false;
    }

    return true;
}


void uwbTask(void *param)
{
    uint8_t byte;

    char line[UWB_LINE_MAX_LENGTH];

    uint16_t line_length = 0U;

    bool discard_current_line = false;

    uwbRangeData_t range_data;
    uwbPosition_t position;

    (void)param;

    if (!uwbInit())
    {
        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        /*
         * 没有UART6数据时阻塞，不占用CPU。
         * UWB字节进入队列后立即恢复运行。
         */
        if (xQueueReceive(uwb_rx_queue, &byte, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

		if (byte == '\r' || byte == '\n')
		{
			if (line_length > 0U && !discard_current_line)
			{
				line[line_length] = '\0';

				if (uwbParseMcFrame(line, &range_data))
				{
					xQueueOverwrite(uwb_range_queue, &range_data);

					if (uwbSolvePosition2D(&range_data, uwb_tag_z_m, &position))
					{
						xQueueOverwrite(uwb_position_queue, &position);
					}
					else
					{
						memset(&position, 0, sizeof(position));
						position.timestamp_ms = range_data.timestamp_ms;
						position.tag_z_m = uwb_tag_z_m;
						position.valid = false;
						xQueueOverwrite(uwb_position_queue, &position);
					}
				}
			}

			line_length = 0U;
			discard_current_line = false;
		}
		else if (!discard_current_line)
		{
			if (line_length < (UWB_LINE_MAX_LENGTH - 1U))
			{
				line[line_length] = (char)byte;
				line_length++;
			}
			else
			{
				line_length = 0U;
				discard_current_line = true;
			}
		}
    }
}

/*
 * UART出现溢出、噪声或帧错误后重新启动接收。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART6))
    {
        HAL_UART_Receive_IT(&huart6, &uwb_rx_byte, 1U);
    }
}



//void uwbRxCpltFromISR(void)
//{
//    BaseType_t higher_priority_task_woken = pdFALSE;

//    if (uwb_rx_queue != NULL)
//    {
//        xQueueSendFromISR(uwb_rx_queue, &uwb_rx_byte, &higher_priority_task_woken);
//	}
//    HAL_UART_Receive_IT(&huart6, &uwb_rx_byte, 1U);
//    portYIELD_FROM_ISR(higher_priority_task_woken);
//}


void uwbRxCpltFromISR(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (uwb_debug_rx_count < UWB_DEBUG_RX_BUFFER_SIZE)
    {
        uwb_debug_rx_buffer[uwb_debug_rx_count] = uwb_rx_byte;
        uwb_debug_rx_count++;
    }

    if (uwb_rx_queue != NULL)
    {
        xQueueSendFromISR(uwb_rx_queue, &uwb_rx_byte, &higher_priority_task_woken);
    }

    HAL_UART_Receive_IT(&huart6, &uwb_rx_byte, 1U);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}