#include <string.h>
#include <math.h>
#include "jetson_link.h"
#include "attitude_pid.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h" 

mavlink_system_t mavlink_system = {1, 200};

static uint8_t rx_buf;
static mavlink_message_t msg;
static mavlink_status_t status;


SemaphoreHandle_t mavlink_tx_done;
uint8_t           mavlink_tx_buf[MAVLINK_TX_BUF_SIZE];
volatile uint16_t mavlink_tx_len;

static volatile uint32_t parsed_frames = 0;

static xQueueHandle param_rx_queue;
static int16_t param_send_index = -1;   // -1 = 不在发送中

static volatile setpointCache_t sp_cache[2];    /*used for double buffering*/
static volatile bool sp_active = false;

static const param_entry_t param_table[PARAM_COUNT] = {
    {"RATE_PITCH_P",   &pidRatePitch.kp},
    {"RATE_PITCH_I",   &pidRatePitch.ki},
    {"RATE_PITCH_D",   &pidRatePitch.kd},
    {"RATE_YAW_P",     &pidRateYaw.kp},
    {"RATE_YAW_I",     &pidRateYaw.ki},
    {"RATE_YAW_D",     &pidRateYaw.kd},
    {"PITCH_P",        &pidPitch.kp},
    {"PITCH_I",        &pidPitch.ki},
    {"PITCH_D",        &pidPitch.kd},
    {"YAW_P",          &pidYaw.kp},
    {"YAW_I",          &pidYaw.ki},
    {"YAW_D",          &pidYaw.kd},
};
/**/
static int16_t param_find_by_id(const char *id)
{
    for (int16_t i = 0; i < PARAM_COUNT; i++)
    {
        if (strncmp(param_table[i].param_id, id, 16) == 0)
            return i;
    }
    return -1;
}

static void param_send_value(uint16_t index)
{
    mavlink_msg_param_value_send(
        MAVLINK_COMM_0,
        (const char *)param_table[index].param_id,
        *param_table[index].value,
        MAV_PARAM_TYPE_REAL32,
        PARAM_COUNT,
        index);
}

static void param_send_all(void)
{
    for (uint16_t i = 0; i < PARAM_COUNT; i++)
    {
        param_send_value(i);

    }
}



void jetsonLinkTask(void *param)
{
    mavlink_tx_done = xSemaphoreCreateBinary();/*created to synchronize usart1 transmission*/
    xSemaphoreGive(mavlink_tx_done);

    angleControlInit(0.004f, 0.004f);
	
		param_rx_queue = xQueueCreate(5, sizeof(mavlink_message_t));/*queue recieve the mavlink message from jetson board*/
	
    HAL_UART_Receive_IT(&huart1, &rx_buf, 1);

    uint32_t lastAtt = xTaskGetTickCount();/*used to control send frequence of attitude*/
    uint32_t lastHb  = xTaskGetTickCount();/*used to control send freqience of heartbeat*/
	
    while (1)
    {
				
        uint32_t now = xTaskGetTickCount();

        /* HEARTBEAT: 1Hz */
        if (now - lastHb >= 1000)
        {
            lastHb = now;
						mavlink_msg_heartbeat_send(MAVLINK_COMM_0,
								0,    // MAV_TYPE_GENERIC
								0,    // MAV_AUTOPILOT_GENERIC
								0,    // base_mode
								0,    // custom_mode
								4);   // MAV_STATE_ACTIVE
        }

        /* ATTITUDE: 50Hz */
        if (now - lastAtt >= 20)
        {
            lastAtt = now;
            attitude_t att;
            Axis3f gyr;
            getAttitudeData(&att);
            getAngleRateData(&gyr);

            mavlink_msg_attitude_send(MAVLINK_COMM_0,
                HAL_GetTick() * 1000,
                att.roll  * DEG2RAD,
                att.pitch * DEG2RAD,
                att.yaw   * DEG2RAD,
                gyr.x     * DEG2RAD,
                gyr.y     * DEG2RAD,
                gyr.z     * DEG2RAD);
        }

				/* 处理参数请求队列（任务上下文，可安全调用 xSemaphoreTake） */
				
				mavlink_message_t qmsg;
				
				while (xQueueReceive(param_rx_queue, &qmsg, 0) == pdTRUE)
				{
						switch (qmsg.msgid)
						{
						case MAVLINK_MSG_ID_PARAM_REQUEST_LIST://21
							  param_send_index = 0;
								//param_send_all();
								break;

						case MAVLINK_MSG_ID_PARAM_REQUEST_READ://20
						{
								mavlink_param_request_read_t req;
								mavlink_msg_param_request_read_decode(&qmsg, &req);
								int16_t idx = -1;
								if (req.param_index >= 0 && req.param_index < PARAM_COUNT)
										idx = req.param_index;
								else if (req.param_id[0] != '\0')
										idx = param_find_by_id((const char *)req.param_id);
								if (idx >= 0)
										param_send_value(idx);
								break;
						}

						case MAVLINK_MSG_ID_PARAM_SET://23
						{
								mavlink_param_set_t p;
								mavlink_msg_param_set_decode(&qmsg, &p);
								int16_t idx = param_find_by_id((const char *)p.param_id);
								if (idx >= 0)
								{
										*param_table[idx].value = p.param_value;
										param_send_value(idx);
								}
								break;
						}
						
						case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE://70
						{
								mavlink_rc_channels_override_t rc;
								mavlink_msg_rc_channels_override_decode(&qmsg, &rc);

								volatile setpointCache_t *back = &sp_cache[!sp_active];
								back->yaw  = (rc.chan1_raw - 1500.0f) * 180.0f / 500.0f;   // ±500 → ±500
								back->pitch    = (rc.chan2_raw - 1500.0f) * (30.0f / 500.0f);
								back->heave  = (rc.chan3_raw - 1500.0f) * 500.0f / 500.0f;
								back->thrust = (rc.chan4_raw - 1500.0f) * 500.0f / 500.0f;     // 1000→0.0, 1500→0.5, 2000→1.0
								sp_active = !sp_active;
								break;
						}
						default:
								break;
						}
				}
				
				if (param_send_index >= 0 && param_send_index < PARAM_COUNT)
				{
						param_send_value(param_send_index);
						param_send_index++;
				}
				else
				{
						param_send_index = -1;
				}
				
        vTaskDelay(1);
				
    }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (mavlink_parse_char(MAVLINK_COMM_0, rx_buf, &msg, &status))
        {
            parsed_frames++;

            /* 把参数相关的消息推进队列，不在 ISR 里处理 */
						if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST ||
								msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ ||
								msg.msgid == MAVLINK_MSG_ID_PARAM_SET ||
								msg.msgid == MAVLINK_MSG_ID_SET_ATTITUDE_TARGET||
								msg.msgid == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE)
						{
								xQueueSendFromISR(param_rx_queue, &msg, NULL);
						}
        }
        HAL_UART_Receive_IT(&huart1, &rx_buf, 1);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(mavlink_tx_done, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void getSetpoint(setpoint_t *setp)
{
    setp->attitude.pitch = sp_cache[sp_active].pitch;
    setp->attitude.yaw   = sp_cache[sp_active].yaw;
    setp->thrust = sp_cache[sp_active].thrust;
    setp->heave  = sp_cache[sp_active].heave;
}
