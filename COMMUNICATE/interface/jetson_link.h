#ifndef __JETSON_LINK_H
#define __JETSON_LINK_H

/* ===== 前置依赖（宏展开需要这些类型和函数） ===== */
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32f4xx.h"
#include "main.h"
/* ===== 外部变量声明（宏展开需要知道这些符号） ===== */
extern UART_HandleTypeDef huart1;

#define MAVLINK_TX_BUF_SIZE  280
extern SemaphoreHandle_t mavlink_tx_done;
extern uint8_t           mavlink_tx_buf[MAVLINK_TX_BUF_SIZE];
extern volatile uint16_t mavlink_tx_len;

/* ===== MAVLink 发送宏（必须在 mavlink.h 之前定义） ===== */
#define MAVLINK_START_UART_SEND(chan, length)                    \
    do {                                                         \
        xSemaphoreTake(mavlink_tx_done, portMAX_DELAY);           \
        mavlink_tx_len = 0;                                      \
    } while (0)

#define MAVLINK_SEND_UART_BYTES(chan, buf, len)                  \
    do {                                                         \
        configASSERT((mavlink_tx_len + (len)) <=                 \
                     MAVLINK_TX_BUF_SIZE);                        \
        memcpy(&mavlink_tx_buf[mavlink_tx_len], (buf), (len));   \
        mavlink_tx_len += (len);                                 \
    } while (0)

#define MAVLINK_END_UART_SEND(chan, length)                      \
    do {                                                         \
        if (HAL_UART_Transmit_DMA(&huart1,                       \
                                  mavlink_tx_buf,                 \
                                  mavlink_tx_len) != HAL_OK) {    \
            xSemaphoreGive(mavlink_tx_done);                     \
        }                                                        \
    } while (0)

/* ===== MAVLink 库（展开时会用到上面的宏） ===== */
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "mavlink.h"

/* ===== 项目头文件 ===== */
#include "stabilizer_types.h"
#include "stabilizer.h"

#define PARAM_COUNT  12
		
/* 心跳包信息 */
#define UTTER_MAV_TYPE_SUBMARINE      12U
#define UTTER_MAV_AUTOPILOT_GENERIC   0U
#define UTTER_MAV_STATE_ACTIVE        4U
	
/* 位置消息发送间隔 */
#define LOCAL_POSITION_SEND_PERIOD_MS 20U
	
	
typedef struct {
		const char param_id[17];
		float      *value;
} param_entry_t;

typedef struct {
    float pitch;
    float yaw;
    float thrust;
    float heave;
} setpointCache_t;

void getSetpoint(setpoint_t *setp);
void jetsonLinkTask(void *param);

#endif
