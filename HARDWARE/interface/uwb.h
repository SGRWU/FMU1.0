#ifndef __UWB_H
#define __UWB_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"


/* ==================== UWB基本配置 ==================== */

/* 固定使用四个基站 */
#define UWB_ANCHOR_NUM              4U

/* 四个基站全部有效时，MASK应为0x0F */
#define UWB_VALID_MASK              0x0FU

/* UART6字节接收队列长度 */
#define UWB_RX_QUEUE_LENGTH         128U

/* 单行mc数据的最大长度 */
#define UWB_LINE_MAX_LENGTH         128U

/* 定位数据超时时间 */
#define UWB_DATA_TIMEOUT_MS         300U

/* 允许的最大测距均方根残差 */
#define UWB_MAX_RESIDUAL_M          1.0f


/* ==================== 基站坐标配置 ==================== */

/*
 * 单位：米。
 *
 * 必须替换为实际测量的基站坐标。
 * 四个基站必须使用同一个坐标系。
 */

/* Anchor 0 */
#define UWB_A0_X_M                  0.0f
#define UWB_A0_Y_M                  0.0f
#define UWB_A0_Z_M                  2.0f

/* Anchor 1 */
#define UWB_A1_X_M                  6.0f
#define UWB_A1_Y_M                  8.0f
#define UWB_A1_Z_M                  2.0f

/* Anchor 2 */
#define UWB_A2_X_M                  0.0f
#define UWB_A2_Y_M                  8.0f
#define UWB_A2_Z_M                  2.0f

/* Anchor 3 */
#define UWB_A3_X_M                  6.0f
#define UWB_A3_Y_M                  0.0f
#define UWB_A3_Z_M                  2.0f


/*
 * 第一阶段使用的标签固定Z坐标。
 *
 * 后续接入深度计后，可以通过uwbSetTagZ()动态更新。
 */
#define UWB_DEFAULT_TAG_Z_M         1.0f


/* ==================== 数据结构 ==================== */

/*
 * 单个UWB基站的位置。
 */
typedef struct
{
    float x_m;
    float y_m;
    float z_m;
} uwbAnchor_t;


/*
 * Mini5的mc帧解析结果。
 *
 * UWB原始距离单位为毫米，因此使用uint32_t保存，
 * 避免协议解析阶段产生不必要的浮点转换。
 */
typedef struct
{
    uint32_t timestamp_ms;

    uint32_t range_mm[UWB_ANCHOR_NUM];

    uint8_t mask;
    uint8_t sequence;

    uint16_t range_count;

    uint8_t tag_id;
    uint8_t report_anchor_id;

    bool valid;
} uwbRangeData_t;


/*
 * 二维位置解算结果。
 *
 * x_m、y_m是UWB解算结果。
 * tag_z_m是外部提供给二维解算的已知标签高度，
 * 不是UWB解算出的Z坐标。
 */
typedef struct
{
    uint32_t timestamp_ms;

    float x_m;
    float y_m;
    float tag_z_m;

    float residual_m;

    bool valid;
} uwbPosition_t;


/* ==================== 对外函数 ==================== */

/*
 * 创建接收队列并启动UART6接收。
 */
bool uwbInit(void);


/*
 * UWB数据处理任务。
 *
 * 负责：
 * 1. 接收UART6字节；
 * 2. 拼接完整mc数据帧；
 * 3. 解析四路距离；
 * 4. 解算二维位置；
 * 5. 通过USART1打印。
 */
void uwbTask(void *param);


/*
 * 设置二维位置解算使用的标签Z坐标。
 *
 * 单位：米。
 */
void uwbSetTagZ(float tag_z_m);


/*
 * 非消费式读取最新二维位置。
 *
 * 返回true：
 * 位置有效并且没有超时。
 *
 * 返回false：
 * 没有定位结果、解算失败或数据超时。
 */
bool uwbGetPosition(uwbPosition_t *position);

void uwbRxCpltFromISR(void);

#endif