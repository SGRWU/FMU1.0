#ifndef __SYSTEM_H
#define __SYSTEM_H


#include "icm-42688.h"
#include "spi_driver.h"
#include "ms5837.h"
#include "iic_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "imu_sensor.h"
#include "depth_sensor.h"
#include "uwb.h"

#include <stdio.h>
#include "stabilizer.h"
#include "jetson_link.h"
#include "thrusters.h"

void systemInit(void);
#endif
