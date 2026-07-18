#ifndef __SYSTEM_H
#define __SYSTEM_H


#include "icm-42688.h"
#include "spi_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sensor.h"

#include <stdio.h>
#include "stabilizer.h"
#include "jetson_link.h"
#include "thrusters.h"

void systemInit(void);
#endif
