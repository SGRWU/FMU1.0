#ifndef DEPTH_SENSOR_H
#define DEPTH_SENSOR_H

#include <stdbool.h>
#include "stabilizer_types.h"

void depthTask(void *param);

bool depthReadLatest(depthData_t *data);


#endif