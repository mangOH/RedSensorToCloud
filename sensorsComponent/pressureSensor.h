#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include "legato.h"

LE_SHARED le_result_t mangOH_ReadPressureSensor(double *reading);
LE_SHARED le_result_t mangOH_ReadTemperatureSensor(double *reading);

#endif // PRESSURE_SENSOR_H
