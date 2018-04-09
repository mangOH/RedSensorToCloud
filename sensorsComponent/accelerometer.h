#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "legato.h"

LE_SHARED void imu_Init(void);
LE_SHARED le_result_t mangOH_ReadAccelerometer(double *xAcc, double *yAcc, double *zAcc);
LE_SHARED le_result_t mangOH_ReadGyro(double *x, double *y, double *z);
LE_SHARED le_result_t mangOH_ReadImuTemp(double *temperature);

#endif // ACCELEROMETER_H
