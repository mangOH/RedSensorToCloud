#include "legato.h"
#include "interfaces.h"

#include "accelerometer.h"
#include "sensorUtils.h"

static const char FormatStr[] = "/sys/devices/i2c-0/0-0060/iio:device0/in_%s%s_input";
static const char AccType[] = "acc";
static const char Gyroype[] = "gyro";
static const char CompX[] = "x";
static const char CompY[] = "y";
static const char CompZ[] = "z";

le_result_t mangOH_ReadAccelerometer
(
    double *xAcc,
    double *yAcc,
    double *zAcc
)
{
    /* TODO: Enable once the file paths are correct
    le_result_t r;
    char path[256];

    int pathLen = snprintf(path, sizeof(path), FormatStr, AccType, CompX);
    LE_ASSERT(pathLen < sizeof(path));
    r = ReadDoubleFromFile(path, xAcc);
    if (r != LE_OK)
    {
        goto done;
    }

    pathLen = snprintf(path, sizeof(path), FormatStr, AccType, CompY);
    LE_ASSERT(pathLen < sizeof(path));
    r = ReadDoubleFromFile(path, yAcc);
    if (r != LE_OK)
    {
        goto done;
    }

    pathLen = snprintf(path, sizeof(path), FormatStr, AccType, CompZ);
    LE_ASSERT(pathLen < sizeof(path));
    r = ReadDoubleFromFile(path, zAcc);

done:
    return r;
    */


    *xAcc = 0.0;
    *yAcc = 0.0;
    *zAcc = 0.0;
    return LE_OK;
}

le_result_t mangOH_ReadGyro
(
    double *x,
    double *y,
    double *z
)
{
    /* TODO: Enable once the file paths are correct
    le_result_t r;
    char path[256];

    int pathLen = snprintf(path, sizeof(path), FormatStr, GyroType, CompX);
    LE_ASSERT(pathLen < sizeof(path));
    r = ReadDoubleFromFile(path, x);
    if (r != LE_OK)
    {
        goto done;
    }

    pathLen = snprintf(path, sizeof(path), FormatStr, GyroType, CompY);
    LE_ASSERT(pathLen < sizeof(path));
    r = ReadDoubleFromFile(path, y);
    if (r != LE_OK)
    {
        goto done;
    }

    pathLen = snprintf(path, sizeof(path), FormatStr, GyroType, CompZ);
    LE_ASSERT(pathLen < sizeof(path));
    r = ReadDoubleFromFile(path, z);

done:
    return r;
    */


    *x = 0.0;
    *y = 0.0;
    *z = 0.0;
    return LE_OK;
}
