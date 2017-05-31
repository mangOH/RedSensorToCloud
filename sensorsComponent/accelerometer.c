#include "legato.h"
#include "interfaces.h"

#include "accelerometer.h"
#include "sensorUtils.h"

static const char FormatStr[] = "/sys/devices/i2c-0/0-0068/iio:device0/in_%s_%s";
static const char AccType[]   = "accel";
static const char GyroType[]  = "anglvel";
static const char CompX[]     = "x_raw";
static const char CompY[]     = "y_raw";
static const char CompZ[]     = "z_raw";
static const char CompScale[] = "scale";

/**
 * Reports the x, y and z accelerometer readings in meters per second squared.
 */
le_result_t mangOH_ReadAccelerometer
(
    double *xAcc,
    double *yAcc,
    double *zAcc
)
{
    le_result_t r;
    char path[256];

    double scaling = 0.0;
    int pathLen = snprintf(path, sizeof(path), FormatStr, AccType, CompScale);
    LE_ASSERT(pathLen < sizeof(path));
    r = ReadDoubleFromFile(path, &scaling);
    if (r != LE_OK)
    {
        goto done;
    }

    pathLen = snprintf(path, sizeof(path), FormatStr, AccType, CompX);
    LE_ASSERT(pathLen < sizeof(path));
    r = ReadDoubleFromFile(path, xAcc);
    if (r != LE_OK)
    {
        goto done;
    }
    *xAcc *= scaling;

    pathLen = snprintf(path, sizeof(path), FormatStr, AccType, CompY);
    LE_ASSERT(pathLen < sizeof(path));
    r = ReadDoubleFromFile(path, yAcc);
    if (r != LE_OK)
    {
        goto done;
    }
    *yAcc *= scaling;

    pathLen = snprintf(path, sizeof(path), FormatStr, AccType, CompZ);
    LE_ASSERT(pathLen < sizeof(path));
    r = ReadDoubleFromFile(path, zAcc);
    *zAcc *= scaling;

done:
    return r;
}

/**
 * Reports the x, y and z gyro readings in radians per second.
 */
le_result_t mangOH_ReadGyro
(
    double *x,
    double *y,
    double *z
)
{
    le_result_t r;
    char path[256];

    double scaling = 0.0;
    int pathLen = snprintf(path, sizeof(path), FormatStr, GyroType, CompScale);
    LE_ASSERT(pathLen < sizeof(path));
    r = ReadDoubleFromFile(path, &scaling);
    if (r != LE_OK)
    {
        goto done;
    }

    pathLen = snprintf(path, sizeof(path), FormatStr, GyroType, CompX);
    LE_ASSERT(pathLen < sizeof(path));
    r = ReadDoubleFromFile(path, x);
    if (r != LE_OK)
    {
        goto done;
    }
    *x *= scaling;

    pathLen = snprintf(path, sizeof(path), FormatStr, GyroType, CompY);
    LE_ASSERT(pathLen < sizeof(path));
    r = ReadDoubleFromFile(path, y);
    if (r != LE_OK)
    {
        goto done;
    }
    *y *= scaling;

    pathLen = snprintf(path, sizeof(path), FormatStr, GyroType, CompZ);
    LE_ASSERT(pathLen < sizeof(path));
    r = ReadDoubleFromFile(path, z);
    *z *= scaling;

done:
    return r;
}
