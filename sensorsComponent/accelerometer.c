//--------------------------------------------------------------------------------------------------
/**
 * Implementation of the Inertial Measurement Unit (IMU) sensor interface.
 *
 * Provides the accelerometer and gyro IPC API services and plugs into the Data Hub.
 *
 * Copyright (C) Sierra Wireless Inc.
 */
//--------------------------------------------------------------------------------------------------

#include "legato.h"
#include "interfaces.h"

#include "accelerometer.h"
#include "sensorUtils.h"
#include "periodicSensor.h"

static const char FormatStr[] = "/sys/devices/i2c-0/0-0068/iio:device0/in_%s_%s";
static const char AccType[]   = "accel";
static const char GyroType[]  = "anglvel";
static const char TempType[]  = "temp";
static const char CompX[]     = "x_raw";
static const char CompY[]     = "y_raw";
static const char CompZ[]     = "z_raw";
static const char CompScale[] = "scale";

static psensor_Ref_t GyroPSensorRef;
static psensor_Ref_t AccelPSensorRef;

//--------------------------------------------------------------------------------------------------
/**
 * Sample timer expiry function for the gyroscope.
 */
//--------------------------------------------------------------------------------------------------
static void GyroSample
(
    psensor_Ref_t ref
)
{
    double x;
    double y;
    double z;

    le_result_t result = mangOH_ReadGyro(&x, &y, &z);

    if (result == LE_OK)
    {
        char sample[256];

        int len = snprintf(sample, sizeof(sample), "{ \"x\": %lf, \"y\": %lf, \"z\": %lf }", x, y, z);
        if (len >= sizeof(sample))
        {
            LE_FATAL("JSON string (len %d) is longer than buffer (size %zu).", len, sizeof(sample));
        }

        psensor_PushJson(ref, 0 /* now */, sample);
    }
    else
    {
        LE_ERROR("Failed to read gyro (%s).", LE_RESULT_TXT(result));
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Sample timer expiry function for the accelerometer.
 */
//--------------------------------------------------------------------------------------------------
static void AccelSample
(
    psensor_Ref_t ref
)
{
    double x;
    double y;
    double z;

    le_result_t result = mangOH_ReadAccelerometer(&x, &y, &z);

    if (result == LE_OK)
    {
        char sample[256];

        int len = snprintf(sample, sizeof(sample), "{ \"x\": %lf, \"y\": %lf, \"z\": %lf }", x, y, z);
        if (len >= sizeof(sample))
        {
            LE_FATAL("JSON string (len %d) is longer than buffer (size %zu).", len, sizeof(sample));
        }

        psensor_PushJson(ref, 0 /* now */, sample);
    }
    else
    {
        LE_ERROR("Failed to read accelerometer (%s).", LE_RESULT_TXT(result));
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Sample timer expiry function for the temperature.
 */
//--------------------------------------------------------------------------------------------------
static void TempSample
(
    psensor_Ref_t ref
)
{
    double sample;

    le_result_t result = mangOH_ReadImuTemp(&sample);

    if (result == LE_OK)
    {
        psensor_PushNumeric(ref, 0 /* now */, sample);
    }
    else
    {
        LE_ERROR("Failed to read IMU temperature (%s).", LE_RESULT_TXT(result));
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Initializes the IMU module.
 */
//--------------------------------------------------------------------------------------------------
void imu_Init
(
    void
)
{
    GyroPSensorRef = psensor_Create("gyro", DHUBIO_DATA_TYPE_JSON, "", GyroSample);
    AccelPSensorRef = psensor_Create("accel", DHUBIO_DATA_TYPE_JSON, "", AccelSample);
    AccelPSensorRef = psensor_Create("imu/temp", DHUBIO_DATA_TYPE_NUMERIC, "", TempSample);
}


//--------------------------------------------------------------------------------------------------
/**
 * Reports the x, y and z accelerometer readings in meters per second squared.
 */
//--------------------------------------------------------------------------------------------------
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


//--------------------------------------------------------------------------------------------------
/**
 * Reports the x, y and z gyro readings in radians per second.
 */
//--------------------------------------------------------------------------------------------------
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


//--------------------------------------------------------------------------------------------------
/**
 * Reports the temperature of the Inertial Measurement Unit (IMU) in degrees Celcius.
 *
 * @return LE_OK if successful, something else if failed.
 */
//--------------------------------------------------------------------------------------------------
le_result_t mangOH_ReadImuTemp
(
    double *temperature
)
{
    le_result_t r;
    char path[256];

    double scaling = 0.0;
    int pathLen = snprintf(path, sizeof(path), FormatStr, TempType, CompScale);
    LE_ASSERT(pathLen < sizeof(path));
    r = ReadDoubleFromFile(path, &scaling);
    if (r != LE_OK)
    {
        LE_ERROR("Failed to read scale");
        goto done;
    }

    double offset = 0.0;
    pathLen = snprintf(path, sizeof(path), FormatStr, TempType, "offset");
    LE_ASSERT(pathLen < sizeof(path));
    r = ReadDoubleFromFile(path, &offset);
    if (r != LE_OK)
    {
        LE_ERROR("Failed to read offset");
        goto done;
    }

    pathLen = snprintf(path, sizeof(path), FormatStr, TempType, "raw");
    LE_ASSERT(pathLen < sizeof(path));
    r = ReadDoubleFromFile(path, temperature);
    if (r != LE_OK)
    {
        LE_ERROR("Failed to read raw value");
        goto done;
    }

    *temperature = (*temperature + offset) * scaling / 1000;

done:
    return r;
}
