#include "legato.h"
#include "interfaces.h"
#include "periodicSensor.h"
#include "pressureSensor.h"
#include "sensorUtils.h"

static const char PressureFile[] = "/sys/devices/i2c-0/0-0076/iio:device1/in_pressure_input";
static const char TemperatureFile[] = "/sys/devices/i2c-0/0-0076/iio:device1/in_temp_input";

static void SamplePressure
(
    psensor_Ref_t ref
)
{
    double sample;

    le_result_t result = mangOH_ReadPressureSensor(&sample);

    if (result == LE_OK)
    {
        psensor_PushNumeric(ref, 0 /* now */, sample);
    }
    else
    {
        LE_ERROR("Failed to read sensor (%s).", LE_RESULT_TXT(result));
    }
}


static void SampleTemperature
(
    psensor_Ref_t ref
)
{
    double sample;

    le_result_t result = mangOH_ReadTemperatureSensor(&sample);

    if (result == LE_OK)
    {
        psensor_PushNumeric(ref, 0 /* now */, sample);
    }
    else
    {
        LE_ERROR("Failed to read sensor (%s).", LE_RESULT_TXT(result));
    }
}


void pressure_Init
(
    void
)
{
    psensor_Create("pressure", DHUBIO_DATA_TYPE_NUMERIC, "kPa", SamplePressure);
    psensor_Create("pressure/temp", DHUBIO_DATA_TYPE_NUMERIC, "degC", SampleTemperature);
}


/**
 * Reports the pressure kPa.
 */
le_result_t mangOH_ReadPressureSensor
(
    double *reading
)
{
    return ReadDoubleFromFile(PressureFile, reading);
}

/**
 * Reports the temperature in degrees celcius.
 */
le_result_t mangOH_ReadTemperatureSensor
(
    double *reading
)
{
    int temp;
    le_result_t r = ReadIntFromFile(TemperatureFile, &temp);
    if (r != LE_OK)
    {
        return r;
    }

    // The divider is 1000 based on the comments in the kernel driver on bmp280_compensate_temp()
    // which is called by bmp280_read_temp()
    *reading = ((double)temp) / 1000.0;
    return LE_OK;
}
