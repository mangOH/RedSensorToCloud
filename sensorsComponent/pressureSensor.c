#include "legato.h"
#include "interfaces.h"

#include "pressureSensor.h"
#include "sensorUtils.h"

static const char PressureFile[] = "/sys/devices/i2c-0/0-0076/iio:device1/in_pressure_input";
static const char TemperatureFile[] = "/sys/devices/i2c-0/0-0076/iio:device1/in_temp_input";

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
