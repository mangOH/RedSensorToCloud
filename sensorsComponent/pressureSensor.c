#include "legato.h"
#include "interfaces.h"

#include "pressureSensor.h"
#include "sensorUtils.h"

static const char PressureFile[] = "/sys/devices/i2c-0/0-0076/iio:device0/in_pressure_input";
static const char TemperatureFile[] = "/sys/devices/i2c-0/0-0076/iio:device0/in_temp_input";

le_result_t mangOH_ReadPressureSensor
(
    double *reading
)
{
    return ReadDoubleFromFile(PressureFile, reading);
}

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

    // TODO: verify that divider is correct
    *reading = temp / 1000.0;
    return LE_OK;
}
