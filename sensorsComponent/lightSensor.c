#include "legato.h"
#include "interfaces.h"

#include "lightSensor.h"

const char lightSensorAdc[] = "EXT_ADC3";

le_result_t mangOH_ReadLightSensor
(
    int32_t *reading
)
{
    return le_adc_ReadValue(lightSensorAdc, reading);
}
