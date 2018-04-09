#include "legato.h"
#include "interfaces.h"
#include "periodicSensor.h"
#include "lightSensor.h"

const char lightSensorAdc[] = "EXT_ADC3";


static void Sample
(
    psensor_Ref_t ref
)
{
    int32_t sample;

    le_result_t result = mangOH_ReadLightSensor(&sample);

    if (result == LE_OK)
    {
        psensor_PushNumeric(ref, 0 /* now */, (double)sample);
    }
    else
    {
        LE_ERROR("Failed to read sensor (%s).", LE_RESULT_TXT(result));
    }
}


void light_Init
(
    void
)
{
    psensor_Create("light", DHUBIO_DATA_TYPE_NUMERIC, "", Sample);
}


le_result_t mangOH_ReadLightSensor
(
    int32_t *reading
)
{
    return le_adc_ReadValue(lightSensorAdc, reading);
}
