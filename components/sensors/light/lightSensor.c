//--------------------------------------------------------------------------------------------------
/**
 * Implementation of the mangOH Red light sensor interface.
 *
 * Provides the accelerometer and gyro IPC API services and plugs into the Legato Data Hub.
 *
 * Copyright (C) Sierra Wireless Inc.
 */
//--------------------------------------------------------------------------------------------------

#include "legato.h"
#include "interfaces.h"
#include "periodicSensor.h"
#include "lightSensor.h"

const char lightSensorAdc[] = "EXT_ADC3";


static void Sample
(
    psensor_Ref_t ref,
    void *contextPtr
)
{
    int32_t sample;

    le_result_t result = light_Read(&sample);

    if (result == LE_OK)
    {
        psensor_PushNumeric(ref, 0 /* now */, (double)sample);
    }
    else
    {
        LE_ERROR("Failed to read sensor (%s).", LE_RESULT_TXT(result));
    }
}


COMPONENT_INIT
{
    psensor_Create("light", DHUBIO_DATA_TYPE_NUMERIC, "", Sample, NULL);
}


//--------------------------------------------------------------------------------------------------
/**
 * Read the light intensity measurement.
 *
 * @return LE_OK if successful.
 */
//--------------------------------------------------------------------------------------------------
le_result_t light_Read
(
    int32_t* readingPtr
        ///< [OUT] Where the light intensity reading will be put if LE_OK is returned.
)
{
    return le_adc_ReadValue(lightSensorAdc, readingPtr);
}
