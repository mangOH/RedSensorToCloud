//--------------------------------------------------------------------------------------------------
/**
 * Implementation of the mangOH Red position sensor interface to the Data Hub.
 *
 * Copyright (C) Sierra Wireless Inc.
 */
//--------------------------------------------------------------------------------------------------

#include "legato.h"
#include "interfaces.h"
#include "periodicSensor.h"


static void Sample
(
    psensor_Ref_t ref,
    void *contextPtr
)
{
    int32_t lat;
    int32_t lon;
    int32_t hAccuracy;
    int32_t alt;
    int32_t vAccuracy;

    le_result_t posRes = le_pos_Get3DLocation(&lat, &lon, &hAccuracy, &alt, &vAccuracy);

    if (posRes == LE_OK)
    {
        char json[256];

        int len = snprintf(json,
                           sizeof(json),
                           "{ \"lat\": %lf, \"lon\": %lf, \"hAcc\": %lf,"
                            " \"alt\": %lf, \"vAcc\": %lf }",
                           (double)lat / 1000000.0,
                           (double)lon / 1000000.0,
                           (double)hAccuracy,
                           (double)alt / 1000.0,
                           (double)vAccuracy);
        if (len >= sizeof(json))
        {
            LE_FATAL("JSON string (len %d) is longer than buffer (size %zu).", len, sizeof(json));
        }

        psensor_PushJson(ref, 0 /* now */, json);
    }
    else
    {
        LE_ERROR("Failed to read sensor (%s).", LE_RESULT_TXT(posRes));
    }
}


COMPONENT_INIT
{
    // Activate the positioning service.
    le_posCtrl_ActivationRef_t posCtrlRef = le_posCtrl_Request();
    LE_FATAL_IF(posCtrlRef == NULL, "Couldn't activate positioning service");

    // Use the periodic sensor component from the Data Hub to implement the timer and Data Hub
    // interface.  We'll provide samples as JSON structures.
    psensor_Create("position", DHUBIO_DATA_TYPE_JSON, "", Sample, NULL);
}
