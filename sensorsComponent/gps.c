#include "legato.h"
#include "interfaces.h"
#include "periodicSensor.h"
#include "gps.h"


static void Sample
(
    psensor_Ref_t ref
)
{
    double latitude;
    double longitude;
    double hAccuracy;
    double altitude;
    double vAccuracy;

    le_result_t result = mangOH_ReadGps(&latitude, &longitude, &hAccuracy, &altitude, &vAccuracy);

    if (result == LE_OK)
    {
        char json[256];

        int len = snprintf(json,
                           sizeof(json),
                           "{ \"lat\": %lf, \"lon\": %lf, \"hAcc\": %lf,"
                            " \"alt\": %lf, \"vAcc\": %lf }",
                           latitude,
                           longitude,
                           hAccuracy,
                           altitude,
                           vAccuracy);
        if (len >= sizeof(json))
        {
            LE_FATAL("JSON string (len %d) is longer than buffer (size %zu).", len, sizeof(json));
        }

        psensor_PushJson(ref, 0 /* now */, json);
    }
    else
    {
        LE_ERROR("Failed to read sensor (%s).", LE_RESULT_TXT(result));
    }
}


void gps_Init
(
    void
)
{
    psensor_Create("position", DHUBIO_DATA_TYPE_JSON, "", Sample);
}


le_result_t mangOH_ReadGps(
    double *latitude,     ///< WGS84 Latitude in degrees, positive North [resolution 1e-6].
    double *longitude,    ///< WGS84 Longitude in degrees, positive East [resolution 1e-6].
    double *hAccuracy,    ///< Horizontal position's accuracy in meters.
    double *altitude,     ///< Altitude in meters, above Mean Sea Level.
    double *vAccuracy     ///< Vertical position's accuracy in meters.
)
{
    int32_t latitudeReading;
    int32_t longitudeReading;
    int32_t hAccuracyReading;
    int32_t altitudeReading;
    int32_t vAccuracyReading;

    le_result_t posRes = le_pos_Get3DLocation(
        &latitudeReading,
        &longitudeReading,
        &hAccuracyReading,
        &altitudeReading,
        &vAccuracyReading);
    if (posRes == LE_OK)
    {
        *latitude  = latitudeReading / 1000000.0;
        *longitude = longitudeReading / 1000000.0;
        *hAccuracy = (double)hAccuracyReading;
        *altitude  = altitudeReading / 1000.0;
        *vAccuracy = (double)vAccuracyReading;
    }
    return posRes;
}
