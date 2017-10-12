#include "legato.h"
#include "interfaces.h"

#include "gps.h"


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
