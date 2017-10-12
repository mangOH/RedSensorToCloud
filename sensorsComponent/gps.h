#ifndef GPS_H
#define GPS_H

#include "legato.h"

LE_SHARED le_result_t mangOH_ReadGps(
    double *latitude,     ///< WGS84 Latitude in degrees, positive North [resolution 1e-6].
    double *longitude,    ///< WGS84 Longitude in degrees, positive East [resolution 1e-6].
    double *hAccuracy,    ///< Horizontal position's accuracy in meters.
    double *altitude,     ///< Altitude in meters, above Mean Sea Level.
    double *vAccuracy     ///< Vertical position's accuracy in meters.
);

#endif // GPS_H
