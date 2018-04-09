#include "legato.h"
#include "interfaces.h"
#include "accelerometer.h"
#include "lightSensor.h"
#include "gps.h"
#include "pressureSensor.h"

COMPONENT_INIT
{
    le_posCtrl_ActivationRef_t posCtrlRef = le_posCtrl_Request();
    LE_FATAL_IF(posCtrlRef == NULL, "Couldn't activate positioning service");

    imu_Init();
    light_Init();
    gps_Init();
    pressure_Init();
}
