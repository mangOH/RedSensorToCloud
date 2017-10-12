#include "legato.h"
#include "interfaces.h"

COMPONENT_INIT
{
#ifdef GPS_ENABLE
    le_posCtrl_ActivationRef_t posCtrlRef = le_posCtrl_Request();
    LE_FATAL_IF(posCtrlRef == NULL, "Couldn't activate positioning service");
#endif // GPS_ENABLE
}
