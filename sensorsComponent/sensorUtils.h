#ifndef SENSOR_UTILS_H
#define SENSOR_UTILS_H

#include "legato.h"

le_result_t ReadIntFromFile(const char *filePath, int *value);
le_result_t ReadDoubleFromFile(const char *filePath, double *value);

#endif // SENSOR_UTILS_H
