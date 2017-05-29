#include "legato.h"
#include "interfaces.h"
#include "lightSensor.h"
#include "pressureSensor.h"
#include "accelerometer.h"


//--------------------------------------------------------------------------------------------------
/*
 * type definitions
 */
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
/**
 * Represents a single logical sensor reading. The function pointers provide a generic way to read a
 * value and check if the reading exceeds the threshold for publishing.
 */
//--------------------------------------------------------------------------------------------------
struct Item
{
    const char *name;
    le_result_t (*read)(void *value);
    bool (*thresholdCheck)(const void *recordedValue, const void *readValue);
    le_result_t (*record)(le_avdata_RecordRef_t ref, uint64_t timestamp, void *value);
    void (*copyValue)(void *dest, const void *src);
    void *lastValueRead;
    void *lastValueRecorded;
    uint64_t lastTimeRecorded;
    uint64_t lastTimeRead;
};

struct Acceleration
{
    double x;
    double y;
    double z;
};

struct Gyro
{
    double x;
    double y;
    double z;
};

struct SensorReadings
{
    int32_t lightLevel;
    double pressure;
    double temperature;
    struct Acceleration acc;
    struct Gyro gyro;
};

//--------------------------------------------------------------------------------------------------
/*
 * static function declarations
 */
//--------------------------------------------------------------------------------------------------
static void PushCallbackHandler(le_avdata_PushStatus_t status, void* context);
static uint64_t GetCurrentTimestamp(void);
static void SampleTimerHandler(le_timer_Ref_t timer);

static le_result_t LightSensorRead(void *value);
static bool LightSensorThreshold(const void *recordedValue, const void* readValue);
static le_result_t LightSensorRecord(le_avdata_RecordRef_t ref, uint64_t timestamp, void *value);
static void LightSensorCopyValue(void *dest, const void *src);

static le_result_t PressureSensorRead(void *value);
static bool PressureSensorThreshold(const void *recordedValue, const void* readValue);
static le_result_t PressureSensorRecord(le_avdata_RecordRef_t ref, uint64_t timestamp, void *value);
static void PressureSensorCopyValue(void *dest, const void *src);

static le_result_t TemperatureSensorRead(void *value);
static bool TemperatureSensorThreshold(const void *recordedValue, const void* readValue);
static le_result_t TemperatureSensorRecord(le_avdata_RecordRef_t ref, uint64_t timestamp, void *value);
static void TemperatureSensorCopyValue(void *dest, const void *src);

static le_result_t AccelerometerRead(void *value);
static bool AccelerometerThreshold(const void *recordedValue, const void* readValue);
static le_result_t AccelerometerRecord(le_avdata_RecordRef_t ref, uint64_t timestamp, void *value);
static void AccelerometerCopyValue(void *dest, const void *src);

static le_result_t GyroRead(void *value);
static bool GyroThreshold(const void *recordedValue, const void* readValue);
static le_result_t GyroRecord(le_avdata_RecordRef_t ref, uint64_t timestamp, void *value);
static void GyroCopyValue(void *dest, const void *src);

static void AvSessionStateHandler (le_avdata_SessionState_t state, void *context);



//--------------------------------------------------------------------------------------------------
/*
 * variable definitions
 */
//--------------------------------------------------------------------------------------------------

// Wait time between each round of sensor readings.
static const int DelayBetweenReadings = 1;

// The maximum amount of time to wait for a reading to exceed a threshold before a publish is
// forced.
static const int MaxIntervalBetweenPublish = 120;

// The minimum amount of time to wait between publishing data.
static const int MinIntervalBetweenPublish = 10;

// How old the last published value must be for an item to be considered stale. The next time a
// publish occurs, the most recent reading of all stale items will be published.
static const int TimeToStale = 60;

static le_timer_Ref_t SampleTimer;
static le_avdata_RequestSessionObjRef_t AvSession;
static le_avdata_RecordRef_t RecordRef;
static le_avdata_SessionStateHandlerRef_t HandlerRef;

static bool DeferredPublish = false;
static uint64_t LastTimePublished = 0;

static struct
{
    struct SensorReadings recorded;
    struct SensorReadings read;
    bool readPreviously;
} SensorData = { .readPreviously=false };

struct Item Items[] =
{
    {
        .name = "light level",
        .read = LightSensorRead,
        .thresholdCheck = LightSensorThreshold,
        .record = LightSensorRecord,
        .copyValue = LightSensorCopyValue,
        .lastValueRead = &SensorData.read.lightLevel,
        .lastValueRecorded = &SensorData.recorded.lightLevel,
        .lastTimeRead = 0,
        .lastTimeRecorded = 0,
    },
    {
        .name = "pressure",
        .read = PressureSensorRead,
        .thresholdCheck = PressureSensorThreshold,
        .record = PressureSensorRecord,
        .copyValue = PressureSensorCopyValue,
        .lastValueRead = &SensorData.read.pressure,
        .lastValueRecorded = &SensorData.recorded.pressure,
        .lastTimeRead = 0,
        .lastTimeRecorded = 0,
    },
    {
        .name = "temperature",
        .read = TemperatureSensorRead,
        .thresholdCheck = TemperatureSensorThreshold,
        .record = TemperatureSensorRecord,
        .copyValue = TemperatureSensorCopyValue,
        .lastValueRead = &SensorData.read.temperature,
        .lastValueRecorded = &SensorData.recorded.temperature,
        .lastTimeRead = 0,
        .lastTimeRecorded = 0,
    },
    {
        .name = "accelerometer",
        .read = AccelerometerRead,
        .thresholdCheck = AccelerometerThreshold,
        .record = AccelerometerRecord,
        .copyValue = AccelerometerCopyValue,
        .lastValueRead = &SensorData.read.acc,
        .lastValueRecorded = &SensorData.recorded.acc,
        .lastTimeRead = 0,
        .lastTimeRecorded = 0,
    },
    {
        .name = "gyro",
        .read = GyroRead,
        .thresholdCheck = GyroThreshold,
        .record = GyroRecord,
        .copyValue = GyroCopyValue,
        .lastValueRead = &SensorData.read.gyro,
        .lastValueRecorded = &SensorData.recorded.gyro,
        .lastTimeRead = 0,
        .lastTimeRecorded = 0,
    },
};


//--------------------------------------------------------------------------------------------------
/*
 * static function definitions
 */
//--------------------------------------------------------------------------------------------------

static void PushCallbackHandler
(
    le_avdata_PushStatus_t status,
    void* context
)
{
    switch (status)
    {
    case LE_AVDATA_PUSH_SUCCESS:
        // data pushed successfully
        break;

    case LE_AVDATA_PUSH_FAILED:
        LE_WARN("Push was not successful");
        break;

    default:
        LE_ERROR("Unhandled push status %d", status);
        break;
    }
}


static uint64_t GetCurrentTimestamp(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t utcMilliSec = (uint64_t)(tv.tv_sec) * 1000 + (uint64_t)(tv.tv_usec) / 1000;
    return utcMilliSec;
}


static void SampleTimerHandler
(
    le_timer_Ref_t timer
)
{
    uint64_t now = GetCurrentTimestamp();
    bool publish = false;

    for (int i = 0; i < NUM_ARRAY_MEMBERS(Items); i++)
    {
        le_result_t r;
        struct Item *it = &Items[i];
        r = it->read(it->lastValueRead);
        if (r == LE_OK)
        {
            it->lastTimeRead = now;
            if (it->lastTimeRecorded == 0 || it->thresholdCheck(it->lastValueRead, it->lastValueRecorded))
            {
                r = it->record(RecordRef, now, it->lastValueRead);
                if (r == LE_OK)
                {
                    it->copyValue(it->lastValueRecorded, it->lastValueRead);
                    publish = true;
                }
                else
                {
                    LE_WARN("Failed to record %s", it->name);
                }
            }
        }
        else
        {
            LE_WARN("Failed to read %s", it->name);
        }

        if ((now - it->lastTimeRecorded) > (TimeToStale * 1000) &&
            it->lastTimeRead > LastTimePublished)
        {
            publish = true;
        }
    }

    if (publish || DeferredPublish)
    {
        if ((now - LastTimePublished) < MinIntervalBetweenPublish)
        {
            DeferredPublish = true;
        }
        else
        {
            // Find all of the stale items and record their current reading
            for (int i = 0; i < NUM_ARRAY_MEMBERS(Items); i++)
            {
                struct Item *it = &Items[i];
                if ((now - it->lastTimeRecorded) > (TimeToStale * 1000) &&
                    it->lastTimeRead > it->lastTimeRecorded)
                {
                    le_result_t r = it->record(RecordRef, it->lastTimeRead, it->lastValueRead);
                    if (r == LE_OK)
                    {
                        it->copyValue(it->lastValueRecorded, it->lastValueRead);
                        it->lastTimeRecorded = it->lastTimeRead;
                    }
                    else
                    {
                        LE_WARN("Failed to record %s", it->name);
                    }
                }
            }

            le_result_t r = le_avdata_PushRecord(RecordRef, PushCallbackHandler, NULL);
            if (r != LE_OK)
            {
                LE_ERROR("Failed to push record - %s", LE_RESULT_TXT(r));
            }
            else
            {
                LastTimePublished = now;
                DeferredPublish = false;
            }
        }
    }
}


static le_result_t LightSensorRead
(
    void *value
)
{
    int32_t *v = value;
    return mangOH_ReadLightSensor(v);
}

static bool LightSensorThreshold
(
    const void *recordedValue,
    const void *readValue
)
{
    const int32_t *v1 = recordedValue;
    const int32_t *v2 = readValue;

    return abs(*v1 - *v2) > 200;
}

static le_result_t LightSensorRecord
(
    le_avdata_RecordRef_t ref,
    uint64_t timestamp,
    void *value
)
{
    const char *path = "Sensors/Light/Level";
    int32_t *v = value;
    le_result_t result = le_avdata_RecordInt(RecordRef, path, *v, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record light sensor reading - %s", LE_RESULT_TXT(result));
    }

    return result;
}


static void LightSensorCopyValue
(
    void *dest,
    const void *src
)
{
    int32_t *d = dest;
    const int32_t *s = src;
    *d = *s;
}


static le_result_t PressureSensorRead
(
    void *value
)
{
    double *v = value;
    return mangOH_ReadPressureSensor(v);
}

static bool PressureSensorThreshold
(
    const void *recordedValue,
    const void *readValue
)
{
    const double *v1 = recordedValue;
    const double *v2 = readValue;

    // TODO: What units is pressure in and what is a suitable threshold?
    return fabs(*v1 - *v2) > 200.0;
}

static le_result_t PressureSensorRecord
(
    le_avdata_RecordRef_t ref,
    uint64_t timestamp,
    void *value
)
{
    const char *path = "Sensors/Pressure/Pressure";
    int32_t *v = value;
    le_result_t result = le_avdata_RecordFloat(RecordRef, path, *v, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record pressure sensor reading - %s", LE_RESULT_TXT(result));
    }

    return result;
}


static void PressureSensorCopyValue
(
    void *dest,
    const void *src
)
{
    double *d = dest;
    const double *s = src;
    *d = *s;
}


static le_result_t TemperatureSensorRead
(
    void *value
)
{
    double *v = value;
    return mangOH_ReadTemperatureSensor(v);
}

static bool TemperatureSensorThreshold
(
    const void *recordedValue,
    const void *readValue
)
{
    const double *v1 = recordedValue;
    const double *v2 = readValue;

    return fabs(*v1 - *v2) > 2.0;
}

static le_result_t TemperatureSensorRecord
(
    le_avdata_RecordRef_t ref,
    uint64_t timestamp,
    void *value
)
{
    const char *path = "Sensors/Pressure/Temperature";
    int32_t *v = value;
    le_result_t result = le_avdata_RecordFloat(RecordRef, path, *v, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record pressure sensor reading - %s", LE_RESULT_TXT(result));
    }

    return result;
}


static void TemperatureSensorCopyValue
(
    void *dest,
    const void *src
)
{
    double *d = dest;
    const double *s = src;
    *d = *s;
}


static le_result_t AccelerometerRead
(
    void *value
)
{
    struct Acceleration *v = value;
    return mangOH_ReadAccelerometer(&v->x, &v->y, &v->z);
}

static bool AccelerometerThreshold
(
    const void *recordedValue,
    const void *readValue
)
{
    const struct Acceleration *v1 = recordedValue;
    const struct Acceleration *v2 = readValue;

    double deltaX = v1->x - v2->x;
    double deltaY = v1->y - v2->y;
    double deltaZ = v1->z - v2->z;

    double deltaAcc = sqrt(pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2));

    // TODO: I think that the acceleration is in m/s^2, so 4.9 is half of a G.
    return fabs(deltaAcc) > 4.9;
}

static le_result_t AccelerometerRecord
(
    le_avdata_RecordRef_t ref,
    uint64_t timestamp,
    void *value
)
{
    // The '_' is a placeholder that will be replaced
    char path[] = "Sensors/Accelerometer/Acceleration/_";
    struct Acceleration *v = value;
    le_result_t result = LE_FAULT;

    path[sizeof(path) - 1] = 'X';
    result = le_avdata_RecordFloat(RecordRef, path, v->x, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record accelerometer x reading - %s", LE_RESULT_TXT(result));
        goto done;
    }

    path[sizeof(path) - 1] = 'Y';
    result = le_avdata_RecordFloat(RecordRef, path, v->y, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record accelerometer y reading - %s", LE_RESULT_TXT(result));
        goto done;
    }

    path[sizeof(path) - 1] = 'Z';
    result = le_avdata_RecordFloat(RecordRef, path, v->z, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record accelerometer z reading - %s", LE_RESULT_TXT(result));
        goto done;
    }

done:
    return result;
}


static void AccelerometerCopyValue
(
    void *dest,
    const void *src
)
{
    struct Acceleration *d = dest;
    const struct Acceleration *s = src;
    d->x = s->x;
    d->y = s->y;
    d->z = s->z;
}


static le_result_t GyroRead
(
    void *value
)
{
    struct Gyro *v = value;
    return mangOH_ReadGyro(&v->x, &v->y, &v->z);
}

static bool GyroThreshold
(
    const void *recordedValue,
    const void *readValue
)
{
    const struct Acceleration *v1 = recordedValue;
    const struct Acceleration *v2 = readValue;

    double deltaX = v1->x - v2->x;
    double deltaY = v1->y - v2->y;
    double deltaZ = v1->z - v2->z;

    double deltaAcc = sqrt(pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2));

    // TODO: What is the unit of the gyro? degrees per second? revolutions per second? revolutions
    // per minute? radians per second? other?
    return fabs(deltaAcc) > 10.0;
}

static le_result_t GyroRecord
(
    le_avdata_RecordRef_t ref,
    uint64_t timestamp,
    void *value
)
{
    // The '_' is a placeholder that will be replaced
    char path[] = "Sensors/Accelerometer/Gyro/_";
    struct Gyro *v = value;
    le_result_t result = LE_FAULT;

    path[sizeof(path) - 1] = 'X';
    result = le_avdata_RecordFloat(RecordRef, path, v->x, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record gyro x reading - %s", LE_RESULT_TXT(result));
        goto done;
    }

    path[sizeof(path) - 1] = 'Y';
    result = le_avdata_RecordFloat(RecordRef, path, v->y, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record gyro y reading - %s", LE_RESULT_TXT(result));
        goto done;
    }

    path[sizeof(path) - 1] = 'Z';
    result = le_avdata_RecordFloat(RecordRef, path, v->z, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record gyro z reading - %s", LE_RESULT_TXT(result));
        goto done;
    }

done:
    return result;
}


static void GyroCopyValue
(
    void *dest,
    const void *src
)
{
    struct Gyro *d = dest;
    const struct Gyro *s = src;
    d->x = s->x;
    d->y = s->y;
    d->z = s->z;
}


static void AvSessionStateHandler
(
    le_avdata_SessionState_t state,
    void *context
)
{
    switch (state)
    {
        case LE_AVDATA_SESSION_STARTED:
            LE_ASSERT_OK(le_timer_Start(SampleTimer));
            break;

        case LE_AVDATA_SESSION_STOPPED:
            LE_ASSERT_OK(le_timer_Stop(SampleTimer));
            break;

        default:
            LE_ERROR("Unsupported AV session state %d", state);
            break;
    }
}

COMPONENT_INIT
{
    HandlerRef = le_avdata_AddSessionStateHandler(AvSessionStateHandler, NULL);
    AvSession = le_avdata_RequestSession();
    // TODO: Ther eis an issue with le_avdata_RequestSession() where it returns NULL if the control
    // app has already established a session. For now we just ignore the return value and hope that
    // a session has been requested.
    //LE_FATAL_IF(AvSession == NULL, "Failed to request avdata session");
    LE_INFO("AvSession=0x%p", AvSession);
    RecordRef = le_avdata_CreateRecord();

    SampleTimer = le_timer_Create("Sensor Read");
    LE_ASSERT_OK(le_timer_SetMsInterval(SampleTimer, DelayBetweenReadings * 1000));
    LE_ASSERT_OK(le_timer_SetRepeat(SampleTimer, 0));
    LE_ASSERT_OK(le_timer_SetHandler(SampleTimer, SampleTimerHandler));
}
