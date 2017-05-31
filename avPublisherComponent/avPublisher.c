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
 * An abstract representation of a sensor
 *
 * Values are represented as void* because different sensors may produce double, uint32_t or a
 * custom struct type. The only requirement is that all of the functions must expect the same type
 * of value.
 */
//--------------------------------------------------------------------------------------------------
struct Item
{
    // A human readable name for the sensor
    const char *name;

    // Reads a value from the sensor
    le_result_t (*read)(void *value);

    // Checks to see if the value read exceeds the threshold relative to the last recorded value. If
    // the function returns true, the readValue will be recorded.
    bool (*thresholdCheck)(const void *recordedValue, const void *readValue);

    // Records the value into the given record.
    le_result_t (*record)(le_avdata_RecordRef_t ref, uint64_t timestamp, void *value);

    // Copies the sensor reading from src to dest. The management of the src and dest data is the
    // responsibility of the caller.
    void (*copyValue)(void *dest, const void *src);

    // Most recently read value from the sensor. Should be initialized to point at a variable large
    // enough to store a sensor reading.
    void *lastValueRead;

    // Most recently recorded value from the sensor. Must be initialized to point to a variable to
    // store a reading and must be a differnt variable from what lastValueRead is pointing at.
    void *lastValueRecorded;

    // Time when the last reading was recorded.
    uint64_t lastTimeRecorded;

    // Time when the last reading was read.
    uint64_t lastTimeRead;
};

//--------------------------------------------------------------------------------------------------
/**
 * 3D acceleration value.
 */
//--------------------------------------------------------------------------------------------------
struct Acceleration
{
    double x;
    double y;
    double z;
};

//--------------------------------------------------------------------------------------------------
/**
 * A 3D angular velocity value read from the accelerometer.
 */
//--------------------------------------------------------------------------------------------------
struct Gyro
{
    double x;
    double y;
    double z;
};

//--------------------------------------------------------------------------------------------------
/**
 * A data structure that stores a single reading from all of the sensors.
 */
//--------------------------------------------------------------------------------------------------
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


//--------------------------------------------------------------------------------------------------
/*
 * Data storage for sensor readings.
 *
 * This struct contains the most recently read values from the sensors and the most recently
 * recorded values from the sensors.
 */
//--------------------------------------------------------------------------------------------------
static struct
{
    struct SensorReadings recorded;  // sensor values most recently recorded
    struct SensorReadings read;      // sensor values most recently read
} SensorData;

/* static bool ReadLightOnly; */

//--------------------------------------------------------------------------------------------------
/**
 * An array representing all of the sensor values to read and publish
 */
//--------------------------------------------------------------------------------------------------
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


//--------------------------------------------------------------------------------------------------
/**
 * Handles notification of LWM2M push status.
 *
 * This function will warn if there is an error in pushing data, but it does not make any attempt to
 * retry pushing the data.
 */
//--------------------------------------------------------------------------------------------------
static void PushCallbackHandler
(
    le_avdata_PushStatus_t status, ///< Push success/failure status
    void* context                  ///< Not used
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


//--------------------------------------------------------------------------------------------------
/**
 * Convenience function to get current time as uint64_t.
 *
 * @return
 *      Current time as a uint64_t
 */
//--------------------------------------------------------------------------------------------------
static uint64_t GetCurrentTimestamp(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t utcMilliSec = (uint64_t)(tv.tv_sec) * 1000 + (uint64_t)(tv.tv_usec) / 1000;
    return utcMilliSec;
}


//--------------------------------------------------------------------------------------------------
/**
 * Handler for the sensor sampling timer
 *
 * Each time this function is called due to timer expiry each sensor described in the Items array
 * will be read. If any sensor item's thresholdCheck() function returns true, then that reading is
 * recorded and a publish action is scheduled. The data will be published immediately unless fewer
 * than MinIntervalBetweenPublish seconds have elapsed since the last publish. If that is the case,
 * the publish will be deferred until the minimum wait has elapsed. If no publish has occurred for
 * MaxIntervalBetweenPublish seconds, then a publish is forced. When a push is about to be executed
 * the list of items is checked again for any entries which have not been recorded in greater than
 * TimeToStale seconds. Stale items are recorded and then the record is published.
 */
//--------------------------------------------------------------------------------------------------
static void SampleTimerHandler
(
    le_timer_Ref_t timer  ///< Sensor sampling timer
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

        if ((now - it->lastTimeRecorded) > (MaxIntervalBetweenPublish * 1000) &&
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

//--------------------------------------------------------------------------------------------------
/**
 * Read the light sensor
 *
 * @return
 *      LE_OK on success.  Any other return value is a failure.
 */
//--------------------------------------------------------------------------------------------------
static le_result_t LightSensorRead
(
    void *value  ///< Pointer to the int32_t variable to store the reading in
)
{
    int32_t *v = value;
    return mangOH_ReadLightSensor(v);
}

//--------------------------------------------------------------------------------------------------
/**
 * Checks to see if the light level has changed sufficiently to warrant recording of a new reading.
 *
 * @return
 *      true if the threshold for recording has been exceeded
 */
//--------------------------------------------------------------------------------------------------
static bool LightSensorThreshold
(
    const void *recordedValue, ///< Last recorded light sensor reading
    const void *readValue      ///< Most recent light sensor reading
)
{
    const int32_t *v1 = recordedValue;
    const int32_t *v2 = readValue;

    return abs(*v1 - *v2) > 200;
}

//--------------------------------------------------------------------------------------------------
/**
 * Records a light sensor reading at the given time into the given record
 *
 * @return
 *      - LE_OK on success
 *      - LE_OVERFLOW if the record is full
 *      - LE_FAULT non-specific failure
 */
//--------------------------------------------------------------------------------------------------
static le_result_t LightSensorRecord
(
    le_avdata_RecordRef_t ref, ///< Record reference to record the value into
    uint64_t timestamp,        ///< Timestamp to associate with the value
    void *value                ///< The int32_t value to record
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

//--------------------------------------------------------------------------------------------------
/**
 * Copies an int32_t light sensor reading between two void pointers
 */
//--------------------------------------------------------------------------------------------------
static void LightSensorCopyValue
(
    void *dest,      ///< copy destination
    const void *src  ///< copy source
)
{
    int32_t *d = dest;
    const int32_t *s = src;
    *d = *s;
}

//--------------------------------------------------------------------------------------------------
/**
 * Read the pressure sensor
 *
 * @return
 *      LE_OK on success.  Any other return value is a failure.
 */
//--------------------------------------------------------------------------------------------------
static le_result_t PressureSensorRead
(
    void *value  ///< Pointer to a double to store the reading in
)
{
    double *v = value;
    return mangOH_ReadPressureSensor(v);
}

//--------------------------------------------------------------------------------------------------
/**
 * Checks to see if the pressure has changed sufficiently to warrant recording of a new reading.
 *
 * @return
 *      true if the threshold for recording has been exceeded
 */
//--------------------------------------------------------------------------------------------------
static bool PressureSensorThreshold
(
    const void *recordedValue, ///< Last recorded pressure reading
    const void *readValue      ///< Most recent pressure reading
)
{
    const double *v1 = recordedValue;
    const double *v2 = readValue;

    return fabs(*v1 - *v2) > 1.0;
}

//--------------------------------------------------------------------------------------------------
/**
 * Records a pressure sensor reading at the given time into the given record
 *
 * @return
 *      - LE_OK on success
 *      - LE_OVERFLOW if the record is full
 *      - LE_FAULT non-specific failure
 */
//--------------------------------------------------------------------------------------------------
static le_result_t PressureSensorRecord
(
    le_avdata_RecordRef_t ref, ///< Record reference to record the value into
    uint64_t timestamp,        ///< Timestamp to associate with the value
    void *value                ///< The double value to record
)
{
    const char *path = "Sensors/Pressure/Pressure";
    double *v = value;
    le_result_t result = le_avdata_RecordFloat(RecordRef, path, *v, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record pressure sensor reading - %s", LE_RESULT_TXT(result));
    }

    return result;
}

//--------------------------------------------------------------------------------------------------
/**
 * Copies a double pressure reading between two void pointers
 */
//--------------------------------------------------------------------------------------------------
static void PressureSensorCopyValue
(
    void *dest,      ///< copy destination
    const void *src  ///< copy source
)
{
    double *d = dest;
    const double *s = src;
    *d = *s;
}


//--------------------------------------------------------------------------------------------------
/**
 * Read the temperature sensor
 *
 * @return
 *      LE_OK on success.  Any other return value is a failure.
 */
//--------------------------------------------------------------------------------------------------
static le_result_t TemperatureSensorRead
(
    void *value  ///< Pointer to the double variable to store the reading in
)
{
    double *v = value;
    return mangOH_ReadTemperatureSensor(v);
}

//--------------------------------------------------------------------------------------------------
/**
 * Checks to see if the temperature has changed sufficiently to warrant recording of a new reading.
 *
 * @return
 *      true if the threshold for recording has been exceeded
 */
//--------------------------------------------------------------------------------------------------
static bool TemperatureSensorThreshold
(
    const void *recordedValue, ///< Last recorded temperature reading
    const void *readValue      ///< Most recent temperature reading
)
{
    const double *v1 = recordedValue;
    const double *v2 = readValue;

    return fabs(*v1 - *v2) > 2.0;
}

//--------------------------------------------------------------------------------------------------
/**
 * Records a temperature reading at the given time into the given record
 *
 * @return
 *      - LE_OK on success
 *      - LE_OVERFLOW if the record is full
 *      - LE_FAULT non-specific failure
 */
//--------------------------------------------------------------------------------------------------
static le_result_t TemperatureSensorRecord
(
    le_avdata_RecordRef_t ref, ///< Record reference to record the value into
    uint64_t timestamp,        ///< Timestamp to associate with the value
    void *value                ///< The double value to record
)
{
    const char *path = "Sensors/Pressure/Temperature";
    double *v = value;
    le_result_t result = le_avdata_RecordFloat(RecordRef, path, *v, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record pressure sensor reading - %s", LE_RESULT_TXT(result));
    }

    return result;
}

//--------------------------------------------------------------------------------------------------
/**
 * Copies an double temperature reading between two void pointers
 */
//--------------------------------------------------------------------------------------------------
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

//--------------------------------------------------------------------------------------------------
/**
 * Read the acceleration from the accelerometer
 *
 * @return
 *      LE_OK on success.  Any other return value is a failure.
 */
//--------------------------------------------------------------------------------------------------
static le_result_t AccelerometerRead
(
    void *value  ///< Pointer to a struct Acceleration to store the reading in
)
{
    struct Acceleration *v = value;
    return mangOH_ReadAccelerometer(&v->x, &v->y, &v->z);
}

//--------------------------------------------------------------------------------------------------
/**
 * Checks to see if the acceleration has changed sufficiently to warrant recording of a new reading.
 *
 * @return
 *      true if the threshold for recording has been exceeded
 */
//--------------------------------------------------------------------------------------------------
static bool AccelerometerThreshold
(
    const void *recordedValue, ///< Last recorded acceleration reading
    const void *readValue      ///< Most recent acceleration reading
)
{
    const struct Acceleration *v1 = recordedValue;
    const struct Acceleration *v2 = readValue;

    double deltaX = v1->x - v2->x;
    double deltaY = v1->y - v2->y;
    double deltaZ = v1->z - v2->z;

    double deltaAcc = sqrt(pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2));

    // The acceleration is in m/s^2, so 4.9 is half of a G.
    return fabs(deltaAcc) > 4.9;
}

//--------------------------------------------------------------------------------------------------
/**
 * Records an acceleration at the given time into the given record
 *
 * @return
 *      - LE_OK on success
 *      - LE_OVERFLOW if the record is full
 *      - LE_FAULT non-specific failure
 */
//--------------------------------------------------------------------------------------------------
static le_result_t AccelerometerRecord
(
    le_avdata_RecordRef_t ref, ///< Record reference to record the value into
    uint64_t timestamp,        ///< Timestamp to associate with the value
    void *value                ///< The struct Acceleration value to record
)
{
    // The '_' is a placeholder that will be replaced
    char path[] = "Sensors/Accelerometer/Acceleration/_";
    struct Acceleration *v = value;
    le_result_t result = LE_FAULT;

    path[sizeof(path) - 2] = 'X';
    result = le_avdata_RecordFloat(RecordRef, path, v->x, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record accelerometer x reading - %s", LE_RESULT_TXT(result));
        goto done;
    }

    path[sizeof(path) - 2] = 'Y';
    result = le_avdata_RecordFloat(RecordRef, path, v->y, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record accelerometer y reading - %s", LE_RESULT_TXT(result));
        goto done;
    }

    path[sizeof(path) - 2] = 'Z';
    result = le_avdata_RecordFloat(RecordRef, path, v->z, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record accelerometer z reading - %s", LE_RESULT_TXT(result));
        goto done;
    }

done:
    return result;
}

//--------------------------------------------------------------------------------------------------
/**
 * Copies a struct Acceleration between two void pointers
 */
//--------------------------------------------------------------------------------------------------
static void AccelerometerCopyValue
(
    void *dest,      ///< copy destination
    const void *src  ///< copy source
)
{
    struct Acceleration *d = dest;
    const struct Acceleration *s = src;
    d->x = s->x;
    d->y = s->y;
    d->z = s->z;
}

//--------------------------------------------------------------------------------------------------
/**
 * Read the angular velocity from the accelerometer
 *
 * @return
 *      LE_OK on success.  Any other return value is a failure.
 */
//--------------------------------------------------------------------------------------------------
static le_result_t GyroRead
(
    void *value  ///< Pointer to a struct Gyro to store the reading in
)
{
    struct Gyro *v = value;
    return mangOH_ReadGyro(&v->x, &v->y, &v->z);
}

//--------------------------------------------------------------------------------------------------
/**
 * Checks to see if the angular velocity has changed sufficiently to warrant recording of a new
 * reading.
 *
 * @return
 *      true if the threshold for recording has been exceeded
 */
//--------------------------------------------------------------------------------------------------
static bool GyroThreshold
(
    const void *recordedValue, ///< Last recorded angular velocity
    const void *readValue      ///< Most recent angular velocity
)
{
    const struct Gyro *v1 = recordedValue;
    const struct Gyro *v2 = readValue;

    double deltaX = v1->x - v2->x;
    double deltaY = v1->y - v2->y;
    double deltaZ = v1->z - v2->z;

    double deltaAngVel = sqrt(pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2));

    return fabs(deltaAngVel) > (M_PI / 2.0);
}

//--------------------------------------------------------------------------------------------------
/**
 * Records an angular velocity at the given time into the given record
 *
 * @return
 *      - LE_OK on success
 *      - LE_OVERFLOW if the record is full
 *      - LE_FAULT non-specific failure
 */
//--------------------------------------------------------------------------------------------------
static le_result_t GyroRecord
(
    le_avdata_RecordRef_t ref, ///< Record reference to record the value into
    uint64_t timestamp,        ///< Timestamp to associate with the value
    void *value                ///< The struct Gyro value to record
)
{
    // The '_' is a placeholder that will be replaced
    char path[] = "Sensors/Accelerometer/Gyro/_";
    struct Gyro *v = value;
    le_result_t result = LE_FAULT;

    path[sizeof(path) - 2] = 'X';
    result = le_avdata_RecordFloat(RecordRef, path, v->x, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record gyro x reading - %s", LE_RESULT_TXT(result));
        goto done;
    }

    path[sizeof(path) - 2] = 'Y';
    result = le_avdata_RecordFloat(RecordRef, path, v->y, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record gyro y reading - %s", LE_RESULT_TXT(result));
        goto done;
    }

    path[sizeof(path) - 2] = 'Z';
    result = le_avdata_RecordFloat(RecordRef, path, v->z, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record gyro z reading - %s", LE_RESULT_TXT(result));
        goto done;
    }

done:
    return result;
}

//--------------------------------------------------------------------------------------------------
/**
 * Copies a struct Gyro between two void pointers
 */
//--------------------------------------------------------------------------------------------------
static void GyroCopyValue
(
    void *dest,      ///< copy destination
    const void *src  ///< copy source
)
{
    struct Gyro *d = dest;
    const struct Gyro *s = src;
    d->x = s->x;
    d->y = s->y;
    d->z = s->z;
}

//--------------------------------------------------------------------------------------------------
/**
 * Handle changes in the AirVantage session state
 *
 * When the session is started the timer to sample the sensors is started and when the session is
 * stopped so is the timer.
 */
//--------------------------------------------------------------------------------------------------
static void AvSessionStateHandler
(
    le_avdata_SessionState_t state,
    void *context
)
{
    switch (state)
    {
        case LE_AVDATA_SESSION_STARTED:
        {
            // TODO: checking for LE_BUSY is a temporary workaround for the session state problem
            // described below.
            le_result_t status = le_timer_Start(SampleTimer);
            if (status == LE_BUSY)
            {
                LE_INFO("Received session started when timer was already running");
            }
            else
            {
                LE_ASSERT_OK(status);
            }
            break;
        }

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
    RecordRef = le_avdata_CreateRecord();

    SampleTimer = le_timer_Create("Sensor Read");
    LE_ASSERT_OK(le_timer_SetMsInterval(SampleTimer, DelayBetweenReadings * 1000));
    LE_ASSERT_OK(le_timer_SetRepeat(SampleTimer, 0));
    LE_ASSERT_OK(le_timer_SetHandler(SampleTimer, SampleTimerHandler));

    HandlerRef = le_avdata_AddSessionStateHandler(AvSessionStateHandler, NULL);
    AvSession = le_avdata_RequestSession();
    // TODO: There is an issue with le_avdata_RequestSession() where it returns NULL if the control
    // app has already established a session. For now we just ignore the return value and hope that
    // a session has been requested.
    //LE_FATAL_IF(AvSession == NULL, "Failed to request avdata session");

    // TODO: There is another problem where a session started event will not be sent to the client
    // if a session is already established. As a workaround for this, we explicitly call the
    // AvSessionStateHandler and tell it that a session is established.
    AvSessionStateHandler(LE_AVDATA_SESSION_STARTED, NULL);
}
