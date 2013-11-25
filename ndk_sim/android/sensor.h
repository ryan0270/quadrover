#ifndef ANDROID_SENSOR_H
#define ANDROID_SENSOR_H

/*
 * Sensor types
 * (keep in sync with hardware/sensor.h)
 */
enum {
    ASENSOR_TYPE_ACCELEROMETER      = 1,
    ASENSOR_TYPE_MAGNETIC_FIELD     = 2,
    ASENSOR_TYPE_GYROSCOPE          = 4,
    ASENSOR_TYPE_LIGHT              = 5,
    ASENSOR_TYPE_PROXIMITY          = 8
};

/*
 * Sensor accuracy measure
 */
enum {
    ASENSOR_STATUS_UNRELIABLE       = 0,
    ASENSOR_STATUS_ACCURACY_LOW     = 1,
    ASENSOR_STATUS_ACCURACY_MEDIUM  = 2,
    ASENSOR_STATUS_ACCURACY_HIGH    = 3
};

/* NOTE: Must match hardware/sensors.h */
typedef struct ASensorVector {
    union {
        float v[3];
        struct {
            float x;
            float y;
            float z;
        };
        struct {
            float azimuth;
            float pitch;
            float roll;
        };
    };
    int8_t status;
    uint8_t reserved[3];
} ASensorVector;

/* NOTE: Must match hardware/sensors.h */
typedef struct ASensorEvent {
    int32_t version; /* sizeof(struct ASensorEvent) */
    int32_t sensor;
    int32_t type;
    int32_t reserved0;
    int64_t timestamp;
    union {
        float           data[16];
        ASensorVector   vector;
        ASensorVector   acceleration;
        ASensorVector   magnetic;
        float           temperature;
        float           distance;
        float           light;
        float           pressure;
    };
    int32_t reserved1[4];
} ASensorEvent;


struct ASensorManager;
typedef struct ASensorManager ASensorManager;

struct ASensorEventQueue;
typedef struct ASensorEventQueue ASensorEventQueue;

struct ASensor;
typedef struct ASensor ASensor;
typedef ASensor const* ASensorRef;
typedef ASensorRef const* ASensorList;
#endif
