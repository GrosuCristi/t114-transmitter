#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include <Wire.h>
#include "bmp280.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Sensor
{
    // public — read after sensor_update()
    float temperature;   // degrees Celsius
    float pressure;      // hPa
    float altitude;      // meters above sea level (relative to sea_level_hpa)
    float absolute_altitude;      // meters above sea level (relative to sea_level_hpa)

    // private — set by sensor_init()/sensor_begin(), don't touch directly
    TwoWire* i2c;
    uint8_t deviceAddress;
    float sea_level_hpa;
    float altitude_offset;

    struct bmp280_dev dev;
    struct bmp280_config conf;
    struct bmp280_uncomp_data uncomp;
} Sensor;

void sensor_init(Sensor* s, TwoWire* twowire, uint8_t address);
bool sensor_begin(Sensor* s);
bool sensor_update(Sensor* s);
void sensor_set_sea_level_pressure(Sensor* s, float hpa);
void sensor_reset_altitude(Sensor* s);

#ifdef __cplusplus
}
#endif

#endif
