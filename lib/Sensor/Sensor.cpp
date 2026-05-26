#include "Sensor.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Bosch's bmp280_dev has no user_data slot, so the C read/write callbacks
// can't be told which Sensor they belong to. We stash the active one here.
// Consequence: only one Sensor can be in use at a time.
static Sensor* sensor_instance = NULL;

static int8_t i2c_read(uint8_t dev_id, uint8_t reg, uint8_t* buf, uint16_t len) {
    if (!sensor_instance)
        return BMP280_E_COMM_FAIL;

    TwoWire* w = sensor_instance->i2c;
    w->beginTransmission(dev_id);
    w->write(reg);

    if (w->endTransmission(true) != 0)
        return BMP280_E_COMM_FAIL;

    if (w->requestFrom(dev_id, (uint8_t)len) != len)
        return BMP280_E_COMM_FAIL;

    for (uint16_t i = 0; i < len; i++) buf[i] = w->read();
        return BMP280_OK;
}

static int8_t i2c_write(uint8_t dev_id, uint8_t reg, uint8_t* buf, uint16_t len) {
    if (!sensor_instance)
        return BMP280_E_COMM_FAIL;

    TwoWire* w = sensor_instance->i2c;
    w->beginTransmission(dev_id);
    w->write(reg);
    w->write(buf, len);

    return (w->endTransmission() == 0) ? BMP280_OK : BMP280_E_COMM_FAIL;
}

static void bmp_delay_ms(uint32_t period) {
    delay(period);
}

extern "C" {

void sensor_init(Sensor* s, TwoWire* twowire, uint8_t deviceAddress)
{
    s->temperature = 0.0f;
    s->pressure = 0.0f;
    s->altitude = 0.0f;
    s->absolute_altitude = 0.0f;
    s->altitude_offset = 0.0f;
    s->i2c = twowire;
    s->deviceAddress = deviceAddress;
    s->sea_level_hpa = 1013.25f;
}

bool sensor_begin(Sensor* s)
{
    sensor_instance = s;

    s->dev.dev_id = s->deviceAddress;
    s->dev.intf = BMP280_I2C_INTF;
    s->dev.read = &i2c_read;
    s->dev.write = &i2c_write;
    s->dev.delay_ms = &bmp_delay_ms;

    if (bmp280_init(&s->dev) != BMP280_OK) {
        return false;
    }

    if (bmp280_get_config(&s->conf, &s->dev) != BMP280_OK) {
        return false;
    }

    s->conf.os_pres = BMP280_OS_16X;           // high-res pressure for altitude
    s->conf.os_temp = BMP280_OS_2X;
    s->conf.filter = BMP280_FILTER_COEFF_16;  // drop to COEFF_2 for fast ascent
    s->conf.odr = BMP280_ODR_0_5_MS;
    s->conf.spi3w_en = 0;

    if (bmp280_set_config(&s->conf, &s->dev) != BMP280_OK) {
        return false;
    }

    if (bmp280_set_power_mode(BMP280_NORMAL_MODE, &s->dev) != BMP280_OK) {
        return false;
    }

    return true;
}

bool sensor_update(Sensor* s)
{
    if (bmp280_get_uncomp_data(&s->uncomp, &s->dev) != BMP280_OK)
        return false;

    double tempC = 0.0, presPa = 0.0;
    if (bmp280_get_comp_temp_double(&tempC, s->uncomp.uncomp_temp, &s->dev) != BMP280_OK)
        return false;

    if (bmp280_get_comp_pres_double(&presPa, s->uncomp.uncomp_press, &s->dev) != BMP280_OK)
        return false;

    s->temperature = (float)tempC;
    s->pressure = (float)(presPa / 100.0);
    s->absolute_altitude = 44330.0f * (1.0f - powf(s->pressure / s->sea_level_hpa, 0.1903f));
    s->altitude = s->absolute_altitude + s->altitude_offset;

    return true;
}

void sensor_set_sea_level_pressure(Sensor* s, float hpa)
{
    s->sea_level_hpa = hpa;
}

void sensor_reset_altitude(Sensor* s)
{
    s->altitude_offset = -s->absolute_altitude;
}

} // extern "C"
