#include <Arduino.h>
#include <RadioLib.h>
#include <Wire.h>
#include "Sensor.h"
#include <nrfx_timer.h>
#include <Servo.h>

#define SAMPLES_COUNT 10000
#define SAMPLE_PERIOD_MS 10
#define TRANSMIT_PERIOD_FACTOR 50 // 10ms * 50 = every 500ms

// Flight detection tuning all in meters
#define LAUNCH_ALTITUDE_THRESHOLD 5.0f // climb above this => launch detected
#define APOGEE_DESCENT_THRESHOLD 2.0f // drop this far below peak => falling
#define DESCENT_CONFIRM_SAMPLES 10 // consecutive falling samples to confirm (5 * 10ms = 50ms)

// Parachute servo
#define SERVO_PIN 33 
#define SERVO_CLOSED_ANGLE 0 // degrees: cover latched shut
#define SERVO_OPEN_ANGLE 90 // degrees: cover released

/*******************************************************************************
Globals
*******************************************************************************/

// SX1262 wiring on the Heltec Mesh Node T114 (from variant.h):
//   NSS/CS = P0.24, DIO1 = P0.20, RESET = P0.25, BUSY = P0.17
//   DIO2 drives the RF switch, DIO3 powers a 1.8 V TCXO.
SX1262 radio = new Module(SX126X_CS, SX126X_DIO1, SX126X_RESET, SX126X_BUSY);

Sensor sensor;
volatile bool sensor_due = false;
volatile bool transmit_due = false;
nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(1); // create an instance of timer1

int16_t timeframe_samples[TRANSMIT_PERIOD_FACTOR] = {0};
int16_t max_timeframe_altitude = 0;
uint16_t sample_idx = 0;
uint16_t launch_sample_idx = 0;

bool is_flying = false;
bool is_falling = false;        // parachute_due: set once apogee is passed
float peak_altitude = 0.0f;     // highest altitude seen since launch
uint16_t descent_count = 0;     // consecutive samples below (peak - threshold)

Servo parachute_servo;
bool parachute_deployed = false; // one-shot latch so we open the cover only once

/*******************************************************************************
Function prototypes
*******************************************************************************/
void timer1_isr_handler(nrf_timer_event_t event_type, void* p_context);
void send_to_ground(int16_t altitude);
void panic(const char* msg);
void process_sensor_reading();
void open_parachute_cover();

/*******************************************************************************
SETUP
*******************************************************************************/
void setup()
{
    pinMode(LED_GREEN, OUTPUT);
    pinMode(PIN_BUTTON1, INPUT_PULLUP);

    Serial.begin(115200);
    unsigned long start = millis();
    while (!Serial && millis() - start < 3000) {}

    nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG;
    config.frequency = NRF_TIMER_FREQ_1MHz;        // 1 tick = 1 µs
    config.mode      = NRF_TIMER_MODE_TIMER;
    config.bit_width = NRF_TIMER_BIT_WIDTH_32;
    config.interrupt_priority = 6;

    nrfx_err_t err = nrfx_timer_init(&m_timer, &config, timer1_isr_handler);
    if (err != NRFX_SUCCESS) {
        Serial.println("Timer init failed!");
        while (1);
    }

    // 2. Set compare value: 10ms = 10,000 µs at 1MHz
    uint32_t ticks = nrfx_timer_ms_to_ticks(&m_timer, SAMPLE_PERIOD_MS);

    nrfx_timer_extended_compare(
        &m_timer,
        NRF_TIMER_CC_CHANNEL0,
        ticks,
        NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,  // resets counter automatically
        true                                   // enable interrupt
    );
    nrfx_timer_enable(&m_timer);

    Wire1.begin();
    Wire1.setClock(400000);

    sensor_init(&sensor, &Wire1, BMP280_I2C_ADDR_PRIM);
    if (!sensor_begin(&sensor)) panic("BMP280 init failed");

    // Parachute servo: attach and hold the cover closed.
    parachute_servo.attach(SERVO_PIN);
    parachute_servo.write(SERVO_CLOSED_ANGLE);

    int radio_state = radio.begin(
        868.0, // freq MHz
        250.0, // BW kHz
        7, // SF
        7, // CR
        RADIOLIB_SX126X_SYNC_WORD_PRIVATE, // sync word
        14, // output power dBm
        8, // preamble
        1.8, // TCXO V
        false // useRegulatorLDO
    );
    if (radio_state != RADIOLIB_ERR_NONE) {
        Serial.printf("radio.begin failed, code %d\n", radio_state);
        while (true) { delay(1000); }
    }

    // DIO2 is wired to the antenna T/R switch on this module.
    radio.setDio2AsRfSwitch(true);
}

/*******************************************************************************
LOOP
*******************************************************************************/
void loop()
{
    if (sensor_due) {
        sensor_due = false;
        process_sensor_reading();    
    }

    if (transmit_due && is_flying) {
        transmit_due = false;
        send_to_ground(max_timeframe_altitude);
    }

    if (is_falling && !parachute_deployed) {
        open_parachute_cover();
        parachute_deployed = true;
    }

    // Reset rocket on button press
    if (!digitalRead(PIN_BUTTON1)) {
        sensor_reset_altitude(&sensor);
        is_flying = false;
        is_falling = false;
        peak_altitude = 0.0f;
        descent_count = 0;

        parachute_deployed = false;
        parachute_servo.write(SERVO_CLOSED_ANGLE); // re-arm: close the cover
    }
}


/*******************************************************************************
Function definitions
*******************************************************************************/
void timer1_isr_handler(nrf_timer_event_t event_type, void* p_context)
{
    if (event_type == NRF_TIMER_EVENT_COMPARE0) {
        sensor_due = true;
        if (sample_idx < TRANSMIT_PERIOD_FACTOR - 1) {
            sample_idx++;
        } else {
            sample_idx = 0;
            max_timeframe_altitude = 0;
            transmit_due = true;
        }
    }
}

void send_to_ground(int16_t altitude)
{
    digitalWrite(LED_GREEN, LED_STATE_ON);
    int state = radio.transmit((uint8_t*)&altitude, sizeof(int16_t));
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("  failed, code %d\n", state);
    }
    digitalWrite(LED_GREEN, !LED_STATE_ON);
}

void panic(const char* msg)
{
    digitalWrite(LED_GREEN, LED_STATE_ON);
    while (true) {
        Serial.println(msg);
        delay(1000);
    }
}

void process_sensor_reading()
{
    if (!sensor_update(&sensor)) {
        Serial.println("BMP280 read failed");
        return;
    }

    float alt = sensor.altitude;

    timeframe_samples[sample_idx] = (int16_t)alt;
    if (alt > max_timeframe_altitude) {
        max_timeframe_altitude = (int16_t)alt;
    }

    // Launch detection 
    if (!is_flying && alt > LAUNCH_ALTITUDE_THRESHOLD) {
        is_flying = true;
        peak_altitude = alt;
    }

    // descent detection
    if (is_flying && !is_falling) {
        if (alt > peak_altitude) {
            peak_altitude = alt;      // still climbing, update the peak
            descent_count = 0;
        } else if (alt < peak_altitude - APOGEE_DESCENT_THRESHOLD) {
            // Falling far enough below the peak; require several consecutive
            // samples so a single noisy reading can't trigger early.
            if (++descent_count >= DESCENT_CONFIRM_SAMPLES) {
                is_falling = true;    // parachute_due -> deploy
            }
        } else {
            descent_count = 0;        // within noise band, not a real descent
        }
    }
}

void open_parachute_cover()
{
    parachute_servo.write(SERVO_OPEN_ANGLE);
}