#include <Arduino.h>
#include <RadioLib.h>
#include <Wire.h>
#include "Sensor.h"
#include <nrfx_timer.h>
#include <Servo.h>

#define SAMPLE_PERIOD_MS 10
#define TRANSMIT_PERIOD_MS 200UL
#define TRANSMIT_TICKS (TRANSMIT_PERIOD_MS / SAMPLE_PERIOD_MS) // Transmit interval in ticks

#define ALT_SAMPLES_ARRAY_LENGTH 40 // the receiver screen can only show 40 points

// Flight detection tuning all in meters
#define LAUNCH_ALTITUDE_THRESHOLD 2.0f // climb above this => launch detected
#define APOGEE_DESCENT_THRESHOLD 1.0f // drop this far below peak => falling
#define DESCENT_CONFIRM_SAMPLES 5 // consecutive falling samples to confirm
#define ARM_DELAY_MS 7000UL // wait 7s after the button press before zeroing altitude

// automatic disarm
#define AUTO_DISARM_TIME_MS 300000UL // 5 minutes
#define AUTO_DISARM_TICKS (AUTO_DISARM_TIME_MS / SAMPLE_PERIOD_MS) // Auto disarm interval in ticks

// Parachute servo
#define SERVO_PIN 33
#define SERVO_CLOSED_ANGLE 10 // degrees: cover latched shut
#define SERVO_OPEN_ANGLE 100 // degrees: cover released
#define SERVO_MOVE_TIME_MS 750UL // travel time before detaching (no position feedback)
#define SERVO_MOVE_TICKS (SERVO_MOVE_TIME_MS / SAMPLE_PERIOD_MS) // travel time in timer1 ticks

/*******************************************************************************
Globals
*******************************************************************************/
SX1262 radio = new Module(SX126X_CS, SX126X_DIO1, SX126X_RESET, SX126X_BUSY);

nrfx_timer_t timer = NRFX_TIMER_INSTANCE(1); // create an instance of timer1
volatile uint16_t transmit_countdown = 0;
volatile uint16_t servo_detach_countdown = 0;
volatile uint32_t auto_disarm_countdown = 0;
volatile bool sensor_due = false;

int16_t max_timeframe_altitude = 0;

bool is_armed = false;
bool is_flying = false;
bool is_falling = false;
float peak_altitude = 0.0f; 
uint16_t descent_count = 0;

int16_t altitude_samples[ALT_SAMPLES_ARRAY_LENGTH] = {0};
uint16_t altitude_sample_idx = 0;
bool samples_transmitted = false;

Sensor sensor;

Servo servo;

/*******************************************************************************
Function prototypes
*******************************************************************************/
void timer1_isr_handler(nrf_timer_event_t event_type, void* p_context);
void send_to_ground(int16_t altitude);
void panic(const char* msg);
void handle_sensor_reading(void);
void flight_engine_routine(void);
void handle_button_press(void);
void button_update(void);
void servo_goto(int angle);
void servo_update(void);
void handle_transmit_samples(void);

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

    nrfx_err_t err = nrfx_timer_init(&timer, &config, timer1_isr_handler);
    if (err != NRFX_SUCCESS) {
        panic("Timer init failed");
    }

    // 2. Set compare value: 10ms = 10,000 µs at 1MHz
    uint32_t ticks = nrfx_timer_ms_to_ticks(&timer, SAMPLE_PERIOD_MS);

    nrfx_timer_extended_compare(
        &timer,
        NRF_TIMER_CC_CHANNEL0,
        ticks,
        NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,  // resets counter automatically
        true                                   // enable interrupt
    );
    nrfx_timer_enable(&timer);

    Wire1.begin();
    Wire1.setClock(400000);
    
    sensor_init(&sensor, &Wire1, BMP280_I2C_ADDR_PRIM);
    if (!sensor_begin(&sensor)) panic("BMP280 init failed");

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
        char msg[40];
        sprintf(msg, "radio.begin failed, code %d\n", radio_state);
        panic(msg);
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
        handle_sensor_reading();

        if (is_armed) {
            flight_engine_routine();
        }
    }

    // Transmit only while flying
    if (transmit_countdown == 0 ) {
        transmit_countdown = TRANSMIT_TICKS;

        if (is_flying && !is_falling && altitude_sample_idx < ALT_SAMPLES_ARRAY_LENGTH) {
            altitude_samples[altitude_sample_idx] = max_timeframe_altitude;
            altitude_sample_idx++;
        }

        max_timeframe_altitude = 0;
        Serial.println(sensor.altitude);
    }

    if (auto_disarm_countdown == 0 && is_armed) {
        is_armed = false;
    }

    if (is_armed) {
        servo_goto(SERVO_CLOSED_ANGLE);
    } else {
        servo_goto(SERVO_OPEN_ANGLE);
    }
    
    servo_update();

    button_update();

    // transmit samples array
    if (is_flying && is_falling && !samples_transmitted) {
        handle_transmit_samples();
        samples_transmitted = true;
    }
}


/*******************************************************************************
Function definitions
*******************************************************************************/
void timer1_isr_handler(nrf_timer_event_t event_type, void* p_context)
{
    if (event_type == NRF_TIMER_EVENT_COMPARE0) {
        sensor_due = true;

        if (transmit_countdown > 0) {
            transmit_countdown--;
        }

        if (servo_detach_countdown > 0) {
            servo_detach_countdown--;
        }

        if (auto_disarm_countdown > 0) {
            auto_disarm_countdown--;
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

void handle_sensor_reading(void)
{
    if (!sensor_update(&sensor)) {
        Serial.println("BMP280 read failed");
        return;
    }
}

void flight_engine_routine(void)
{
    float alt = sensor.altitude;

    if (alt > max_timeframe_altitude) {
        max_timeframe_altitude = (int16_t)alt;
    }

    // Launch detection 
    if (!is_flying && alt > LAUNCH_ALTITUDE_THRESHOLD) {
        is_flying = true;
        peak_altitude = alt;
        altitude_sample_idx = 0;
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
                is_falling = true;    // deploy parachute
                is_armed = false;
            }
        } else {
            descent_count = 0;        // within noise band, not a real descent
        }
    }
}

void servo_goto(int angle)
{
    static int16_t servo_target_angle = -1;
    if (angle == servo_target_angle) return;
    servo_target_angle = angle;
    servo.attach(SERVO_PIN);
    servo.write(angle);
    servo_detach_countdown = SERVO_MOVE_TICKS; // timer1 ISR decrements every 10ms
}

// Once the countdown expires the servo has had time to reach the target: release it.
void servo_update(void)
{
    if (servo.attached() && servo_detach_countdown == 0) {
        servo.detach();
    }
}

void handle_button_press(void)
{
    is_flying = false;
    is_falling = false;
    samples_transmitted = false;
    descent_count = 0;
    
    if (is_armed) {
        is_armed = false;
    } else {
        digitalWrite(LED_GREEN, LED_STATE_ON);
        delay(ARM_DELAY_MS);
        digitalWrite(LED_GREEN, !LED_STATE_ON);

        is_armed = true;
        auto_disarm_countdown = AUTO_DISARM_TICKS;
        sensor_reset_altitude(&sensor);
    }
}

void button_update(void)
{
    static bool button_was_pressed = false;
    bool is_button_pressed = !digitalRead(PIN_BUTTON1);
    if (is_button_pressed && !button_was_pressed) {
        handle_button_press();
    }

    button_was_pressed = is_button_pressed;
}

void handle_transmit_samples(void)
{
    for (uint16_t i = 0; i < altitude_sample_idx; i++) {
        send_to_ground(altitude_samples[i]);
    }
}