#include <Arduino.h>
#include <RadioLib.h>
#include <Wire.h>
#include "Sensor.h"
#include <nrfx_timer.h>
#include <Servo.h>

// Kalman filter (pure C library in lib/Kalman). Pulled in with C linkage so this
// C++ translation unit links against the C-compiled sources, following the
// library README's "copy-paste sources" usage with single-TU static inlines.
extern "C" {
#define EXTERN_INLINE_MATRIX static inline
#define EXTERN_INLINE_KALMAN static inline
#include <kalman.h>

// One filter named "altitude": 3 states [altitude, velocity, acceleration], 0 inputs.
#define KALMAN_NAME altitude
#define KALMAN_NUM_STATES 3
#define KALMAN_NUM_INPUTS 0
#include <kalman_factory_filter.h>

// One measurement named "baro": the single barometric altitude reading.
#define KALMAN_MEASUREMENT_NAME baro
#define KALMAN_NUM_MEASUREMENTS 1
#include <kalman_factory_measurement.h>

#include <kalman_factory_cleanup.h>
}

#define SAMPLE_PERIOD_MS 10
#define TRANSMIT_PERIOD_MS 200UL
#define TRANSMIT_TICKS (TRANSMIT_PERIOD_MS / SAMPLE_PERIOD_MS) // Transmit interval in ticks

#define ALT_SAMPLES_ARRAY_LENGTH 40 // the receiver screen can only show 40 points

// Flight detection tuning
#define SAMPLE_DT_S (SAMPLE_PERIOD_MS / 1000.0f) // Kalman time step (s)
#define LAUNCH_ALTITUDE_THRESHOLD 2.0f // climb above this => start logging (m)
#define LAUNCH_VELOCITY_THRESHOLD 2.0f // upward speed above this => real boost seen (m/s)

// Kalman tuning. R is the barometric measurement noise; lambda (<1) injects
// process noise so the filter keeps trusting new readings instead of rigidly
// extrapolating its own trajectory.
#define KF_MEAS_VARIANCE 1.0f // R: altitude measurement variance (m^2)
#define KF_INIT_VARIANCE 1.0f // initial diagonal of the state covariance P
#define KF_LAMBDA 0.98f       // fading-memory factor, 0 < lambda <= 1

// Apogee prediction: deploy as the filtered velocity is about to cross zero. The
// lead time fires the chute just before the geometric peak, so it is already out
// before the airframe can tilt over and start to fall.
#define APOGEE_LEAD_TIME_S 0.15f // deploy this long before predicted apogee (s)
#define APOGEE_CONFIRM_SAMPLES 3 // consecutive apogee detections to confirm

#define ARM_DELAY_MS 7000UL // wait 7s after the button press before zeroing altitude

// automatic disarm
#define AUTO_DISARM_TIME_MS 300000UL // 5 minutes
#define AUTO_DISARM_TICKS (AUTO_DISARM_TIME_MS / SAMPLE_PERIOD_MS) // Auto disarm interval in ticks

// Parachute servo
#define SERVO_PIN 33
#define SERVO_CLOSED_ANGLE 100 // degrees: cover latched shut
#define SERVO_OPEN_ANGLE 10 // degrees: cover released
#define SERVO_MOVE_TIME_MS 750UL // travel time before detaching (no position feedback)
#define SERVO_MOVE_TICKS (SERVO_MOVE_TIME_MS / SAMPLE_PERIOD_MS) // travel time in timer1 ticks

// Deployment profile. Two strategies depending on how high the flight got:
//  - Low flights have no room to coast, so the chute comes out right at apogee.
//  - Tall flights coast down and deploy lower to cut drift under canopy. A
//    descent-speed fallback still forces deployment if a long ballistic free
//    fall builds up dangerous speed before the target altitude is reached.
#define H_LOW_THRESHOLD_M    20.0f   // apogee below this => deploy at apogee (m)
#define H_DEPLOY_TARGET_M    20.0f   // on descent, deploy at/below this altitude (m)
#define V_SAFETY_FALLBACK_MS -30.0f  // deploy if descent speed exceeds this (m/s)

// Flight phase state machine. The single source of truth for where we are in
// the flight; parachute deployment and telemetry logging both key off it.
//   IDLE -> ASCENDING -> APOGEE_DETECTED -> DESCENDING -> DEPLOYED
enum flight_state_t {
    FLIGHT_IDLE,            // on the pad, waiting to leave
    FLIGHT_ASCENDING,       // climbing; watching for apogee
    FLIGHT_APOGEE_DETECTED, // peak reached; choosing a deploy strategy
    FLIGHT_DESCENDING,      // coasting down toward the deploy altitude
    FLIGHT_DEPLOYED,        // chute released (terminal)
};

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
flight_state_t flight_state = FLIGHT_IDLE;
bool boost_detected = false;   // a clear upward boost has been seen this flight
float max_altitude = 0.0f;     // highest filtered altitude this flight (m)
float peak_altitude = 0.0f;    // apogee altitude recorded at deployment (m)
float deploy_altitude = 0.0f;  // filtered altitude at deployment, for logging (m)
uint16_t apogee_count = 0;     // consecutive apogee detections

// Latest Kalman estimate, refreshed every flight cycle.
float kf_altitude = 0.0f;
float kf_velocity = 0.0f;
float kf_accel = 0.0f;

kalman_t* kf = nullptr;
kalman_measurement_t* kfm = nullptr;

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
void deploy_parachute(void);
void kalman_setup(void);
void kalman_reset(void);
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

    kalman_setup();

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

        if (flight_state != FLIGHT_IDLE && flight_state != FLIGHT_DEPLOYED &&
            altitude_sample_idx < ALT_SAMPLES_ARRAY_LENGTH) {
            altitude_samples[altitude_sample_idx] = max_timeframe_altitude;
            altitude_sample_idx++;
        }

        max_timeframe_altitude = 0;
        // Debug telemetry over USB serial: raw vs filtered altitude, plus the
        // estimated vertical velocity and acceleration used for apogee timing.
        Serial.print(sensor.altitude);
        Serial.print(' ');
        Serial.print(kf_altitude);
        Serial.print(' ');
        Serial.print(kf_velocity);
        Serial.print(' ');
        Serial.println(kf_accel);
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
    if (flight_state == FLIGHT_DEPLOYED && !samples_transmitted) {
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
    // Run one Kalman step on the latest barometric altitude: predict the state
    // forward one sample, then correct it with the new measurement. The tuned
    // predict injects process noise (lambda) so the velocity and acceleration
    // estimates stay responsive instead of locking onto a stale trajectory.
    kalman_predict_tuned(kf, KF_LAMBDA);
    matrix_set(kalman_get_measurement_vector(kfm), 0, 0, sensor.altitude);
    kalman_correct(kf, kfm);

    const matrix_t* x = kalman_get_state_vector(kf);
    kf_altitude = matrix_get(x, 0, 0);
    kf_velocity = matrix_get(x, 1, 0);
    kf_accel    = matrix_get(x, 2, 0);

    // Track the peak filtered altitude for the telemetry graph and, separately,
    // the highest altitude this flight (drives the apogee deploy-strategy branch).
    if (kf_altitude > max_timeframe_altitude) {
        max_timeframe_altitude = (int16_t)kf_altitude;
    }
    if (kf_altitude > max_altitude) {
        max_altitude = kf_altitude;
    }

    // Altitude-target deploy and missed-apogee failsafe. Once past the pad, at
    // or below the deploy target, and clearly coming down from the peak, release
    // the chute -- in any flight state. The "coming down" test uses how far we
    // have fallen below the peak rather than the velocity sign, so it still
    // fires if the velocity estimate has gone bad (the most likely reason apogee
    // detection would have failed in the first place). That drop margin also
    // stops it from firing on the way *up* through the target altitude.
    if (flight_state != FLIGHT_IDLE && flight_state != FLIGHT_DEPLOYED &&
        kf_altitude <= H_DEPLOY_TARGET_M &&
        (max_altitude - kf_altitude) > LAUNCH_ALTITUDE_THRESHOLD) {
        deploy_parachute();
        return;
    }

    switch (flight_state) {
    case FLIGHT_IDLE:
        // Leave the pad once we clearly climb above the launch threshold.
        if (kf_altitude > LAUNCH_ALTITUDE_THRESHOLD) {
            flight_state = FLIGHT_ASCENDING;
            altitude_sample_idx = 0;
        }
        break;

    case FLIGHT_ASCENDING: {
        // Only trust apogee detection once a real upward burn is seen, so pad
        // noise (velocity hovering around zero) can never deploy.
        if (!boost_detected && kf_velocity > LAUNCH_VELOCITY_THRESHOLD) {
            boost_detected = true;
        }
        if (!boost_detected) {
            break;
        }

        // Apogee is where vertical velocity crosses zero. While coasting the
        // craft decelerates (accel < 0), so the time until velocity reaches
        // zero is -v/a. Flag apogee once that is within the lead time, or once
        // the peak has already been reached (v <= 0).
        bool apogee_now = (kf_velocity <= 0.0f);
        bool apogee_imminent = false;
        if (kf_accel < 0.0f) {
            float time_to_apogee = -kf_velocity / kf_accel;
            apogee_imminent = (time_to_apogee <= APOGEE_LEAD_TIME_S);
        }

        if (apogee_now || apogee_imminent) {
            // Require a few consecutive detections so one noisy estimate can't
            // fire the chute early.
            if (++apogee_count >= APOGEE_CONFIRM_SAMPLES) {
                flight_state = FLIGHT_APOGEE_DETECTED;
            }
        } else {
            apogee_count = 0;
        }
        break;
    }

    case FLIGHT_APOGEE_DETECTED:
        // Branch on how high we got. Low flights have no room to coast, so the
        // chute comes out at the peak. Tall flights wait and deploy lower.
        if (max_altitude < H_LOW_THRESHOLD_M) {
            deploy_parachute();
        } else {
            flight_state = FLIGHT_DESCENDING;
        }
        break;

    case FLIGHT_DESCENDING:
        // The target-altitude deploy is handled by the failsafe block above.
        // This is the long-free-fall guard: a tall flight that builds up
        // dangerous descent speed deploys immediately, before reaching target.
        if (kf_velocity <= V_SAFETY_FALLBACK_MS) {
            deploy_parachute();
        }
        break;

    case FLIGHT_DEPLOYED:
        // Chute is out; nothing left to do.
        break;
    }
}

// Release the parachute. Idempotent: safe to call on every tick. Disarming
// opens the servo cover via the main-loop servo logic (armed => closed, else
// open), and records where/why the chute came out for post-flight analysis.
void deploy_parachute(void)
{
    if (flight_state == FLIGHT_DEPLOYED) {
        return;
    }

    peak_altitude   = max_altitude;
    deploy_altitude = kf_altitude;
    flight_state    = FLIGHT_DEPLOYED;
    is_armed        = false; // opens the cover (see servo logic in loop())

    Serial.print("DEPLOY state-entry alt=");
    Serial.print(deploy_altitude);
    Serial.print(" peak=");
    Serial.print(peak_altitude);
    Serial.print(" v=");
    Serial.println(kf_velocity);
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
    flight_state = FLIGHT_IDLE;
    samples_transmitted = false;
    boost_detected = false;
    apogee_count = 0;
    max_altitude = 0.0f;

    if (is_armed) {
        is_armed = false;
    } else {
        digitalWrite(LED_GREEN, LED_STATE_ON);
        delay(ARM_DELAY_MS);
        digitalWrite(LED_GREEN, !LED_STATE_ON);

        is_armed = true;
        auto_disarm_countdown = AUTO_DISARM_TICKS;
        sensor_reset_altitude(&sensor);
        kalman_reset();   // start the filter from rest at the new zero altitude
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

// Configure the constant Kalman matrices once at startup. The model is a
// constant-acceleration kinematics model sampled every SAMPLE_DT_S seconds:
//   altitude += velocity*dt + 0.5*accel*dt^2
//   velocity += accel*dt
//   accel    += 0            (driven only by process noise)
void kalman_setup(void)
{
    kf  = kalman_filter_altitude_init();
    kfm = kalman_filter_altitude_measurement_baro_init();

    const float dt = SAMPLE_DT_S;

    // State transition matrix A.
    matrix_t* A = kalman_get_state_transition(kf);
    matrix_set(A, 0, 0, 1.0f); matrix_set(A, 0, 1, dt);   matrix_set(A, 0, 2, 0.5f * dt * dt);
    matrix_set(A, 1, 0, 0.0f); matrix_set(A, 1, 1, 1.0f); matrix_set(A, 1, 2, dt);
    matrix_set(A, 2, 0, 0.0f); matrix_set(A, 2, 1, 0.0f); matrix_set(A, 2, 2, 1.0f);

    // We observe altitude only: H = [1 0 0].
    matrix_t* H = kalman_get_measurement_transformation(kfm);
    matrix_set(H, 0, 0, 1.0f);
    matrix_set(H, 0, 1, 0.0f);
    matrix_set(H, 0, 2, 0.0f);

    // Measurement noise covariance R.
    matrix_set(kalman_get_process_noise(kfm), 0, 0, KF_MEAS_VARIANCE);

    kalman_reset();
}

// Reset the filter to "at rest at altitude zero" with a fresh covariance. Called
// on every arm so each flight starts clean.
void kalman_reset(void)
{
    matrix_t* x = kalman_get_state_vector(kf);
    matrix_set(x, 0, 0, 0.0f);
    matrix_set(x, 1, 0, 0.0f);
    matrix_set(x, 2, 0, 0.0f);

    matrix_t* P = kalman_get_system_covariance(kf);
    for (uint_fast8_t r = 0; r < 3; r++) {
        for (uint_fast8_t c = 0; c < 3; c++) {
            matrix_set(P, r, c, (r == c) ? KF_INIT_VARIANCE : 0.0f);
        }
    }

    kf_altitude = 0.0f;
    kf_velocity = 0.0f;
    kf_accel = 0.0f;
    apogee_count = 0;
    boost_detected = false;
    flight_state = FLIGHT_IDLE;
    max_altitude = 0.0f;
}