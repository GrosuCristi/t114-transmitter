#ifndef NRFX_CONFIG_H__
#define NRFX_CONFIG_H__

#define NRFX_POWER_ENABLED              1
#define NRFX_POWER_DEFAULT_CONFIG_IRQ_PRIORITY  7

#define NRFX_CLOCK_ENABLED 0

#define NRFX_SPIM_ENABLED            1
#define NRFX_SPIM_MISO_PULL_CFG      1 // pulldown
#define NRFX_SPIM_EXTENDED_ENABLED   0

#define NRFX_SPIM0_ENABLED           0 // used as I2C
#define NRFX_SPIM1_ENABLED           0 // used as I2C
#define NRFX_SPIM2_ENABLED           1

#define NRFX_SPIS_ENABLED 0
#define NRFX_SPIS_DEFAULT_CONFIG_IRQ_PRIORITY 7
#define NRFX_SPIS_CONFIG_LOG_ENABLED 0
#define NRFX_SPIS_CONFIG_LOG_LEVEL 3
#define NRFX_SPIS_CONFIG_INFO_COLOR 0
#define NRFX_SPIS_CONFIG_DEBUG_COLOR 0

#define NRFX_SPIS0_ENABLED 0
#define NRFX_SPIS1_ENABLED 0
#define NRFX_SPIS2_ENABLED 0

#define NRFX_PWM_ENABLED 0
#define NRFX_PWM0_ENABLED 0
#define NRFX_PWM1_ENABLED 0
#define NRFX_PWM2_ENABLED 0
#define NRFX_PWM3_ENABLED 0

// TIMER: locally enabled for application use.
// TIMER0 is reserved by the SoftDevice; TIMER2 is often used by the Adafruit
// core for PWM/Tone, so keep them disabled unless you know what you're doing.
#define NRFX_TIMER_ENABLED 1
#define NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY 6
#define NRFX_TIMER0_ENABLED 0
#define NRFX_TIMER1_ENABLED 1
#define NRFX_TIMER2_ENABLED 0
#define NRFX_TIMER3_ENABLED 0
#define NRFX_TIMER4_ENABLED 0

#if defined(NRF52840_XXAA)
  #define NRFX_QSPI_ENABLED   1
  #define NRFX_SPIM3_ENABLED  1
#elif defined(NRF52833_XXAA)
  #define NRFX_QSPI_ENABLED   0
  #define NRFX_SPIM3_ENABLED  1
#else
  #define NRFX_QSPI_ENABLED   0
  #define NRFX_SPIM3_ENABLED  0
#endif

// NRFX temp
#define NRFX_TEMP_ENABLED 1
#define NRFX_TEMP_DEFAULT_CONFIG_IRQ_PRIORITY 7

#endif // NRFX_CONFIG_H__
