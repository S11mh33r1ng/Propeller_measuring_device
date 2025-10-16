// #ifndef RPM_H
// #define RPM_H

// #include "Arduino.h"
// #include <Arduino_PortentaBreakout.h>

// #define RPM_UPDATE_INTERVAL 500  // 1000 millisecond update rate.
// #define MS_TO_M  60000.0

// #define HALL_SENSOR_1_PIN SAI_FS
// #define HALL_SENSOR_2_PIN SAI_D0

// void initRpmController(uint8_t propNumber);
// float updateRpm(uint8_t propNumber);

// #endif


// #ifndef RPM_H
// #define RPM_H

// #include "Arduino.h"
// #include <Arduino_PortentaBreakout.h>

// // Tuneables
// #define RPM_UPDATE_INTERVAL 1000    // update result every 1000 ms (1 s)
// #define MS_TO_M  60000.0
// #define PULSES_PER_REV  1.0         // change here if your disk/magnet differs
// #define MIN_PULSE_US    200         // ignore edges closer than this (glitch filter)

// // Pins
// #define HALL_SENSOR_1_PIN SAI_FS
// #define HALL_SENSOR_2_PIN SAI_D0

// void initRpmController(uint8_t propNumber);
// float updateRpm(uint8_t propNumber);

// #endif

#ifndef RPM_H
#define RPM_H

#include "Arduino.h"
#include <Arduino_PortentaBreakout.h>

// How often updateRpm() emits a new number (ms)
#define RPM_UPDATE_INTERVAL 200
#define MS_TO_M 60000.0

// ✅ Set this to match your hardware
// 1 magnet / 1 slot and counting one edge (FALLING or RISING only) -> 1.0
// 2 magnets / 2 slots (or counting both edges of 1 magnet) -> 2.0
#define PULSES_PER_REV 1.0

// Ignore edges that are too close together (µs) to kill ringing/EMI glitches.
// 300–600 µs is a good start for props in the few-thousand RPM range.
#define MIN_PULSE_US 800

#define PERIOD_SAMPLES 6
// When to switch from period→counting (pulses per update window)
#define HYBRID_COUNTING_MIN_PULSES  20

// Pins
#define HALL_SENSOR_1_PIN SAI_FS
#define HALL_SENSOR_2_PIN SAI_D0

void initRpmController(uint8_t propNumber);
float updateRpm(uint8_t propNumber);

#endif
