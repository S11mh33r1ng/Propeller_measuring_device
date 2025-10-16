// #include "RpmController.h"

// const int sensorPins[] = {HALL_SENSOR_1_PIN, HALL_SENSOR_2_PIN};

// volatile uint32_t pulseCounts[] = {0, 0};
// uint32_t lastMillis[] = {0, 0};
// float rpms[] = {0, 0};

// void hallInterrupt0() {
//   pulseCounts[0]++;
// }

// void hallInterrupt1() {
//   pulseCounts[1]++;
// }

// void initRpmController(uint8_t propNumber) {
//   pinMode(sensorPins[propNumber], INPUT);
//   if (propNumber)
//     attachInterrupt(digitalPinToInterrupt(sensorPins[propNumber]), hallInterrupt1, FALLING);
//   else
//     attachInterrupt(digitalPinToInterrupt(sensorPins[propNumber]), hallInterrupt0, FALLING);
// }

// float updateRpm(uint8_t propNumber) {
//   uint32_t currentMillis = millis();
//   if (currentMillis - lastMillis[propNumber] > RPM_UPDATE_INTERVAL) {
//     noInterrupts();
//     uint32_t pulses = pulseCounts[propNumber];
//     pulseCounts[propNumber] = 0;
//     interrupts();

//     rpms[propNumber] = (pulses / 2.0) / ((currentMillis - lastMillis[propNumber]) / MS_TO_M);
//     lastMillis[propNumber] = currentMillis;
//   }
//   return rpms[propNumber];
// }


// #include "RpmController.h"

// const int sensorPins[] = {HALL_SENSOR_1_PIN, HALL_SENSOR_2_PIN};

// volatile uint32_t pulseCounts[]   = {0, 0};
// volatile uint32_t lastEdgeUs[]    = {0, 0};  // for de-glitching in ISR

// uint32_t lastMillis[]             = {0, 0};
// float rpms[]                      = {0.0f, 0.0f};  // smoothed output

// // --- ISRs ---
// static inline void countEdge(uint8_t i) {
//   uint32_t now = micros();
//   // simple de-glitch: ignore edges too close together
//   if ((now - lastEdgeUs[i]) >= MIN_PULSE_US) {
//     pulseCounts[i]++;
//     lastEdgeUs[i] = now;
//   }
// }

// void hallInterrupt0() { countEdge(0); }
// void hallInterrupt1() { countEdge(1); }

// // --- Setup ---
// void initRpmController(uint8_t propNumber) {
//   // Use pull-up for open-collector style Hall sensors; adjust if you have a push-pull sensor
//   pinMode(sensorPins[propNumber], INPUT_PULLUP);

//   if (propNumber)
//     attachInterrupt(digitalPinToInterrupt(sensorPins[propNumber]), hallInterrupt1, FALLING);
//   else
//     attachInterrupt(digitalPinToInterrupt(sensorPins[propNumber]), hallInterrupt0, FALLING);

//   lastMillis[propNumber] = millis();
//   lastEdgeUs[propNumber] = micros();
// }

// // --- Update ---
// // Counts pulses seen in the last RPM_UPDATE_INTERVAL ms, converts to RPM,
// // then applies an exponential smoothing to stabilize the output.
// float updateRpm(uint8_t propNumber) {
//   const float alpha = 0.9f;  // smoothing factor: 0..1 (higher = more responsive)
//   uint32_t currentMillis = millis();
//   uint32_t dt_ms = currentMillis - lastMillis[propNumber];

//   if (dt_ms >= RPM_UPDATE_INTERVAL) {
//     noInterrupts();
//     uint32_t pulses = pulseCounts[propNumber];
//     pulseCounts[propNumber] = 0;
//     interrupts();

//     // Instantaneous estimate for this window
//     // RPM = (pulses / PULSES_PER_REV) / (dt_ms / 60000)
//     float instant = (pulses * (MS_TO_M / PULSES_PER_REV)) / (float)dt_ms;

//     // Exponential smoothing
//     rpms[propNumber] = alpha * instant + (1.0f - alpha) * rpms[propNumber];

//     lastMillis[propNumber] = currentMillis;
//   }
//   return rpms[propNumber];
// }

// #include "RpmController.h"

// static const int sensorPins[] = { HALL_SENSOR_1_PIN, HALL_SENSOR_2_PIN };

// volatile uint32_t pulseCounts[2] = {0, 0};
// volatile uint32_t lastEdgeUs[2]  = {0, 0};

// static uint32_t lastMillis[2] = {0, 0};
// static float rpms[2]          = {0.0f, 0.0f};

// // --- ISR helper: count only plausible edges (de-glitch) ---
// static inline void countEdge(uint8_t i) {
//   const uint32_t now = micros();
//   if ((now - lastEdgeUs[i]) >= MIN_PULSE_US) {
//     pulseCounts[i]++;
//     lastEdgeUs[i] = now;
//   }
// }

// // ISRs
// static void hallInterrupt0() { countEdge(0); }
// static void hallInterrupt1() { countEdge(1); }

// // --- Setup for one channel ---
// void initRpmController(uint8_t propNumber) {
//   const int pin = sensorPins[propNumber];

//   // Most Hall sensors are open-collector/open-drain → need pull-up
//   pinMode(pin, INPUT_PULLUP);

//   // Count ONE edge per magnet/slot; adjust PULSES_PER_REV if you change this
//   void (*isr)() = (propNumber == 0) ? hallInterrupt0 : hallInterrupt1;
//   attachInterrupt(digitalPinToInterrupt(pin), isr, FALLING);

//   lastMillis[propNumber] = millis();
//   lastEdgeUs[propNumber] = micros();
//   rpms[propNumber] = 0.0f;
// }

// // --- Periodic update ---
// // Returns the most recent RPM; computes a fresh value every RPM_UPDATE_INTERVAL ms.
// float updateRpm(uint8_t propNumber) {
//   const uint32_t nowMs = millis();
//   const uint32_t dtMs  = nowMs - lastMillis[propNumber];

//   if (dtMs >= RPM_UPDATE_INTERVAL) {
//     noInterrupts();
//     const uint32_t pulses = pulseCounts[propNumber];
//     pulseCounts[propNumber] = 0;
//     interrupts();

//     // RPM = (pulses / PULSES_PER_REV) / (dtMs / 60000)
//     //     = pulses * (60000 / (PULSES_PER_REV * dtMs))
//     const float rpm = (pulses * (MS_TO_M / (PULSES_PER_REV * (float)dtMs)));

//     rpms[propNumber] = rpm;
//     lastMillis[propNumber] = nowMs;
//   }
//   return rpms[propNumber];
// }

#include "RpmController.h"

static const int sensorPins[] = { HALL_SENSOR_1_PIN, HALL_SENSOR_2_PIN };

volatile uint32_t pulseCounts[2]   = {0, 0};
volatile uint32_t lastEdgeUs[2]    = {0, 0};

// Period estimator: store the last N intervals per channel
volatile uint32_t intervalUs[2][PERIOD_SAMPLES];
volatile uint8_t  intervalIdx[2]   = {0, 0};
volatile bool     intervalFilled[2]= {false, false};

static uint32_t lastMillis[2]      = {0, 0};
static float    rpms[2]            = {0.0f, 0.0f};

static inline void countEdge(uint8_t i) {
  const uint32_t now = micros();
  const uint32_t dt  = now - lastEdgeUs[i];

  // De-glitch
  if (dt >= MIN_PULSE_US) {
    pulseCounts[i]++;

    // Save interval for period-based estimate
    if (lastEdgeUs[i] != 0) {
      intervalUs[i][intervalIdx[i]] = dt;
      intervalIdx[i] = (intervalIdx[i] + 1) % PERIOD_SAMPLES;
      if (intervalIdx[i] == 0) intervalFilled[i] = true;
    }
    lastEdgeUs[i] = now;
  }
}

// ISRs
static void hallInterrupt0() { countEdge(0); }
static void hallInterrupt1() { countEdge(1); }

void initRpmController(uint8_t propNumber) {
  const int pin = sensorPins[propNumber];
  pinMode(pin, INPUT_PULLUP);                        // clean logic high
  void (*isr)() = (propNumber == 0) ? hallInterrupt0 : hallInterrupt1;
  attachInterrupt(digitalPinToInterrupt(pin), isr, FALLING); // one edge/rev unless PPR > 1

  lastMillis[propNumber] = millis();
  lastEdgeUs[propNumber] = 0;

  // clear period buffers
  for (int k = 0; k < PERIOD_SAMPLES; ++k) intervalUs[propNumber][k] = 0;
  intervalIdx[propNumber] = 0;
  intervalFilled[propNumber] = false;
  rpms[propNumber] = 0.0f;
}

// Compute period-based RPM from averaged interval (us). Returns <0 if not enough data.
static float rpmFromPeriod(uint8_t i) {
  uint32_t sum = 0;
  uint8_t  n   = 0;

  // make a local snapshot (interrupt-safe read)
  noInterrupts();
  const bool filled = intervalFilled[i];
  const uint8_t idx = intervalIdx[i];
  uint32_t buf[PERIOD_SAMPLES];
  for (int k = 0; k < PERIOD_SAMPLES; ++k) buf[k] = intervalUs[i][k];
  interrupts();

  // If not full yet, average only valid entries
  const uint8_t count = filled ? PERIOD_SAMPLES : idx;
  if (count < 2) return -1.0f; // need at least one full interval

  for (uint8_t k = 0; k < count; ++k) {
    const uint32_t v = buf[k];
    if (v > 0) { sum += v; n++; }
  }
  if (n < 1) return -1.0f;

  const float avg_us = (float)sum / (float)n;
  // RPM = 60e6 / (PPR * avg_interval_us)
  return (60.0e6f) / (PULSES_PER_REV * avg_us);
}

float updateRpm(uint8_t propNumber) {
  const uint32_t nowMs = millis();
  const uint32_t dtMs  = nowMs - lastMillis[propNumber];

  if (dtMs >= RPM_UPDATE_INTERVAL) {
    // Snapshot and clear pulse count for counting estimator
    noInterrupts();
    uint32_t pulses = pulseCounts[propNumber];
    pulseCounts[propNumber] = 0;
    interrupts();

    // Counting-based RPM over the window
    // RPM = pulses * (60000 / (PPR * dtMs))
    const float rpm_count = (pulses > 0)
      ? (pulses * (60000.0f / (PULSES_PER_REV * (float)dtMs)))
      : 0.0f;

    // Period-based RPM (better at low speeds)
    const float rpm_period = rpmFromPeriod(propNumber);

    // Hybrid selection:
    // - if we saw only a few pulses in this window, prefer period-based;
    // - otherwise, counting-based is robust and low-noise.
    float rpm_est;
    if (pulses < HYBRID_COUNTING_MIN_PULSES && rpm_period > 0.0f) {
      rpm_est = rpm_period;
    } else {
      rpm_est = rpm_count;
    }

    // Light smoothing to avoid display jitter (adjust 0.2..0.5)
    const float alpha = 0.5f;
    rpms[propNumber] = alpha * rpm_est + (1.0f - alpha) * rpms[propNumber];

    lastMillis[propNumber] = nowMs;
  }
  return rpms[propNumber];
}

