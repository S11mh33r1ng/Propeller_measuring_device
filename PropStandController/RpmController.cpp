#include "RpmController.h"

const int sensorPins[] = {HALL_SENSOR_1_PIN, HALL_SENSOR_2_PIN};

volatile uint32_t pulseCounts[] = {0, 0};
uint32_t lastMillis[] = {0, 0};
float rpms[] = {0, 0};

void hallInterrupt0() {
  pulseCounts[0]++;
}

void hallInterrupt1() {
  pulseCounts[1]++;
}

void initRpmController(uint8_t propNumber) {
  pinMode(sensorPins[propNumber], INPUT);
  if (propNumber)
    attachInterrupt(digitalPinToInterrupt(sensorPins[propNumber]), hallInterrupt1, FALLING);
  else
    attachInterrupt(digitalPinToInterrupt(sensorPins[propNumber]), hallInterrupt0, FALLING);
}

float updateRpm(uint8_t propNumber) {
  uint32_t currentMillis = millis();
  if (currentMillis - lastMillis[propNumber] > RPM_UPDATE_INTERVAL) {
    noInterrupts();
    uint32_t pulses = pulseCounts[propNumber];
    pulseCounts[propNumber] = 0;
    interrupts();

    rpms[propNumber] = (pulses / 2.0) / ((currentMillis - lastMillis[propNumber]) / MS_TO_M);
    lastMillis[propNumber] = currentMillis;
  }
  return rpms[propNumber];
}
