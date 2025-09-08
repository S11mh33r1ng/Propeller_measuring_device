#include "ThrustTorqueController.h"

static PinSet pinsFor(uint8_t prop) {
  if (prop == 0) {
    return PinSet{ THRUST_DAT_1_PIN, THRUST_CLK_1_PIN,
                   TORQUE_DAT_1_PIN, TORQUE_CLK_1_PIN };
  } else {
    return PinSet{ THRUST_DAT_2_PIN, THRUST_CLK_2_PIN,
                   TORQUE_DAT_2_PIN, TORQUE_CLK_2_PIN };
  }
}

ThrustTorqueController::ThrustTorqueController(uint8_t propellerId)
: _pins(pinsFor(propellerId)) {
  this->thrust = new HX711_ADC(_pins.thrDAT, _pins.thrSCK);
  this->torque = new HX711_ADC(_pins.trqDAT, _pins.trqSCK);
}


uint8_t ThrustTorqueController::begin(float thrustArmLength,
                                      float torqueArmLength,
                                      float thrustCalibration,
                                      float torqueCalibration) {
  // geometry must be valid
  if (thrustArmLength <= 0.0f || torqueArmLength <= 0.0f) return 1;

  // Always store arms first so status prints are truthful even if ADC init fails
  this->setThrustArmLength(thrustArmLength);
  this->setTorqueArmLength(torqueArmLength);

  // Safe cal factors (avoid div-by-zero)
  if (thrustCalibration != 0.0f) this->setThrustCalibration(thrustCalibration);
  else                           this->thrust->setCalFactor(this->thrust->getCalFactor() ? this->thrust->getCalFactor() : 1.0f);

  if (torqueCalibration != 0.0f) this->setTorqueCalibration(torqueCalibration);
  else                           this->torque->setCalFactor(this->torque->getCalFactor() ? this->torque->getCalFactor() : 1.0f);

  // (Re)create sensors with the correct pins for this prop
  if (!this->thrust) this->thrust = new HX711_ADC(_pins.thrDAT, _pins.thrSCK);
  if (!this->torque) this->torque = new HX711_ADC(_pins.trqDAT, _pins.trqSCK);

  this->thrust->begin();
  this->torque->begin();

  byte thrustReady = 0, torqueReady = 0;
  unsigned long t0 = millis();
  while ((thrustReady + torqueReady) < 2) {
    if (!thrustReady) thrustReady = this->thrust->startMultiple(STABILIZATION_TIME, TARE_ON_START);
    if (!torqueReady) torqueReady = this->torque->startMultiple(STABILIZATION_TIME, TARE_ON_START);
    if (millis() - t0 > 10000) break; // safety timeout
  }

  bool thrustTO = this->thrust->getTareTimeoutFlag();
  bool torqueTO = this->torque->getTareTimeoutFlag();

  if (thrustTO || torqueTO) {
    this->init = false;                // keep object, but mark not-ready
    return thrustTO ? 2 : 3;           // match your existing return codes
  }

  this->init = true;
  return 0;
}

float ThrustTorqueController::calibrateThrust(float knownMass) {
  this->thrust->refreshDataSet();
  float cal = this->thrust->getNewCalibration(knownMass);
  this->thrustCalibrated = true;
  return cal;
}

float ThrustTorqueController::calibrateTorque(float knownMass) {
  this->torque->refreshDataSet();
  float cal = this->torque->getNewCalibration(knownMass);
  this->torqueCalibrated = true;
  return cal;
}

void ThrustTorqueController::setThrustCalibration(float value) {
  this->thrust->setCalFactor(value);
  this->thrustCalibrated = true;
}
void ThrustTorqueController::setTorqueCalibration(float value) {
  this->torque->setCalFactor(value);
  this->torqueCalibrated = true;
}
void ThrustTorqueController::setThrustArmLength(float value) {
  this->thrustArmLength = value;
}
void ThrustTorqueController::setTorqueArmLength(float value) {
  this->torqueArmLength = value;
}
void ThrustTorqueController::tareAll() {
  this->thrust->tare();
  this->torque->tare();
}

float ThrustTorqueController::getThrustArmLength() const {
  return this->thrustArmLength;
}

float ThrustTorqueController::getTorqueArmLength() const {
  return this->torqueArmLength;
}

float ThrustTorqueController::getThrustCalibration() const {
  return this->thrust ? this->thrust->getCalFactor() : 0.0f;
}

float ThrustTorqueController::getTorqueCalibration() const {
  return this->torque ? this->torque->getCalFactor() : 0.0f;
}

void ThrustTorqueController::getThrust(float* data) {
  data[0] = updateThrust();                                // g
  data[1] = data[0] * G / this->thrustArmLength; // mN
}
void ThrustTorqueController::getTorque(float* data) {
  data[0] = updateTorque();                                // g
  data[1] = data[0] * G * (this->torqueArmLength / 1000.0f); // N·mm
}
float ThrustTorqueController::updateThrust() {
  this->thrust->update();
  return this->thrust->getData();
}
float ThrustTorqueController::updateTorque() {
  this->torque->update();
  return this->torque->getData();
}

bool ThrustTorqueController::getInit() { return this->init; }
bool ThrustTorqueController::getThrustCalibrated() { return this->thrustCalibrated; }
bool ThrustTorqueController::getTorqueCalibrated() { return this->torqueCalibrated; }
bool ThrustTorqueController::getCalibrated() { return this->thrustCalibrated && this->torqueCalibrated; }
