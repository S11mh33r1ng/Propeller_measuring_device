#include "ThrustTorqueController.h"

//HX711_ADC thrustADCs[] = {HX711_ADC(THRUST_DAT_1_PIN, THRUST_CLK_1_PIN), HX711_ADC(THRUST_DAT_2_PIN, THRUST_CLK_2_PIN)};
//HX711_ADC torqueADCs[] = {HX711_ADC(TORQUE_DAT_1_PIN, TORQUE_CLK_1_PIN), HX711_ADC(TORQUE_DAT_2_PIN, TORQUE_CLK_2_PIN)};

ThrustTorqueController::ThrustTorqueController(uint8_t propellerId) {
  this->thrust = new HX711_ADC(THRUST_DAT_1_PIN, THRUST_CLK_1_PIN);
  this->torque = new HX711_ADC(TORQUE_DAT_1_PIN, TORQUE_CLK_1_PIN);
}

// --- Static helpers to map propellerId -> pins --- 
/*ThrustTorqueController::PinSet ThrustTorqueController::thrustPinsFor(uint8_t propId) {
  switch (propId) {
    case 0: default: return { THRUST_DAT_1_PIN, THRUST_CLK_1_PIN };
    case 1:          return { THRUST_DAT_2_PIN, THRUST_CLK_2_PIN };
  }
}
ThrustTorqueController::PinSet ThrustTorqueController::torquePinsFor(uint8_t propId) {
  switch (propId) {
    case 0: default: return { TORQUE_DAT_1_PIN, TORQUE_CLK_1_PIN };
    case 1:          return { TORQUE_DAT_2_PIN, TORQUE_CLK_2_PIN };
  }
}

// --- Ctor ---
ThrustTorqueController::ThrustTorqueController(uint8_t propellerId) {
  PinSet thr = thrustPinsFor(propellerId);
  PinSet trq = torquePinsFor(propellerId);
  this->thrust = new HX711_ADC(thr.dout, thr.sck);
  this->torque = new HX711_ADC(trq.dout, trq.sck);
}*/


uint8_t ThrustTorqueController::begin(float thrustArmLength, float torqueArmLength, float thrustCalibration, float torqueCalibration) {
  if (thrustArmLength == 0.0 || torqueArmLength == 0.0 || thrustCalibration == 0.0 || torqueCalibration == 0.0 )
    return 1;
  this->thrust->begin();
  this->torque->begin();

  byte thrustReady = 0;
  byte torqueReady = 0;
  while ((thrustReady + torqueReady) < 2) {
    if (!thrustReady) thrustReady = this->thrust->startMultiple(STABILIZATION_TIME, TARE_ON_START);
    if (!torqueReady) torqueReady = this->torque->startMultiple(STABILIZATION_TIME, TARE_ON_START);
  }
  
  if (this->thrust->getTareTimeoutFlag()) {
    return 2;
  }
  if (this->torque->getTareTimeoutFlag()) {
    return 3;
  }
  
  this->setThrustArmLength(thrustArmLength);
  this->setTorqueArmLength(torqueArmLength);
  this->setThrustCalibration(thrustCalibration);
  this->setTorqueCalibration(torqueCalibration);

  this->init = true;
  return 0;
}

float ThrustTorqueController::calibrateThrust(float knownMass) {
  this->thrust->refreshDataSet();
  this->thrust->getNewCalibration(knownMass);
  this->thrustCalibrated = true;
}

float ThrustTorqueController::calibrateTorque(float knownMass) {
  this->torque->refreshDataSet();
  this->torque->getNewCalibration(knownMass);
  this->torqueCalibrated = true;
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

void ThrustTorqueController::getThrust(float* data) {
  data[0] = updateThrust();
  data[1] = data[0] * G * this->thrustArmLength / 1000.0;
}

void ThrustTorqueController::getTorque(float* data) {
  data[0] = updateTorque();
  data[1] = data[0] * G * this->torqueArmLength / 1000.0;
}

float ThrustTorqueController::updateThrust() {
  this->thrust->update();
  return this->thrust->getData();
}

float ThrustTorqueController::updateTorque() {
  this->torque->update();
  return this->torque->getData();
}

bool ThrustTorqueController::getInit() {
  return this->init;
}

bool ThrustTorqueController::getThrustCalibrated() {
  return this->thrustCalibrated;
}

bool ThrustTorqueController::getTorqueCalibrated() {
  return this->torqueCalibrated;
}

bool ThrustTorqueController::getCalibrated() {
  return this->thrustCalibrated && this->torqueCalibrated;
}
