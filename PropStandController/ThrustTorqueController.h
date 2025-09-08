#ifndef THR_TRQ_H
#define THR_TRQ_H

#include "Arduino.h"
#include "Arduino_PortentaBreakout.h"
#include "HX711_ADC.h"

// Physical constants
#define G 9.8066520482f

// HX711 startup behavior
#define STABILIZATION_TIME 3000
#define TARE_ON_START true

// === PIN DEFINITIONS (from working multiple_adc_test) ===
// Prop #0 pins
#ifndef THRUST_CLK_1_PIN
#define THRUST_CLK_1_PIN UART1_TX
#endif
#ifndef THRUST_DAT_1_PIN
#define THRUST_DAT_1_PIN UART1_RX
#endif
#ifndef TORQUE_CLK_1_PIN
#define TORQUE_CLK_1_PIN GPIO_4
#endif
#ifndef TORQUE_DAT_1_PIN
#define TORQUE_DAT_1_PIN GPIO_6
#endif

// Prop #1 pins
#ifndef THRUST_CLK_2_PIN
#define THRUST_CLK_2_PIN UART0_TX
#endif
#ifndef THRUST_DAT_2_PIN
#define THRUST_DAT_2_PIN UART0_RX
#endif
#ifndef TORQUE_CLK_2_PIN
#define TORQUE_CLK_2_PIN GPIO_0
#endif
#ifndef TORQUE_DAT_2_PIN
#define TORQUE_DAT_2_PIN GPIO_2
#endif

struct PinSet { uint8_t thrDAT, thrSCK, trqDAT, trqSCK; };

class ThrustTorqueController {
public:
  explicit ThrustTorqueController(uint8_t propellerId);

  uint8_t begin(float thrustArmLength, float torqueArmLength,
                float thrustCalibration, float torqueCalibration);

  float calibrateThrust(float knownMass);
  float calibrateTorque(float knownMass);

  void setThrustCalibration(float value);
  void setTorqueCalibration(float value);
  void setThrustArmLength(float value);
  void setTorqueArmLength(float value);
  void tareAll();

  void getThrust(float* data);
  void getTorque(float* data);

  float updateThrust();
  float updateTorque();

  bool getInit();
  bool getThrustCalibrated();
  bool getTorqueCalibrated();
  bool getCalibrated();

  float getThrustArmLength() const;
  float getTorqueArmLength() const;

  // Read back current calibration factors from HX711_ADC
  float getThrustCalibration() const;
  float getTorqueCalibration() const;

  PinSet pins() const { return _pins; }
  bool thrustTareTimeout() const { return this->thrust ? this->thrust->getTareTimeoutFlag() : true; }
  bool torqueTareTimeout() const { return this->torque ? this->torque->getTareTimeoutFlag() : true; }

private:

  PinSet _pins;

  HX711_ADC* thrust = nullptr;
  HX711_ADC* torque = nullptr;

  float thrustArmLength = 0.0f; // mm
  float torqueArmLength = 0.0f; // mm
  bool init = false;
  bool thrustCalibrated = false;
  bool torqueCalibrated = false;
};

#endif
