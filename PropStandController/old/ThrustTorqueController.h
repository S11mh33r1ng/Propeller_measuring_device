#ifndef THR_TRQ_H
#define THR_TRQ_H

#include "Arduino.h"
#include "Arduino_PortentaBreakout.h"
#include "HX711_ADC.h"

#define G 9.8066520482

#define STABILIZATION_TIME 2000
#define TARE_ON_START true

#define THRUST_CLK_1_PIN UART1_TX
#define THRUST_DAT_1_PIN UART1_RX 
#define TORQUE_CLK_1_PIN UART0_TX 
#define TORQUE_DAT_1_PIN UART0_RX 

#define THRUST_CLK_2_PIN GPIO_4
#define THRUST_DAT_2_PIN GPIO_6
#define TORQUE_CLK_2_PIN GPIO_0
#define TORQUE_DAT_2_PIN GPIO_2


class ThrustTorqueController {
  private:
    //struct PinSet { uint8_t dout; uint8_t sck; };

    // Resolve pin sets by propellerId (0/1). Bounds-checked in ctor.
    //static PinSet thrustPinsFor(uint8_t propId);
    //static PinSet torquePinsFor(uint8_t propId);

    HX711_ADC *thrust, *torque;
    float thrustArmLength, torqueArmLength;

    bool init, thrustCalibrated, torqueCalibrated;
    
    float updateThrust();
    float updateTorque();
    
  public:
    ThrustTorqueController(uint8_t propellerId);
    uint8_t begin(float thrustArmLength, float torqueArmLength, float thrustCalibration, float torqueCalibration);
    float calibrateThrust(float knownMass);
    float calibrateTorque(float knownMass);
    void setThrustCalibration(float value);
    void setTorqueCalibration(float value);
    void setThrustArmLength(float value);
    void setTorqueArmLength(float value);
    void tareAll();
    void getThrust(float* data);
    void getTorque(float* data);

    bool getInit();
    bool getThrustCalibrated();
    bool getTorqueCalibrated();
    bool getCalibrated();
};

#endif
