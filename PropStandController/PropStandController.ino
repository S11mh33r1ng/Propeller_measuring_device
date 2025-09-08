#include <rtos.h>
#include <mbed.h>
#include <Servo.h>
#include <Thread.h>
#include <ThreadController.h>
#include "RpmController.h"
#include "ThrustTorqueController.h"

#define TIMEOUT_MS 90000 // Timeout in milliseconds
#define DEBOUNCE_DELAY 50 // Debounce time in milliseconds
#define EMERGENCY_ACTIVE_LOW 0
#define SAFETY_OPEN_ACTIVE_HIGH 0

#define safety_disconnected A2
#define emergency_pressed D0 //PWM 6
#define first_esc D1 //PWM 5
#define second_esc D2 //PWM 4
#define emergency_stepper D4 //PWM 2
#define beacon_output D3 //PWM 3
#define aoa_out D6 //PWM 0
#define aoss_out D5 //PWM 1

// Define button states
enum ButtonState {
    BUTTON_UP,
    BUTTON_DOWN,
    BUTTON_FALLING,
    BUTTON_RISING
};

rtos::Mutex pitotMutex;
rtos::Mutex thrtrqMutex;
rtos::Mutex rpmMutex;

ThreadController controller = ThreadController();
ThrustTorqueController* thrustTorqueControllers[2] = { nullptr, nullptr };

Thread thread1 = Thread();
Thread* thread2 = new Thread();
Thread* thread3 = new Thread();
Thread* thread4 = new Thread();
Thread* thread5 = new Thread();
Thread* thread6 = new Thread();
Thread* thread7 = new Thread();
Thread* thread8 = new Thread();
Thread* thread9 = new Thread();

Servo aoaServo; 
Servo aossServo;
Servo esc1;
Servo esc2;

ButtonState safetySwitchState = BUTTON_UP;      // INPUT_PULLUP -> idle = HIGH
ButtonState emergencyButtonState = BUTTON_UP;   // idle/unpressed

bool limits_sent = false;
bool read_stream = false;
bool mass_sent = false;
bool cal_value_received = false;
bool init_message = false;
bool input_complete = false;
bool get_pitot = false;
bool cal_found = false;
volatile bool measure_in_progress = false;
bool homing_done = false;
bool no_safety = false;
bool rpm_test = false;
bool got_pitot_readings = false;
bool tare_done = false;
bool beacon_on = false;
bool aoss_enabled = true;
bool twoProps = false;
bool read_stream_test  = false;
bool read_stream_test_second = false;
bool read_stream_test_both   = false;
volatile bool getThrTrqRPM = false;
volatile bool emergency_state = false;
volatile bool emergency_cleared = false;
volatile float weight;
int X_pos;
int Y_pos;
int init_pos = 0;
int stepper_address = 10;
int thrtrq_address = 5;
int rpm_address = 15;
int reqX;
int reqY;
int trimAoA = 79;
int trimAoSS = 105;
int limAoA = 50;
int limMaxAoSS = 56;
int limMinAoSS = 108;
volatile float airspeed_raw;
volatile float aoa_raw;
volatile float aoss_raw;
float receivedValue = 0.0;
float absAoA;
float absAoSS;
float firstTrqArmLength;
float firstTrqCalVal;
float trqCalMass;
float firstThrArmLength; 
float secondTrqArmLength;
float secondTrqCalVal;
float secondThrArmLength;
float firstThrCalVal;
float secondThrCalVal;
int tandemSetup;
float thrCalMass;
float aoaSensorPhysicalAngle = trimAoA; 
float aossSensorPhysicalAngle = trimAoSS;
const float maxAoA = 21.0;            // Maximum AoA without adjustment
const float minAoA = -21.0;           // Minimum AoA without adjustment
const float maxAoSS = 21.0;            // Maximum AoSS without adjustment
const float minAoSS = -21.0;           // Minimum AoSS without adjustment
const float adjustmentStepAoA = 2.0;
const float adjustmentStepAoSS = 2.0;
const float AoSSratio = 1.419;          //ratio between the degree command given and actual degrees (i.e. command is 44 deg, but actual movement of the probe is 31 deg)
unsigned long lastSafetySwitchTime = 0;
unsigned long lastEmergencyButtonTime = 0;
const int ESC_MIN_US = 1000;
const int ESC_MAX_US = 2000;
volatile int escRampMs = 150;     // default ramp time in ms
volatile int escTarget1 = ESC_MIN_US;
volatile int escTarget2 = ESC_MIN_US;
volatile int escCurrent1 = ESC_MIN_US;
volatile int escCurrent2 = ESC_MIN_US;
volatile bool escEnabled1 = false;
volatile bool escEnabled2 = false;
volatile int escMinUs1 = ESC_MIN_US;
volatile int escMaxUs1 = ESC_MAX_US;
volatile int escMinUs2 = ESC_MIN_US;
volatile int escMaxUs2 = ESC_MAX_US;

void manageCommands();
void readPitot();
void aoaController();
void aossController();
void assembleStream();
void checkEmergency();
void beaconController();
void forwardToStepper(String data);
void testReadThrTrq();
void escPwmController();
void escHardStopBoth();

struct InitParams {
  float fTrqArm, fTrqCal, fThrArm, fThrCal;
  float sTrqArm, sTrqCal, sThrArm, sThrCal;
  int   tSetup;
};

static bool parseInitPayload(const String& payloadIn, InitParams &p) {
  String payload = payloadIn;
  payload.trim(); // kill trailing CR/LF/space

  float v[8];
  int start = 0, bar = -1;
  for (int i = 0; i < 8; ++i) {
    bar = payload.indexOf('|', start);
    if (bar < 0) {
      Serial.println(F("ERR|init: not enough '|' for 8 float fields"));
      return false;
    }
    String tok = payload.substring(start, bar);
    tok.trim();
    v[i] = tok.toFloat();          // empty -> 0.0 (we'll detect below)
    start = bar + 1;
  }
  // tail is tandemSetup
  String tail = payload.substring(start);
  tail.trim();
  if (tail.length() == 0) {
    Serial.println(F("ERR|init: missing tandemSetup at end"));
    return false;
  }
  p.tSetup = tail.toInt();

  p.fTrqArm = v[0]; p.fTrqCal = v[1]; p.fThrArm = v[2]; p.fThrCal = v[3];
  p.sTrqArm = v[4]; p.sTrqCal = v[5]; p.sThrArm = v[6]; p.sThrCal = v[7];

  // Debug what we parsed (one time)
  Serial.print(F("PARSED fTrqArm=")); Serial.print(p.fTrqArm);
  Serial.print(F(" fThrArm="));       Serial.print(p.fThrArm);
  Serial.print(F(" sTrqArm="));       Serial.print(p.sTrqArm);
  Serial.print(F(" sThrArm="));       Serial.print(p.sThrArm);
  Serial.print(F(" tSetup="));        Serial.println(p.tSetup);

  return true;
}

static void initControllersFrom(const InitParams &p) {
  // 1) Cache everything for later use (props|2, status, etc.)
  firstTrqArmLength   = p.fTrqArm;
  firstThrArmLength   = p.fThrArm;
  firstTrqCalVal      = p.fTrqCal;
  firstThrCalVal      = p.fThrCal;

  secondTrqArmLength  = p.sTrqArm;
  secondThrArmLength  = p.sThrArm;
  secondTrqCalVal     = p.sTrqCal;
  secondThrCalVal     = p.sThrCal;

  // 2) Always init prop #0 from cached values
  if (!thrustTorqueControllers[0]) thrustTorqueControllers[0] = new ThrustTorqueController(0);
  uint8_t rc0 = thrustTorqueControllers[0]->begin(
      firstThrArmLength, firstTrqArmLength,
      firstThrCalVal,    firstTrqCalVal
  );
  Serial.print(F("BEGIN0 rc=")); Serial.println(rc0);

  // 3) Decide whether to *start* second controller now
  bool wantSecondNow = (p.tSetup == 2);

  if (wantSecondNow) {
    if (!thrustTorqueControllers[1]) {
      thrustTorqueControllers[1] = new ThrustTorqueController(1);
    }
    uint8_t rc1 = thrustTorqueControllers[1]->begin(
        secondThrArmLength, secondTrqArmLength, secondThrCalVal, secondTrqCalVal);


    Serial.print(F("BEGIN1 rc=")); Serial.println(rc1);
    twoProps = true;
  }
  Serial.println(F("Ready!"));
  // 4) RPM init(s)
  initRpmController(0);
  if (twoProps && thrustTorqueControllers[1]) initRpmController(1);
  //Serial.println(F("Ready!"));
}

bool enableSecondProp() {
  if (twoProps) return true; // already on

  if (secondThrArmLength <= 0.0f || secondTrqArmLength <= 0.0f) {
    Serial.println(F("ERR|second_arms_missing"));
    return false;
  }

  float thrCF = (secondThrCalVal != 0.0f) ? secondThrCalVal : 1.0f;
  float trqCF = (secondTrqCalVal != 0.0f) ? secondTrqCalVal : 1.0f;

  thrtrqMutex.lock();
  if (!thrustTorqueControllers[1]) {
    thrustTorqueControllers[1] = new ThrustTorqueController(1);
  }
  uint8_t rc = thrustTorqueControllers[1]->begin(
      secondThrArmLength, secondTrqArmLength,
      thrCF,              trqCF
  );
  thrtrqMutex.unlock();

  if (rc != 0) {
    Serial.print(F("ERR|begin1_rc=")); Serial.println(rc);
    thrtrqMutex.lock();
    delete thrustTorqueControllers[1];
    thrustTorqueControllers[1] = nullptr;
    thrtrqMutex.unlock();
    return false;
  }

  initRpmController(1);
  twoProps = true;
  return true;
}

// Disable controller[1] safely.
void disableSecondProp() {
  read_stream_test_second = false;
  read_stream_test_both   = false;

  thrtrqMutex.lock();
  ThrustTorqueController* p = thrustTorqueControllers[1];
  thrustTorqueControllers[1] = nullptr;   // make readers instantly see “gone”
  thrtrqMutex.unlock();

  if (p) delete p; // free after releasing the lock
  twoProps = false;
}

// motorIdx: 0 = first, 1 = second
static int parsePwmTarget(const String& s, int motorIdx) {
  long v = s.toInt();

  int minUs = (motorIdx == 0) ? escMinUs1 : escMinUs2;
  int maxUs = (motorIdx == 0) ? escMaxUs1 : escMaxUs2;
  if (maxUs < minUs) { int t = minUs; minUs = maxUs; maxUs = t; }

  long target = (v <= 100) ? (minUs + (v * (long)(maxUs - minUs)) / 100L) : v;

  if (target < minUs) target = minUs;
  if (target > maxUs) target = maxUs;
  return (int)target;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(safety_disconnected, INPUT_PULLUP);
  pinMode(emergency_pressed, INPUT_PULLUP);
  pinMode(emergency_stepper, OUTPUT);
  pinMode(beacon_output, OUTPUT);

  aoaServo.attach(aoa_out); //PWM 0
  aossServo.attach(aoss_out); //PWM 1
  aoaServo.write(trimAoA);  // Set servo1 to 90-degree position
  aossServo.write(trimAoSS);  // Set servo2 to 45-degree position
  esc1.attach(first_esc);
  esc1.writeMicroseconds(ESC_MIN_US);
  esc2.attach(second_esc);
  esc2.writeMicroseconds(ESC_MIN_US);
  
  Serial.begin(115200); //Serial between UI
  Serial3.begin(115200); //Serial between Pitot' sensor
  Wire.begin(); //I2C to stepper controller

  thrustTorqueControllers[0] = new ThrustTorqueController(0);

  thread1.onRun(manageCommands);
  thread1.setInterval(0); 
  thread2->onRun(testReadThrTrq);
  thread2->setInterval(100); 
  thread3->onRun(readPitot); 
  thread3->setInterval(0); 
  thread4->onRun(aoaController); 
  thread4->setInterval(0); 
  thread5->onRun(assembleStream); 
  thread5->setInterval(5); //changing this value changes the resolution of the readings, don't set it to 0! 5 default
  thread6->onRun(checkEmergency); 
  thread6->setInterval(10);
  thread7->onRun(escPwmController); 
  thread7->setInterval(10);
  thread8->onRun(aossController); 
  thread8->setInterval(0);
  thread9->onRun(beaconController); 
  thread9->setInterval(0);
  
  // Add threads to controller
  controller.add(&thread1);
  controller.add(thread2);
  controller.add(thread3);
  controller.add(thread4);
  controller.add(thread5);
  controller.add(thread6);
  controller.add(thread7);
  controller.add(thread8);
  controller.add(thread9);

  while (input_complete == false && no_safety == false) {
    digitalWrite(LED_BUILTIN, HIGH); 
    delay(50);                      
    digitalWrite(LED_BUILTIN, LOW);   
    delay(50); 
    String input = Serial.readStringUntil('\n');
    String originalInput = input;
    if (input.startsWith("init|")) {
      InitParams p;
      if (!parseInitPayload(input.substring(5), p)) {
        // keep blinking, wait for a valid init
        continue;
        }
      initControllersFrom(p);
      input_complete = true;
      cal_found = true;
    }
  }
}

void readPitot() {
  got_pitot_readings = false;
  String pitot = Serial3.readStringUntil('\n'); 
  String airspeedStr, aoaStr, aossStr;

  int time = pitot.indexOf(',');
  int date = pitot.indexOf(',', time + 1);
  int as = pitot.indexOf(',', date + 1);
  int ias = pitot.indexOf(',', as + 1);
  int aoa_pos = pitot.indexOf(',', ias + 1);
  int aoss_pos = pitot.indexOf(',', aoa_pos + 1);
  int pa = pitot.indexOf(',', aoss_pos + 1);
  int sp = pitot.indexOf(',', pa + 1);
  int tp = pitot.indexOf(',', sp + 1);
  int cs = pitot.indexOf(',', tp + 1);

  if (as != -1 && aoa_pos != -1 && aoss_pos != -1 && date != -1) {
    String airSpeedStr = pitot.substring(date + 1, as); 
    String aoaStr = pitot.substring(ias + 1, aoa_pos); 
    String aossStr = pitot.substring(aoa_pos + 1, aoss_pos); 
    pitotMutex.lock();
    airspeed_raw = airSpeedStr.toFloat();
    aoa_raw = aoaStr.toFloat();
    aoss_raw = aossStr.toFloat();
    pitotMutex.unlock();
  }
  got_pitot_readings = true;
  if (read_stream) {
    Serial.println("Pitot: " + String(airspeed_raw) + " " + String(aoa_raw) + " " + String(aoss_raw));
  }
}

void manageCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command.startsWith("home") && !no_safety && !emergency_state) {
      forwardToStepper(command); 
      unsigned long startTime = millis();
      while(millis() - startTime < TIMEOUT_MS) {
        checkEmergency();
        if (!emergency_state) {
          Wire.requestFrom(stepper_address, 1); 
          delay(10); 
          if (Wire.available()) {
            int dataType = Wire.read();
            if (dataType == 1) {
              //Serial.println("homing done");
              homing_done = true;
              Serial.println("homing done");
              break;
            }
          }
        }
        else {
          break;
        }
      }
    } else if (command.startsWith("j") && homing_done == true && !no_safety && !emergency_state) { //format of message j|X(steps)|Y(steps)|JogSpeedX|JogSpeedY
      forwardToStepper(command); 
      unsigned long startTime = millis();
      while(millis() - startTime < TIMEOUT_MS) {
        checkEmergency();
        if (!emergency_state) {
          Wire.requestFrom(stepper_address, 1); 
          delay(10); 
          if (Wire.available()) {
            int dataType = Wire.read(); 
            if (dataType == 2) {
              Serial.println("jog done");
              break;
            } 
            else if (dataType == 3) {
              Serial.println("over axis limit");
              break;
            }
            else if (dataType == 4) {
              Serial.println("limit switch");
              break;
            }
          }
        }
        else {
          break;
        }
      }
    } else if (command.startsWith("center") && homing_done == true && !no_safety && !emergency_state) {
      forwardToStepper(command); 
      unsigned long startTime = millis();
      while(millis() - startTime < TIMEOUT_MS) {
        checkEmergency();
        if (!emergency_state) {
          Wire.requestFrom(stepper_address, 1); 
          delay(10); 
          if (Wire.available()) {
            int dataType = Wire.read(); 
            if (dataType == 4) {
              Serial.println("limit switch");
              break;
            }
            else if (dataType == 5) {
              Serial.println("centering done");
              break;
            }
          }
        } 
        else {
          break;
        }
      }
    } else if (command.startsWith("l")) { //format of message l|MaxX(steps)|MaxY(steps)|MaxFeedSpeedX|MaxFeedSpeedY|FeedAccX|FeedAccY
      forwardToStepper(command); 
    } else if (command.startsWith("m") && homing_done == true && !no_safety && !emergency_state) {
      measure_in_progress = true;
      forwardToStepper(command); 
      int limXSeparator = command.indexOf("|", 2);
      if (limXSeparator == -1) return;
      int limYSeparator = command.indexOf("|", limXSeparator + 1);
      if (limYSeparator == -1) return;
      String reqXStr = command.substring(2,limXSeparator);
      String reqYStr = command.substring(limXSeparator + 1, limYSeparator);
      reqX = reqXStr.toInt();
      reqY = reqYStr.toInt();
    } else if (command.startsWith("trimAoA")) {
      int trimSeparator = command.indexOf("|", 2);
      if (trimSeparator == -1) return;
      String trimAoAStr = command.substring(trimSeparator + 1);
      trimAoA = trimAoAStr.toInt();
      aoaServo.write(trimAoA - 10);
      delay(500); 
      aoaServo.write(trimAoA + 10);
      delay(500); 
      aoaServo.write(trimAoA);
      Serial.print("trimAoA set: ");
      Serial.println(trimAoA);
    } else if (command.startsWith("AoAlim")) {
      int limaoaSeparator = command.indexOf("|", 2);
      if (limaoaSeparator == -1) return;
      String limStr = command.substring(limaoaSeparator + 1);
      limAoA = limStr.toInt();
      Serial.print("limAoA set: ");
      Serial.println(limAoA);
    } else if (command.startsWith("trimAoSS")) {
      int trimSeparator = command.indexOf("|", 2);
      if (trimSeparator == -1) return;
      String trimAoSSStr = command.substring(trimSeparator + 1);
      trimAoSS = trimAoSSStr.toInt();
      aossServo.write(trimAoSS);
      Serial.print("trimAoSS set: ");
      Serial.println(trimAoSS);
    } else if (command.startsWith("AoSSLimMax")) {
      int limaossSeparator = command.indexOf("|", 2);
      if (limaossSeparator == -1) return;
      String limStr = command.substring(limaossSeparator + 1);
      limMaxAoSS = limStr.toInt();
      Serial.print("limMaxAoSS set: ");
      Serial.println(limMaxAoSS);
    } else if (command.startsWith("AoSSLimMin")) {
      int limaossSeparator = command.indexOf("|", 2);
      if (limaossSeparator == -1) return;
      String limStr = command.substring(limaossSeparator + 1);
      limMinAoSS = limStr.toInt();
      Serial.print("limMinAoSS set: ");
      Serial.println(limMinAoSS);
    } else if (command.startsWith("enableAoSS")) {
      aoss_enabled = true;
      Serial.println("AoSSenabled");
    } else if (command.startsWith("disableAoSS")) {
      aoss_enabled = false;
      Serial.println("AoSSdisabled");
    } else if (command.startsWith("tare")) {
      if (twoProps) {
        thrustTorqueControllers[0]->tareAll();
        thrustTorqueControllers[1]->tareAll();
      }
      else {
        thrustTorqueControllers[0]->tareAll();
      }
      tare_done = true;
      Serial.println("tare done");
    } else if (command.startsWith("calFirstTorque")) {
      int bar = command.indexOf('|');
      if (bar != -1) {
        float knownMass = command.substring(bar + 1).toFloat();
        Serial.println(knownMass);
        if (knownMass > 0.0f && thrustTorqueControllers[0]) {
          float firstTrqCalVal = thrustTorqueControllers[0]->calibrateTorque(knownMass);
          Serial.println("CalVal: " + String(firstTrqCalVal));
          cal_found = true;
        }
      }
    } else if (command.startsWith("calSecondTorque")) {
      int bar = command.indexOf('|');
      if (bar != -1) {
        float knownMass = command.substring(bar + 1).toFloat();
        Serial.println(knownMass);
        if (knownMass > 0.0f && twoProps && thrustTorqueControllers[1]) {
          float secondTrqCalVal = thrustTorqueControllers[1]->calibrateTorque(knownMass);
          Serial.println("CalVal: " + String(secondTrqCalVal));
          cal_found = true;
        }
      }
    } else if (command.startsWith("calFirstThrust")) { 
      int bar = command.indexOf('|');
      if (bar != -1) {
        float knownMass = command.substring(bar + 1).toFloat();
        Serial.println(knownMass);
        if (knownMass > 0.0f && thrustTorqueControllers[0]) {
          float firstThrCalVal = thrustTorqueControllers[0]->calibrateThrust(knownMass);
          Serial.println("CalVal: " + String(firstThrCalVal));
          cal_found = true;
        }
      }
   } else if (command.startsWith("calSecondThrust")) {
      int bar = command.indexOf('|');
      if (bar != -1) {
        float knownMass = command.substring(bar + 1).toFloat();
        Serial.println(knownMass);
        if (knownMass > 0.0f && twoProps && thrustTorqueControllers[1]) {
          float secondThrCalVal = thrustTorqueControllers[1]->calibrateThrust(knownMass);
          Serial.println("CalVal: " + String(secondThrCalVal));
          cal_found = true;
        }
      }
   } else if (command.startsWith("setFirstThrCalVal")) {
      int bar = command.indexOf('|');
      if (bar != -1) {
        firstThrCalVal = command.substring(bar + 1).toFloat();
        if (firstThrCalVal != 0.0) {
          thrustTorqueControllers[0]->setThrustCalibration(firstThrCalVal);
        }
      }
    } else if (command.startsWith("setSecondThrCalVal")) {
      int bar = command.indexOf('|');
      if (bar != -1) {
        secondThrCalVal = command.substring(bar + 1).toFloat();
        if (secondThrCalVal != 0.0 && twoProps && thrustTorqueControllers[1]) {
          thrustTorqueControllers[1]->setThrustCalibration(secondThrCalVal);
        }
      }
    } else if (command.startsWith("setFirstTrqCalVal")) {
      int bar = command.indexOf('|');
      if (bar != -1) {
        firstTrqCalVal = command.substring(bar + 1).toFloat();
        if (firstTrqCalVal != 0.0) {
          thrustTorqueControllers[0]->setTorqueCalibration(firstTrqCalVal);
        }
      }
    } else if (command.startsWith("setSecondTrqCalVal")) {
      int bar = command.indexOf('|');
      if (bar != -1) {
        secondTrqCalVal = command.substring(bar + 1).toFloat();
        if (secondTrqCalVal != 0.0 && twoProps && thrustTorqueControllers[1]) {
          thrustTorqueControllers[1]->setTorqueCalibration(secondTrqCalVal);
        }
      }
    } else if (command.startsWith("setFirstThrArmLength")) {
      int bar = command.indexOf('|');
      if (bar != -1) {
        firstThrArmLength = command.substring(bar + 1).toFloat();
        if (thrustTorqueControllers[0]) {
          thrustTorqueControllers[0]->setThrustArmLength(firstThrArmLength);
        }
      }
    } else if (command.startsWith("setSecondThrArmLength")) {
      int bar = command.indexOf('|');
      if (bar != -1) {
        secondThrArmLength = command.substring(bar + 1).toFloat();
        if (thrustTorqueControllers[1]) {
          thrustTorqueControllers[1]->setThrustArmLength(secondThrArmLength);
          Serial.println(secondThrArmLength);
        }
      }
    } else if (command.startsWith("setFirstTrqArmLength")) {
      int bar = command.indexOf('|');
      if (bar != -1) {
        firstTrqArmLength = command.substring(bar + 1).toFloat();
        if (thrustTorqueControllers[0]) {
          thrustTorqueControllers[0]->setTorqueArmLength(firstTrqArmLength);
        }
      }
    } else if (command.startsWith("setSecondTrqArmLength")) {
      int bar = command.indexOf('|');
      if (bar != -1) {
        secondTrqArmLength = command.substring(bar + 1).toFloat();
        if (thrustTorqueControllers[1]) {
          thrustTorqueControllers[1]->setTorqueArmLength(secondTrqArmLength);
          Serial.println(secondTrqArmLength);
        }
      }
    } else if (command.startsWith("streamStart")) {
      read_stream = true;
    } else if (command.startsWith("streamStop")) {
      read_stream = false;
    } else if (command.startsWith("ON")) {
      read_stream_test = true;
    } else if (command.startsWith("OFF")) {
      read_stream_test = false;
    } else if (command.startsWith("BeaconON")) {
      beacon_on = true;
    } else if (command.startsWith("BeaconOFF")) {
      beacon_on = false;
    } else if (command.startsWith("props")) {
      int bar = command.indexOf('|');
      if (bar != -1) {
        tandemSetup = command.substring(bar + 1).toInt();
        if (tandemSetup == 1) {
          disableSecondProp();
        }
        else if (tandemSetup == 2) {
          enableSecondProp();
        }
      }
    } else if (command.startsWith("status")) {
      int n = twoProps ? 2 : 1;
      for (int i = 0; i < n; ++i) {
        Serial.print("PROP "); Serial.println(i);
        Serial.print("  ThrArm="); Serial.println(thrustTorqueControllers[i]->getThrustArmLength(), 6);
        Serial.print("  TrqArm="); Serial.println(thrustTorqueControllers[i]->getTorqueArmLength(), 6);
        Serial.print("  ThrCF ="); Serial.println(thrustTorqueControllers[i]->getThrustCalibration(), 6);
        Serial.print("  TrqCF ="); Serial.println(thrustTorqueControllers[i]->getTorqueCalibration(), 6);
      }
    } else if (command.startsWith("setRamp|")) {
      int bar = command.indexOf('|');
      if (bar != -1) {
        int ms = command.substring(bar + 1).toInt();
        if (ms < 50)    ms = 50;
        if (ms > 10000) ms = 10000;
        escRampMs = ms;
        Serial.print(F("OK|rampMs=")); 
        Serial.println(escRampMs);
      }
    } else if (command.startsWith("startMotor|")) {
      int bar1 = command.indexOf('|');
      int bar2 = command.indexOf('|', bar1 + 1);
      if (bar1 < 0 || bar2 < 0) {
        Serial.println(F("ERR|bad_format"));
      } else {
        String v1s = command.substring(bar1 + 1, bar2); v1s.trim();
        String v2s = command.substring(bar2 + 1);       v2s.trim();

        int t1 = parsePwmTarget(v1s, 0);
        escEnabled1 = true; escTarget1 = t1;
        Serial.print(F("OK|first=")); Serial.println(t1);

        if (twoProps && thrustTorqueControllers[1]) {
          int t2 = parsePwmTarget(v2s, 1);
          escEnabled2 = true; escTarget2 = t2;
          Serial.print(F("OK|second=")); Serial.println(t2);
        } else {
          Serial.println(F("ERR|second_prop_not_enabled"));
        }
      }
    } else if (command.startsWith("firstMotorTest|")) {
      int bar = command.indexOf('|');
      if (bar != -1) {
        int pwm = command.substring(bar + 1).toInt();
        if (pwm < escMinUs1) pwm = escMinUs1;
        if (pwm > escMaxUs1) pwm = escMaxUs1;

        escEnabled1 = true;
        escTarget1  = pwm;
        escCurrent1 = pwm;           // jump immediately (bypass ramp)
      }
    } else if (command.startsWith("secondMotorTest|")) {
      int bar = command.indexOf('|');
      if (bar != -1) {
        int pwm = command.substring(bar + 1).toInt();
        if (pwm < escMinUs2) pwm = escMinUs2;
        if (pwm > escMaxUs2) pwm = escMaxUs2;

        if (twoProps && thrustTorqueControllers[1]) {
          escEnabled2 = true;
          escTarget2  = pwm;
          escCurrent2 = pwm;         // jump immediately (bypass ramp)
        } else {
          Serial.println(F("ERR|second_prop_not_enabled"));
        }
      }
    } else if (command.startsWith("setMin|")) {
      // setMin|<motor>|<min>
      int p1 = command.indexOf('|');
      int p2 = command.indexOf('|', p1 + 1);
      if (p1>0 && p2>p1) {
        int m  = command.substring(p1+1, p2).toInt();
        int mn = command.substring(p2+1).toInt();
        if (mn < 500) mn = 500;
        if (m == 1) { escMinUs1 = mn; if (escMaxUs1 < escMinUs1) escMaxUs1 = escMinUs1; }
        else if (m == 2) { escMinUs2 = mn; if (escMaxUs2 < escMinUs2) escMaxUs2 = escMinUs2; }
        else { Serial.println(F("ERR|min_bad_motor")); }
      } else {
        Serial.println(F("ERR|min_bad_format"));
      }
    } else if (command.startsWith("setMax|")) {
      // setMax|<motor>|<max>
      int p1 = command.indexOf('|');
      int p2 = command.indexOf('|', p1 + 1);
      if (p1>0 && p2>p1) {
        int m  = command.substring(p1+1, p2).toInt();
        int mx = command.substring(p2+1).toInt();
        if (mx > 2500) mx = 2500;
        if (m == 1) { escMaxUs1 = mx; if (escMaxUs1 < escMinUs1) escMinUs1 = escMaxUs1; }
        else if (m == 2) { escMaxUs2 = mx; if (escMaxUs2 < escMinUs2) escMinUs2 = escMaxUs2; }
        else { Serial.println(F("ERR|max_bad_motor")); }
      } else {
        Serial.println(F("ERR|max_bad_format"));
      }
    }else if (command.startsWith("stop")) {
      escHardStopBoth();
      Serial.println(F("OK|stopping"));
    } else if (command.startsWith("init")) {
      InitParams p;
      if (!parseInitPayload(command.substring(5), p)) {
        Serial.println(F("ERR|init_bad_field_count"));
      } else {
        initControllersFrom(p);
        cal_found = true;
      }
    }
  }
}

void assembleStream() {
  if (measure_in_progress == true) {
    float firstThrustData=0, firstRawThrustData=0, firstTorqueData=0, firstRawTorqueData=0, firstRpmData=0;
    float secondThrustData=0, secondRawThrustData=0, secondTorqueData=0, secondRawTorqueData=0, secondRpmData=0;

    float data[2];
    thrustTorqueControllers[0]->getThrust(data);
    firstRawThrustData = data[0]; firstThrustData = data[1];
    thrustTorqueControllers[0]->getTorque(data);
    firstRawTorqueData = data[0]; firstTorqueData = data[1];
    firstRpmData = updateRpm(0);

    if (twoProps && thrustTorqueControllers[1]) {
      thrustTorqueControllers[1]->getThrust(data);
      secondRawThrustData = data[0]; secondThrustData = data[1];
      thrustTorqueControllers[1]->getTorque(data);
      secondRawTorqueData = data[0]; secondTorqueData = data[1];
      secondRpmData = updateRpm(1);
    }

    Wire.requestFrom(stepper_address, 10); 
    delay(10); 
    String receivedPos;
    while (Wire.available()) {
      char c = Wire.read();
      if (isPrintable(c)) {
        receivedPos += c;
      }
    }
    // Process the received string only if it's not empty and has changed since the last read
    static String lastReceived = "";  // Static variable to store the last received string
    if (receivedPos != "" && receivedPos != lastReceived) {
      lastReceived = receivedPos;  // Update last received data

      int spaceIndex = receivedPos.indexOf(' ');
      while (spaceIndex != -1) {
        String pair = receivedPos.substring(0, spaceIndex);
        int commaIndex = pair.indexOf(',');
        if (commaIndex != -1) {
          String XString = pair.substring(0, commaIndex);
          String YString = pair.substring(commaIndex + 1);
          X_pos = XString.toInt();
          Y_pos = YString.toInt();
        }
        receivedPos = receivedPos.substring(spaceIndex + 1); 
        spaceIndex = receivedPos.indexOf(' '); 

        // Print always 13 fields: X Y Thr1 Trq1 RPM1 Airspd AoA AoA_abs Aoss AoSS_abs Thr2 Trq2 RPM2
        Serial.print("Measurements: ");
        Serial.print(X_pos);              Serial.print(' ');
        Serial.print(Y_pos);              Serial.print(' ');
        Serial.print(firstThrustData);    Serial.print(' ');
        Serial.print(firstTorqueData);    Serial.print(' ');
        Serial.print(firstRpmData);       Serial.print(' ');
        Serial.print(airspeed_raw);       Serial.print(' ');
        Serial.print(aoa_raw);            Serial.print(' ');
        Serial.print(absAoA);             Serial.print(' ');
        Serial.print(aoss_raw);           Serial.print(' ');
        Serial.print(absAoSS);            Serial.print(' ');
        Serial.print(secondThrustData);   Serial.print(' ');
        Serial.print(secondTorqueData);   Serial.print(' ');
        Serial.println(secondRpmData);

        if (reqX == X_pos && reqY == Y_pos) {
          //Serial.println("in position");
          measure_in_progress = false;
          break;
        }
      }
    }
  }
}

void testReadThrTrq() {
  if (read_stream_test) {
    if (twoProps && thrustTorqueControllers[1]) {
      float data[2];
      thrustTorqueControllers[0]->getThrust(data);
      float firstRawThrustData = data[0];
      float firstThrustData = data[1];
      thrustTorqueControllers[0]->getTorque(data);
      float firstRawTorqueData = data[0];
      float firstTorqueData = data[1];
      thrustTorqueControllers[1]->getThrust(data);
      float secondRawThrustData = data[0];
      float secondThrustData = data[1];
      thrustTorqueControllers[1]->getTorque(data);
      float secondRawTorqueData = data[0];
      float secondTorqueData = data[1];
      float firstRPM = updateRpm(0);
      float secondRPM = updateRpm(1);
      Serial.print("LC test: ");
      Serial.print(firstThrustData);     Serial.print(' ');
      Serial.print(firstRawThrustData);  Serial.print(' ');
      Serial.print(firstTorqueData);     Serial.print(' ');
      Serial.print(firstRawTorqueData);  Serial.print(' ');
      Serial.print(firstRPM);            Serial.print(' ');
      Serial.print(secondThrustData);    Serial.print(' ');
      Serial.print(secondRawThrustData); Serial.print(' ');
      Serial.print(secondTorqueData);    Serial.print(' ');
      Serial.print(secondRawTorqueData); Serial.print(' ');
      Serial.println(secondRPM);
    } else {
      float data[2];
      thrustTorqueControllers[0]->getThrust(data);
      float firstRawThrustData = data[0];
      float firstThrustData = data[1];
      thrustTorqueControllers[0]->getTorque(data);
      float firstRawTorqueData = data[0];
      float firstTorqueData = data[1];
      float secondRawThrustData = 0;
      float secondThrustData = 0;
      float secondRawTorqueData = 0;
      float secondTorqueData = 0;
      float firstRPM = updateRpm(0);
      float secondRPM = 0;
      Serial.print("LC test: ");
      Serial.print(firstThrustData);     Serial.print(' ');
      Serial.print(firstRawThrustData);  Serial.print(' ');
      Serial.print(firstTorqueData);     Serial.print(' ');
      Serial.print(firstRawTorqueData);  Serial.print(' ');
      Serial.print(firstRPM);            Serial.print(' ');
      Serial.print(secondThrustData);    Serial.print(' ');
      Serial.print(secondRawThrustData); Serial.print(' ');
      Serial.print(secondTorqueData);    Serial.print(' ');
      Serial.print(secondRawTorqueData); Serial.print(' ');
      Serial.println(secondRPM);
    }
  }
}

void beaconController() {
  if (beacon_on == true) {
    digitalWrite(beacon_output, HIGH);
  }
  else {
    digitalWrite(beacon_output, LOW);
  }
}

void forwardToStepper(String data) {
  Wire.beginTransmission(stepper_address); 
  Wire.write(data.c_str(), data.length()); 
  delay(10);
  Wire.endTransmission(); 
}

void adjustSensorAoAPosition(float aoa) {  // This function allows for relative adjustment, not absolute value setting.
  if (aoa > maxAoA - adjustmentStepAoA && aoaSensorPhysicalAngle < (trimAoA + limAoA)) {
    aoaSensorPhysicalAngle += adjustmentStepAoA;
    aoaServo.write(aoaSensorPhysicalAngle);
  } if (aoa < minAoA + adjustmentStepAoA && aoaSensorPhysicalAngle > (trimAoA - limAoA)) {
      aoaSensorPhysicalAngle -= adjustmentStepAoA;
      aoaServo.write(aoaSensorPhysicalAngle);
  } if (aoa == 0.00) {
      aoaSensorPhysicalAngle = trimAoA;
      aoaServo.write(aoaSensorPhysicalAngle);
  }
}

void adjustSensorAoSSPosition(float aoss) { // Relative adjuctment, not absolute.
  if (aoss > maxAoSS - adjustmentStepAoSS && aossSensorPhysicalAngle < limMinAoSS) {
    aossSensorPhysicalAngle += adjustmentStepAoSS;
    if (aossSensorPhysicalAngle >= limMinAoSS) {
      aossSensorPhysicalAngle = limMinAoSS;
    }
    //Serial.println(aossSensorPhysicalAngle);
    aossServo.write(aossSensorPhysicalAngle);
  } if (aoss < minAoSS + adjustmentStepAoSS && aossSensorPhysicalAngle > limMaxAoSS) {
      aossSensorPhysicalAngle -= adjustmentStepAoSS;
      if (aossSensorPhysicalAngle <= limMaxAoSS) {
        aossSensorPhysicalAngle = limMaxAoSS;
      }
      //Serial.println(aossSensorPhysicalAngle);
      aossServo.write(aossSensorPhysicalAngle);
  } if (aoss == 0.00) {
      aossSensorPhysicalAngle = trimAoSS;
      aossServo.write(aossSensorPhysicalAngle);
  }
}

float calculateAbsoluteAoA(float aoa) {
  // Calculate the absolute AoA based on the sensor's physical angle
  return aoaSensorPhysicalAngle - trimAoA + aoa; // Adjust based on the midpoint being 90 degrees
}

float calculateAbsoluteAoSS(float aoss) {
  // Calculate the absolute AoA based on the sensor's physical angle
  return (aoss - (trimAoSS - aossSensorPhysicalAngle) / AoSSratio); // Adjust based on the midpoint being 90 degrees
}

void aoaController() {
  float aoa = aoa_raw;               // Read current AoA
  adjustSensorAoAPosition(aoa);           // Adjust sensor position if near limits
  absAoA = calculateAbsoluteAoA(aoa);  // Calculate the absolute AoA
}

void aossController() {
  if (aoss_enabled == true) {
    float aoss = aoss_raw;               // Read current AoA
    adjustSensorAoSSPosition(aoss);           // Adjust sensor position if near limits
    absAoSS = calculateAbsoluteAoSS(aoss);  // Calculate the absolute AoA
  }
  else {
    float aoss = 0;               // Read current AoA
    adjustSensorAoSSPosition(aoss);           // Adjust sensor position if near limits
    absAoSS = calculateAbsoluteAoSS(aoss);  // Calculate the absolute AoA
  }
}

void escPwmController() {
  const int tickMs = 10;

  if (emergency_state || no_safety) {
    escHardStopBoth();
  }

  int min1 = escMinUs1, max1 = escMaxUs1;
  int min2 = escMinUs2, max2 = escMaxUs2;
  if (max1 < min1) { int t=min1; min1=max1; max1=t; }
  if (max2 < min2) { int t=min2; min2=max2; max2=t; }

  int desired1 = escEnabled1 ? escTarget1 : min1;
  int desired2 = (twoProps && thrustTorqueControllers[1]) ? (escEnabled2 ? escTarget2 : min2) : min2;

  auto rampToward = [&](int current, int desired, int minUs, int maxUs) -> int {
    if (current == desired) return current;
    int delta = desired - current;
    int step = (abs(delta) * tickMs) / escRampMs;
    if (step < 1) step = 1;
    current += (delta > 0 ? step : -step);
    if ((delta > 0 && current > desired) || (delta < 0 && current < desired)) current = desired;
    if (current < minUs) current = minUs;
    if (current > maxUs) current = maxUs;
    return current;
  };

  escCurrent1 = rampToward(escCurrent1, desired1, min1, max1);
  escCurrent2 = rampToward(escCurrent2, desired2, min2, max2);

  esc1.writeMicroseconds(escCurrent1);
  esc2.writeMicroseconds(escCurrent2);
}

void escHardStopBoth() {
  escEnabled1 = false; escEnabled2 = false;
  escTarget1  = escMinUs1;
  escTarget2  = escMinUs2;
  escCurrent1 = escMinUs1;
  escCurrent2 = escMinUs2;
  esc1.writeMicroseconds(escMinUs1);
  esc2.writeMicroseconds(escMinUs2);
}

void checkEmergency() {
  static unsigned long lastDbg = 0;


  // Raw reads (both pins configured INPUT_PULLUP)
  int rawE = digitalRead(emergency_pressed);
  int rawS = digitalRead(safety_disconnected);

  const bool emergencyPressed  = (rawE == HIGH);   // button to GND when pressed
  const bool safetyDisconnected = (rawS == HIGH); // loop open = HIGH

  // Announce on changes only (simple debounce)
  static int lastE = -1, lastS = -1;
  if ((int)emergencyPressed != lastE) {
    Serial.println(emergencyPressed ? "Emergency!" : "Emergency cleared");
    lastE = (int)emergencyPressed;
  }
  if ((int)safetyDisconnected != lastS) {
    Serial.println(safetyDisconnected ? "Safety switch disconnected!" : "Safety connected");
    lastS = (int)safetyDisconnected;
  }

  // Latch for the rest of the system
  emergency_state = emergencyPressed;
  no_safety       = safetyDisconnected;

  // Immediate actions
  if (emergency_state || no_safety) {
    measure_in_progress = false;
    beacon_on = false;
    emergency_state = true;
    homing_done = false;
    // stop ESCs immediately (no ramp)
    escEnabled1 = false; escEnabled2 = false;
    escTarget1  = escMinUs1; escTarget2 = escMinUs2;
    esc1.writeMicroseconds(escMinUs1);
    esc2.writeMicroseconds(escMinUs2);
    Serial.println("Emergency!");
    digitalWrite(emergency_stepper, HIGH);
  } else {
    digitalWrite(emergency_stepper, LOW);
  }
}

void loop() {
  controller.run();
}
