#include <rtos.h>
#include <mbed.h>
#include <Servo.h>
#include <Wire.h>
#include <Thread.h>
#include <ThreadController.h>

#define TIMEOUT_MS 90000 // Timeout in milliseconds
#define DEBOUNCE_DELAY 50 // Debounce time in milliseconds

#define safety_disconnected A2
#define emergency_pressed A0
#define emergency_stepper D4
#define emergency_thrtrqrpm D3
#define beacon_output D2

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

ButtonState safetySwitchState = BUTTON_DOWN;
ButtonState emergencyButtonState = BUTTON_DOWN;

char armDirection;
bool limits_sent = false;
bool read_stream = false;
bool mass_sent = false;
bool cal_value_received = false;
bool init_message = false;
bool input_complete = false;
bool get_pitot = false;
bool cal_found = false;
bool measure_in_progress = false;
bool homing_done = false;
bool no_safety = false;
bool rpm_test = false;
bool got_pitot_readings = false;
bool read_stream_test = false;
bool tare_done = false;
bool beacon_on = false;
bool aoss_enabled = true;
volatile bool getThrTrqRPM = false;
volatile bool emergency_state = false;
volatile bool emergency_cleared = false;
volatile long thr;
volatile long trq;
volatile long rpm;
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
float trqArmLength;
float trqCalVal; 
float thrArmLength; 
float thrCalVal;
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
String init_out;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(safety_disconnected, INPUT_PULLUP);
  pinMode(emergency_pressed, INPUT_PULLUP);
  pinMode(emergency_stepper, OUTPUT);
  pinMode(emergency_thrtrqrpm, OUTPUT);
  pinMode(beacon_output, OUTPUT);

  aoaServo.attach(D6); 
  aossServo.attach(D5);
  aoaServo.write(trimAoA);  // Set servo1 to 90-degree position
  aossServo.write(trimAoSS);  // Set servo2 to 45-degree position
  
  Serial.begin(115200); //Serial between UI
  Serial3.begin(115200); //Serial between Pitot' sensor
  Wire.begin(); //I2C to stepper controller
  Wire2.begin(); //I2C for thrust, torque and RPM controller

  thread1.onRun(manageCommands);
  thread1.setInterval(0); 
  thread2->onRun(testReadThrTrq);
  thread2->setInterval(100); 
  thread3->onRun(readPitot); 
  thread3->setInterval(0); 
  thread4->onRun(aoaController); 
  thread4->setInterval(0); 
  thread5->onRun(assembleStream); 
  thread5->setInterval(5); //changing this value changes the resolution of the readings, don't set it to 0!
  thread6->onRun(checkEmergency); 
  thread6->setInterval(0);
  thread7->onRun(rpmTest); 
  thread7->setInterval(0);
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
    delay(10);                      
    digitalWrite(LED_BUILTIN, LOW);   
    delay(10); 
    String input = Serial.readStringUntil('\n');
    String originalInput = input;
    if (input.startsWith("init|")) {
      input.remove(0, 5);
      int delimiterIndex;
      delimiterIndex = input.indexOf('|');
      if (delimiterIndex != -1) {
        trqArmLength = input.substring(0, delimiterIndex).toFloat();
        input.remove(0, delimiterIndex + 1);
        Serial.println(trqArmLength);
        delimiterIndex = input.indexOf('|');
        if (delimiterIndex != -1) {
          trqCalVal = input.substring(0, delimiterIndex).toFloat();
          input.remove(0, delimiterIndex + 1);
          Serial.println(trqCalVal);
          delimiterIndex = input.indexOf('|');
          if (delimiterIndex != -1) {
            thrArmLength = input.substring(0, delimiterIndex).toFloat();
            input.remove(0, delimiterIndex + 1);
            Serial.println(thrArmLength);
            delimiterIndex = input.indexOf('|');

            thrCalVal = input.substring(0, delimiterIndex).toFloat();
            Serial.println(thrCalVal);

          }
        }
      }
      if (trqArmLength != 0.0 && trqCalVal != 0.0 && thrArmLength != 0.0 && thrCalVal != 0.0) {
        input_complete = true;
        String data = "setTrqArmLength|" + String(trqArmLength);
        forwardToThrTrq(data);
        data = "setTrqCalVal|" + String(trqCalVal);
        forwardToThrTrq(data);
        data = "setThrArmLength|" + String(thrArmLength);
        forwardToThrTrq(data);
        data = "setThrCalVal|" + String(thrCalVal);
        forwardToThrTrq(data);
        cal_found = true;
      }
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
  // else {
  //   airspeed_raw = 0.00;
  //   aoa_raw = 0.00;
  //   aoss_raw = 0.00;
  // }
  got_pitot_readings = true;
}

void testReadThrTrq() {
  if (read_stream_test == true) {
    Wire2.requestFrom(thrtrq_address, 30); // Requesting 20 bytes from the slave
    while (Wire2.available() < 30) {
      delay(2);
    }
    
    char buffer[31]; // Buffer to store incoming data plus a space for null-termination
    int i = 0;
    while (Wire2.available()) {
      buffer[i++] = Wire2.read();
      if (i >= 30) break; // Prevent buffer overflow
    }
    buffer[i] = '\0'; // Null-terminate the string

    String receivedthrtrqtest = String(buffer); // Convert char array to String for easier manipulation

    int delimiterIndex1 = receivedthrtrqtest.indexOf(",", 0);
    int delimiterIndex2 = receivedthrtrqtest.indexOf(",", delimiterIndex1 + 1);
    int delimiterIndex3 = receivedthrtrqtest.indexOf(",", delimiterIndex2 + 1);

    if (delimiterIndex1 == -1 || delimiterIndex2 == -1 || delimiterIndex3 == -1) {
      Serial.println("Error: Invalid data format received");
      return;
    }

    String thrStr = receivedthrtrqtest.substring(0, delimiterIndex1);
    String thrWeightStr = receivedthrtrqtest.substring(delimiterIndex1, delimiterIndex2);
    String trqStr = receivedthrtrqtest.substring(delimiterIndex2 + 1, delimiterIndex3);
    String trqWeightStr = receivedthrtrqtest.substring(delimiterIndex3 + 1);

    long thr = thrStr.toInt();
    float thr_weight = thrWeightStr.toFloat();
    long trq = trqStr.toInt();
    float trq_weight = trqWeightStr.toFloat();

    Serial.println("LC test: " + String(thr) + " " + String(thr_weight) + " " + String(trq) + "," + String(trq_weight));
  }
  else {
    //Serial.println("siin");
    return;
  }
}

void manageCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command.startsWith("start") && !no_safety && !emergency_state) {
      forwardToRPM(command);
    } else if (command.startsWith("stop")) {
      forwardToRPM(command);
      rpm_test = false;
    } else if (command.startsWith("min")) {
      forwardToRPM(command);
    } else if (command.startsWith("max")) {
      forwardToRPM(command);
    } else if (command.startsWith("test") && !no_safety && !emergency_state) {
      forwardToRPM(command);
      rpm_test = true;
    } else if (command.startsWith("home") && !no_safety && !emergency_state) {
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
              Serial.println("homing done");
              homing_done = true;
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
      Serial.println(aoss_enabled);
    } else if (command.startsWith("disableAoSS")) {
      aoss_enabled = false;
      Serial.println(aoss_enabled);
    } else if (command.startsWith("tare")) {
      tare_done = false;
      forwardToThrTrq(command);
      unsigned long startTime = millis();
      while(millis() - startTime < TIMEOUT_MS) {
          Wire2.requestFrom(thrtrq_address, 1); 
          delay(10); 
          if (Wire2.available()) {
            int dataType = Wire2.read();
            if (dataType == 6) {
              tare_done = true;
              Serial.println("tare done");
              break;
            }
          }
        }
    } else if (command.startsWith("calTorque")) {
      forwardToThrTrq(command);
      cal_found = false;
    } else if (command.startsWith("calThrust")) {
      forwardToThrTrq(command);
      cal_found = false;
    } else if (command.startsWith("calMass")) {
      if (cal_found == false) {
        forwardToThrTrq(command);
        float receivedFloat;
        Wire2.requestFrom(thrtrq_address, 32);
        char buffer[32];
        int i = 0;
        bool isValidData = false;
        while(Wire2.available() && i < sizeof(buffer) - 1) { 
          char c = Wire2.read();
          if (c == '-' || c == '.' || isdigit(c)) {
            buffer[i++] = c;
            isValidData = true;
          } else if (isValidData) {
            break;
          }
        }
        buffer[i] = '\0';
        receivedFloat = atof(buffer);
        if (isValidData) {
          receivedFloat = atof(buffer);
          Serial.println("CalVal: " + String(receivedFloat));
          cal_found = true;
        } else {
          Serial.println("Error");
          cal_found = false;
          Serial.print("Received data: ");
          Serial.println(buffer);
        }
      }
    } else if (command.startsWith("setThrCalVal")) {
      forwardToThrTrq(command);
    } else if (command.startsWith("setTrqCalVal")) {
      forwardToThrTrq(command);
    } else if (command.startsWith("setThrArmLength")) {
      forwardToThrTrq(command);
    } else if (command.startsWith("setTrqArmLength")) {
      forwardToThrTrq(command);
    } else if (command.startsWith("streamStart")) {
      forwardToThrTrq(command);
      read_stream = true;
    } else if (command.startsWith("streamStop")) {
      forwardToThrTrq(command);
      read_stream = false;
    } else if (command.startsWith("ON")) {
      forwardToThrTrq(command);
      read_stream_test = true;
      //Serial.println(read_stream_test);
    } else if (command.startsWith("OFF")) {
      read_stream_test = false;
      forwardToThrTrq(command);
      //Serial.println(read_stream_test);
    } else if (command.startsWith("BeaconON")) {
      beacon_on = true;
    } else if (command.startsWith("BeaconOFF")) {
      beacon_on = false;
    } else if (command.startsWith("init")) {
      input_complete = false;
      command.remove(0, 5);
      int delimiterIndex;
      delimiterIndex = command.indexOf('|');
      if (delimiterIndex != -1) {
        trqArmLength = command.substring(0, delimiterIndex).toFloat();
        command.remove(0, delimiterIndex + 1);
        delimiterIndex = command.indexOf('|');
        if (delimiterIndex != -1) {
          trqCalVal = command.substring(0, delimiterIndex).toFloat();
          command.remove(0, delimiterIndex + 1);
          delimiterIndex = command.indexOf('|');
          if (delimiterIndex != -1) {
            thrArmLength = command.substring(0, delimiterIndex).toFloat();
            command.remove(0, delimiterIndex + 1);
            delimiterIndex = command.indexOf('|');
            if (delimiterIndex != -1) {
              thrCalVal = command.substring(0, delimiterIndex).toFloat();
            }
          }
        }
      }
      if (trqArmLength != 0.0 && trqCalVal != 0.0 && thrArmLength != 0.0 && thrCalVal != 0.0) {
        input_complete = true;
        String data = "setTrqArmLength|" + String(trqArmLength);
        forwardToThrTrq(data);
        data = "setTrqCalVal|" + String(trqCalVal);
        forwardToThrTrq(data);
        data = "setThrArmLength|" + String(thrArmLength);
        forwardToThrTrq(data);
        data = "setThrCalVal|" + String(thrCalVal);
        forwardToThrTrq(data);
      }
    } 
  }
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

void assembleStream() {
  if (measure_in_progress == true) {
    Wire2.requestFrom(thrtrq_address, 15); 
    delay(10);
    String receivedthrtrq;
    while (Wire2.available()) {
      char c = Wire2.read();
      receivedthrtrq += c;
    }
    int delimiterIndex1 = receivedthrtrq.indexOf(",", 2);
    if (delimiterIndex1 == -1) return;

    String thrStr = receivedthrtrq.substring(0, delimiterIndex1);
    String trqStr = receivedthrtrq.substring(delimiterIndex1 + 1);

    thrtrqMutex.lock();
    thr = thrStr.toInt();
    trq = trqStr.toInt();
    thrtrqMutex.unlock();

    Wire2.requestFrom(rpm_address, 5); 
    delay(10);
    String receivedRPM;
    while (Wire2.available()) {
      char c = Wire2.read();
      receivedRPM += c;
    }
    rpmMutex.lock();
    rpm = receivedRPM.toInt();
    rpmMutex.unlock(); 
    
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

        //if (got_pitot_readings == true) {
          Serial.println("Measurements: " + String(X_pos) + " " + String(Y_pos) + " " + String(thr) + " " + String(trq) + " " + String(rpm) + " " + String(airspeed_raw) + " " + String(aoa_raw) + " " + String(absAoA) + " " + String(aoss_raw) + " " + String(absAoSS));
        //}

        if (reqX == X_pos && reqY == Y_pos) {
          //Serial.println("in position");
          measure_in_progress = false;
          break;
        }
      }
    }
  }
}

ButtonState buttonDebounce(int pin, ButtonState currentState, unsigned long *lastDebounceTime) {
  static const unsigned long debounceDelay = 50;  // debounce time in milliseconds
  int reading = digitalRead(pin);
  unsigned long currentTime = millis();

  switch (currentState) {
      case BUTTON_UP:
          if (reading == LOW) {  // Assuming active low
              *lastDebounceTime = currentTime;
              currentState = BUTTON_FALLING;
          }
          break;
      case BUTTON_DOWN:
          if (reading == HIGH) {
              *lastDebounceTime = currentTime;
              currentState = BUTTON_RISING;
          }
          break;
      case BUTTON_FALLING:
          if ((currentTime - *lastDebounceTime) > debounceDelay) {
              if (reading == LOW) {
                  currentState = BUTTON_DOWN;
              } else {
                  currentState = BUTTON_UP;
              }
          }
          break;
      case BUTTON_RISING:
          if ((currentTime - *lastDebounceTime) > debounceDelay) {
              if (reading == HIGH) {
                  currentState = BUTTON_UP;
              } else {
                  currentState = BUTTON_DOWN;
              }
          }
          break;
  }
  return currentState;
}

void checkEmergency() {
  safetySwitchState = buttonDebounce(safety_disconnected, safetySwitchState, &lastSafetySwitchTime);
  emergencyButtonState = buttonDebounce(emergency_pressed, emergencyButtonState, &lastEmergencyButtonTime);

  if (safetySwitchState == BUTTON_UP) {
    no_safety = true;
    Serial.println("Safety switch disconnected!");
  } else if (safetySwitchState == BUTTON_DOWN) {
    no_safety = false;
  }

  if (emergencyButtonState == BUTTON_UP) {
    emergency_state = true;
    measure_in_progress = false;
    beacon_on = false;
    digitalWrite(emergency_stepper, HIGH);
    digitalWrite(emergency_thrtrqrpm, HIGH);
    forwardToRPM("stop");
    Serial.println("Emergency!");
  } else if (emergencyButtonState == BUTTON_DOWN) {
    emergency_state = false;
    digitalWrite(emergency_stepper, LOW);
    digitalWrite(emergency_thrtrqrpm, LOW);
  } else if (emergencyButtonState == BUTTON_FALLING) {
    homing_done = false;
    measure_in_progress = false;
    rpm_test = false;
    Serial.println("OK");
  }
}

void rpmTest() {
  if (rpm_test == true) {
    Wire2.requestFrom(rpm_address, 5); 
    delay(10);
    String receivedRPM;
    while (Wire2.available()) {
      char c = Wire2.read();
      receivedRPM += c;
    }
    rpmMutex.lock();
    rpm = receivedRPM.toInt();
    rpmMutex.unlock();
    Serial.println("RPM_test:" + String(rpm));
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

void forwardToThrTrq(String data) {
  Wire2.beginTransmission(thrtrq_address); 
  Wire2.write(data.c_str(), data.length()); 
  delay(10);
  Wire2.endTransmission();
}

void forwardToRPM(String data) {
  Wire2.beginTransmission(rpm_address); 
  Wire2.write(data.c_str(), data.length()); 
  delay(10);
  Wire2.endTransmission(); 
}

void adjustSensorAoAPosition(float aoa) {
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

void adjustSensorAoSSPosition(float aoss) {
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

void loop() {
  controller.run();
}


