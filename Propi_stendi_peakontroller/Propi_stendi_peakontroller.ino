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
volatile bool getThrTrqRPM = false;
volatile bool emergency_state = false;
volatile bool emergency_cleared = false;
long thr;
long trq;
long rpm;
int X_pos;
int Y_pos;
int init_pos = 0;
int stepper_address = 10;
int thrtrq_address = 5;
int rpm_address = 15;
int reqX;
int reqY;
int trimAoA = 1500;
int trimAoSS = 1500;
int AoAPos;
float airspeed_raw;
float aoa_raw;
float aoss_raw;
float receivedValue = 0.0;
float ratioAoA = 110; //110 PWM units for 10 deg of preset for AoA
float ratioAoSS;
float absAoA;
float armLength;
float trqLCalVal; 
float trqRCalVal; 
float thrCalVal; 
unsigned long lastSafetySwitchTime = 0;
unsigned long lastEmergencyButtonTime = 0;
String init_out;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(safety_disconnected, INPUT_PULLUP);
  pinMode(emergency_pressed, INPUT_PULLUP);
  pinMode(emergency_stepper, OUTPUT);
  pinMode(emergency_thrtrqrpm, OUTPUT);

  aoaServo.attach(D6); 
  aossServo.attach(D5);
  aoaServo.writeMicroseconds(1500);  // Set servo1 to 90-degree position
  aossServo.writeMicroseconds(1500);  // Set servo2 to 45-degree position
  
  Serial.begin(115200); //Serial between UI
  Serial3.begin(115200); //Serial between Pitot' sensor
  Wire.begin(); //I2C to stepper controller
  Wire2.begin(); //I2C for thrust, torque and RPM controller

  thread1.onRun(readCommands);
  thread1.setInterval(0); 
  thread2->onRun(readThrTrqRPM);
  thread2->setInterval(0); 
  thread3->onRun(readPitot); 
  thread3->setInterval(0); 
  thread4->onRun(angleController); 
  thread4->setInterval(0); 
  thread5->onRun(assembleStream); 
  thread5->setInterval(1); //changing this value changes the resolution of the readings, don't set it to 0!
  thread6->onRun(checkEmergency); 
  thread6->setInterval(0);
  // Add threads to controller
  controller.add(&thread1);
  controller.add(thread2);
  controller.add(thread3);
  //controller.add(thread4);
  controller.add(thread5);
  controller.add(thread6);

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
        armLength = input.substring(0, delimiterIndex).toFloat();
        input.remove(0, delimiterIndex + 1);

        delimiterIndex = input.indexOf('|');
        if (delimiterIndex != -1) {
          trqLCalVal = input.substring(0, delimiterIndex).toFloat();
          input.remove(0, delimiterIndex + 1);

          delimiterIndex = input.indexOf('|');
          if (delimiterIndex != -1) {
            trqRCalVal = input.substring(0, delimiterIndex).toFloat();
            input.remove(0, delimiterIndex + 1);

            delimiterIndex = input.indexOf('|');
            if (delimiterIndex != -1) {
              thrCalVal = input.substring(0, delimiterIndex).toFloat();
              input.remove(0, delimiterIndex + 1);
              
              armDirection = input.charAt(0);
            }
          }
        }
      }
      if (armLength != 0.0 && trqLCalVal != 0.0 && trqRCalVal != 0.0 && thrCalVal != 0.0 && (armDirection != 'R' || armDirection != 'L' || armDirection != '0')) {
        input_complete = true;
        cal_found = true;
        forwardToThrTrq(originalInput);
        cal_found = true;
      }
    }
  }
}

void readPitot() {
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
}

void readThrTrqRPM() {
  if (read_stream == true) {
    if (getThrTrqRPM == true) {
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
    }
    getThrTrqRPM = false;
  }
  else {
    thr = 0;
    trq = 0;
    rpm = 0;
  }
}

void readCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command.startsWith("start") && !no_safety && !emergency_state) {
      forwardToRPM(command);
    } else if (command.startsWith("stop")) {
      forwardToRPM(command);
    } else if (command.startsWith("min")) {
      forwardToRPM(command);
    } else if (command.startsWith("max")) {
      forwardToRPM(command);
    } else if (command.startsWith("tare")) {
      forwardToThrTrq(command);
    } else if (command.startsWith("init")) {
      forwardToThrTrq(command);
    } else if (command.startsWith("calLeft")) {
      forwardToThrTrq(command);
      cal_found = false;
    } else if (command.startsWith("calRight")) {
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
    } else if (command.startsWith("measureL")) {
      forwardToThrTrq(command);
    } else if (command.startsWith("measureR")) {
      forwardToThrTrq(command);
    } else if (command.startsWith("measure0")) {
      forwardToThrTrq(command);
    } else if (command.startsWith("measure0")) {
      forwardToThrTrq(command);
    } else if (command.startsWith("measure0")) {
      forwardToThrTrq(command);
    } else if (command.startsWith("setThrCalVal")) {
      forwardToThrTrq(command);
    } else if (command.startsWith("setTrqLCalVal")) {
      forwardToThrTrq(command);
    } else if (command.startsWith("setTrqRCalVal")) {
      forwardToThrTrq(command);
    } else if (command.startsWith("setArmLength")) {
      forwardToThrTrq(command);
    } else if (command.startsWith("streamStart")) {
      forwardToThrTrq(command);
      read_stream = true;
    } else if (command.startsWith("streamStop")) {
      forwardToThrTrq(command);
      read_stream = false;
    } else if (command.startsWith("h")) {
      forwardToStepper(command); 
      if (command.startsWith("h")) {
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
    } else if (command.startsWith("c") && homing_done == true && !no_safety && !emergency_state) {
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
      String trimStr = command.substring(trimSeparator + 1);
      trimAoA = trimStr.toInt();
      aoaServo.writeMicroseconds(trimAoA - 100);
      delay(500); 
      aoaServo.writeMicroseconds(trimAoA + 100);
      delay(500); 
      aoaServo.writeMicroseconds(trimAoA);
    } 
  }
}

void angleController() {
  // if (aoa_raw >= 10.0 && aoa_raw < 15.0) {
  //   AoAPos = trimAoA + ratioAoA;
  //   absAoA = aoa_raw + 10.0;
  //} else 
  if (aoa_raw >= 15.0 && aoa_raw <= 20.0) {
    AoAPos = trimAoA + (2 * ratioAoA);
    absAoA = aoa_raw + 20.0;
  } else if (aoa_raw > 20.0) {
    AoAPos = trimAoA + (4 * ratioAoA);
    absAoA = aoa_raw + 42.0;
  // } else if (aoa_raw <= -10.0 && aoa_raw > -15.0) {
  //   AoAPos = trimAoA - ratioAoA;
  //   absAoA = aoa_raw - 10.0;
  } else if (aoa_raw <= -15.0 && aoa_raw >= -20.0) {
    AoAPos = trimAoA - (2 * ratioAoA);
    absAoA = aoa_raw - 21.0;
  } else if (aoa_raw < -20.0) {
    AoAPos = trimAoA - (4 * ratioAoA);
    absAoA = aoa_raw - 43.0;
  } else {
    AoAPos = trimAoA;
    absAoA = aoa_raw;
  }
  //int AoAPos = trimAoA + ((aoa_raw) * (ratioAoA / 10));
  aoaServo.writeMicroseconds(AoAPos);  // Set servo1 to 90-degree position
  // Serial.print(AoAPos);
  // Serial.print(" ");
  // Serial.print(absAoA);
  // Serial.print(" ");
  // Serial.println(aoa_raw);
  //aossServo.writeMicroseconds(1500);  // Set servo2 to 45-degree position
}

void assembleStream() {
  if (measure_in_progress == true) {
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
        receivedPos = receivedPos.substring(spaceIndex + 1);  // Reduce the receivedString
        spaceIndex = receivedPos.indexOf(' ');  // Find next space

        getThrTrqRPM = true;
        Serial.println(String(X_pos) + " " + String(Y_pos) + " " + String(thr) + " " + String(trq) + " " + String(rpm) + " " + String(airspeed_raw) + " " + String(aoa_raw) + " " + String(aoss_raw));

        if (reqX == X_pos && reqY == Y_pos) {
          Serial.println("position reached");
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
    Serial.println("OK");
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

void loop() {
  if (init_message == true) {
    forwardToThrTrq(init_out);
    init_message = false;
  }
  controller.run();
}


