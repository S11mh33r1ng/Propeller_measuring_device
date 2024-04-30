#include <Wire.h>
#include "AccelStepper.h" 

AccelStepper stepperX(1, 2, 5);   // 1 = Interface
                                  // Pin 2 connected to STEP pin
                                  // Pin 5 connected to DIR pin
AccelStepper stepperY(1, 3, 6);   // 1 = Interface
                                  // Pin 3 connected to STEP pin
                                  // Pin 6 connected to DIR pin                              

// Define the Pins used
#define home_switch_x 9 // Pin 2 connected to Home Switch (MicroSwitch) X axis
#define home_switch_y 10 // Pin 3 connected to Home Switch (MicroSwitch) Y axis
#define enable_pin 8
#define emergency_pressed 12

long TravelX; //steps of travel X
long TravelY; //steps of travel Y
long SpeedX;
long SpeedY;
long initial_homing_x=-1;  // Used to Home Stepper X at startup
long initial_homing_y=-1;  // Used to Home Stepper Y at startup
long MaxX = 7880; //max travel in steps
long MaxY = 2625; //max travel in steps
long MinX = 0;
long MinY = 0;

int move_finished_x=1;  // Used to check if move on X axis is completed
int move_finished_y=1;  // Used to check if move on Y axis is completed
int homing_maxspeed_x = 200;
int homing_accel_x = 100;
int homing_maxspeed_y = 200;
int homing_accel_y = 100;
int feed_maxspeed_x = 1200;
int feed_accel_x = 800;
int feed_maxspeed_y = 600;
int feed_accel_y = 400;
int x_pos = 0;
int a;
int b;

bool homing_performed = false;
bool limit_switch_hit = false;
bool home_now = false;
bool home_found = false;
bool center_now = false;
bool center_in_pos = false;
bool jog_now = false;
bool jog_in_pos = false;
bool over_axis_limit = false;
bool measure_now = false;
bool measure_in_pos = false;
bool send_position = false;

void setup() {
  Serial.begin(115200);
  Wire.begin(10);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  pinMode(home_switch_x, INPUT_PULLUP);
  pinMode(home_switch_y, INPUT_PULLUP);
  pinMode(enable_pin, OUTPUT);
  digitalWrite(enable_pin, LOW);
  pinMode(emergency_pressed, INPUT);
  
//  Set Max Speed and Acceleration of each Steppers at startup for homing
  stepperX.setMaxSpeed(homing_maxspeed_x);
  stepperX.setAcceleration(homing_accel_x);  
  stepperY.setMaxSpeed(homing_maxspeed_y);
  stepperY.setAcceleration(homing_accel_y);
}

void check_emergency() {
  if (digitalRead(emergency_pressed) == 1) {
    digitalWrite(enable_pin, HIGH);
    //Serial.println("Emergency!");
    if (digitalRead(emergency_pressed) == 0) {
      homing_performed = false;
      }
    }
}

void loop() {
  while(true) {
    if (home_now == true) {
      digitalWrite(enable_pin, LOW);
      home_found = false;
      homing_performed = false;
      stepperX.setMaxSpeed(homing_maxspeed_x);      // Set Max Speed of Stepper (Slower to get better accuracy)
      stepperX.setAcceleration(homing_accel_x);  // Set Acceleration of Stepper
      stepperY.setMaxSpeed(homing_maxspeed_y);
      stepperY.setAcceleration(homing_accel_y);

      while (digitalRead(home_switch_y)) {  // Make the Stepper move CCW until the switch is activated   
        stepperY.moveTo(initial_homing_y);  // Set the position to move to
        initial_homing_y--;  // Decrease by 1 for next move if needed
        stepperY.run();  // Start moving the stepper
        check_emergency();
        delay(5);
        }

      stepperY.setCurrentPosition(0);  // Set the current position as zero for now
      stepperY.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
      stepperY.setAcceleration(100.0);  // Set Acceleration of Stepper
      initial_homing_y=1;

      while (!digitalRead(home_switch_y)) { // Make the Stepper move CW until the switch is deactivated
        stepperY.moveTo(initial_homing_y);  
        stepperY.run();
        initial_homing_y++;
        check_emergency();
        delay(3);
      }
      
      stepperY.setCurrentPosition(0);
      stepperY.setMaxSpeed(feed_maxspeed_y);      // Set Max Speed of Stepper (Faster for regular movements)
      stepperY.setAcceleration(feed_accel_y);  // Set Acceleration of Stepper

      while (digitalRead(home_switch_x)) {  // Make the Stepper move CCW until the switch is activated   
        stepperX.moveTo(initial_homing_x);  // Set the position to move to
        initial_homing_x--;  // Decrease by 1 for next move if needed
        stepperX.run();  // Start moving the stepper
        check_emergency();
        delay(3);
      } 

      stepperX.setCurrentPosition(0);  // Set the current position as zero for now
      stepperX.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
      stepperX.setAcceleration(100.0);  // Set Acceleration of Stepper
      initial_homing_x=1;

      while (!digitalRead(home_switch_x)) { // Make the Stepper move CW until the switch is deactivated
        stepperX.moveTo(initial_homing_x);  
        stepperX.run();
        initial_homing_x++;
        check_emergency();
        delay(3);
      }
      
      stepperX.setCurrentPosition(0);
      stepperX.setMaxSpeed(feed_maxspeed_x);      // Set Max Speed of Stepper (Faster for regular movements)
      stepperX.setAcceleration(feed_accel_x);  // Set Acceleration of Stepper

      homing_performed = true;
      home_now = false;
      home_found = true;
    } 
    else if (jog_now == true) {
      over_axis_limit = false;
      move_finished_x = 0;
      move_finished_y = 0;
      jog_in_pos = false;
      stepperX.setMaxSpeed(SpeedX);
      stepperY.setMaxSpeed(SpeedY);
      if (TravelX < MinX || TravelX > MaxX || TravelY < MinY || TravelY > MaxY) {
        over_axis_limit = true;
        break;
      } 
      if (TravelX >= MinX || TravelX <= MaxX && TravelY >= MinY || TravelY <= MaxY) {
        stepperX.moveTo(TravelX);
        stepperY.moveTo(TravelY);
        while ((stepperX.distanceToGo() != 0) || (stepperY.distanceToGo() != 0)) {
          if (!digitalRead(home_switch_y) || !digitalRead(home_switch_x)) {
            stepperX.stop();
            stepperY.stop();
            stepperX.setCurrentPosition(0);
            stepperY.setCurrentPosition(0);
            limit_switch_hit = true;
            homing_performed = false;
            digitalWrite(enable_pin, HIGH);
            break;
          } else {
            stepperX.moveTo(TravelX);
            stepperY.moveTo(TravelY);
            stepperX.run();
            stepperY.run();
            check_emergency();
          }
        }
        if ((move_finished_x == 0) && (stepperX.distanceToGo() == 0) && !limit_switch_hit) {
          move_finished_x = 1;
        }
        if ((move_finished_y == 0) && (stepperY.distanceToGo() == 0) && !limit_switch_hit) {
          move_finished_y = 1;
        }
        if (move_finished_x == 1 && move_finished_y == 1) {
          jog_in_pos = true;
        }
      }
      jog_now = false;
    } 
    else if (center_now == true) {
      move_finished_x = 0;
      move_finished_y = 0;
      stepperX.setMaxSpeed(feed_maxspeed_x);
      stepperY.setMaxSpeed(feed_maxspeed_y);
      stepperX.moveTo(MaxX);
      stepperY.moveTo(MinY);
      while ((stepperX.distanceToGo() != 0) || (stepperY.distanceToGo() != 0)) {
        if (!digitalRead(home_switch_y) || !digitalRead(home_switch_x)) {
          stepperX.stop();
          stepperY.stop();
          stepperX.setCurrentPosition(0);
          stepperY.setCurrentPosition(0);
          limit_switch_hit = true;
          homing_performed = false;
          digitalWrite(enable_pin, HIGH);
          break;
        } 
        else {
          stepperX.moveTo(MaxX);
          stepperY.moveTo(MinY);
          stepperX.run();
          stepperY.run();
          check_emergency();
        }
      }
      if ((move_finished_x == 0) && (stepperX.distanceToGo() == 0) && !limit_switch_hit) {
        move_finished_x = 1;
      }
      if ((move_finished_y == 0) && (stepperY.distanceToGo() == 0) && !limit_switch_hit) {
        move_finished_y = 1;
      }
      if (move_finished_x == 1 && move_finished_y == 1) {
        center_in_pos = true;
      }
      center_now = false;
    } 
    else if(measure_now == true) {
      move_finished_x = 0;
      move_finished_y = 0;
      stepperX.setMaxSpeed(SpeedX);
      stepperY.setMaxSpeed(SpeedY);
      stepperX.moveTo(TravelX);
      stepperY.moveTo(TravelY);
      while ((stepperX.distanceToGo() != 0) || (stepperY.distanceToGo() != 0)) {
        if (!digitalRead(home_switch_y) || !digitalRead(home_switch_x)) {
          stepperX.stop();
          stepperY.stop();
          stepperX.setCurrentPosition(0);
          stepperY.setCurrentPosition(0);
          limit_switch_hit = true;
          homing_performed = false;
          digitalWrite(enable_pin, HIGH);
          break;
        } 
        if (x_pos == 1000) {
          send_position = true;
          x_pos = 0;
        }
        else {
          stepperX.moveTo(TravelX);
          stepperY.moveTo(TravelY);
          stepperX.run();
          stepperY.run();
          check_emergency();
          x_pos++;
        }
      }
      if ((move_finished_x == 0) && (stepperX.distanceToGo() == 0) && !limit_switch_hit) {
        move_finished_x = 1;
      }
      if ((move_finished_y == 0) && (stepperY.distanceToGo() == 0) && !limit_switch_hit) {
        move_finished_y = 1;
      }
      if (move_finished_x == 1 && move_finished_y == 1) {
        measure_in_pos = true;
      }
      measure_now = false;
    }
  }
}

void receiveEvent(int numBytes) {
  String command = "";
  while (Wire.available()) {
    char c = Wire.read();
    if (c == ' ') {
      break;
    }
    command += c;
    parseReceivedMessage(command);
  }
}

void parseReceivedMessage(String message) {
  if (message.startsWith("j|")) {
    int jxSeparator = message.indexOf("|", 2);
    if (jxSeparator == -1) return;
    int jySeparator = message.indexOf("|", jxSeparator + 1);
    if (jySeparator == -1) return;
    int jspeedXSeparator = message.indexOf("|", jySeparator + 1);
    if (jspeedXSeparator == -1) return;

    String jxStr = message.substring(2, jxSeparator);
    String jyStr = message.substring(jxSeparator + 1, jySeparator);
    String jspdX = message.substring(jySeparator + 1, jspeedXSeparator);
    String jspdY = message.substring(jspeedXSeparator + 1);
    TravelX = jxStr.toInt();
    TravelY = jyStr.toInt();
    SpeedX = jspdX.toInt();
    SpeedY = jspdY.toInt();
    if (homing_performed == true) {
      jog_now = true;
      digitalWrite(enable_pin, LOW);
    }
  } 
  else if (message.startsWith("l|")) {
    int limXSeparator = message.indexOf("|", 2);
    if (limXSeparator == -1) return;
    int limYSeparator = message.indexOf("|", limXSeparator + 1);
    if (limYSeparator == -1) return;
    int limSpdXSeparator = message.indexOf("|", limYSeparator + 1);
    if (limSpdXSeparator == -1) return;
    int limSpdYSeparator = message.indexOf("|", limSpdXSeparator + 1);
    if (limSpdYSeparator == -1) return;
    int limAccXSeparator = message.indexOf("|", limSpdYSeparator + 1);
    if (limAccXSeparator == -1) return;
    
    String limxStr = message.substring(2, limXSeparator);
    String limyStr = message.substring(limXSeparator + 1, limYSeparator);
    String limSpdX = message.substring(limYSeparator + 1, limSpdXSeparator);
    String limSpdY = message.substring(limSpdXSeparator + 1, limSpdYSeparator);
    String limAccX = message.substring(limSpdYSeparator + 1, limAccXSeparator);
    String limAccY = message.substring(limAccXSeparator + 1);
    MaxX = limxStr.toInt();
    MaxY = limyStr.toInt();
    feed_maxspeed_x = limSpdX.toInt();
    feed_maxspeed_y = limSpdY.toInt();
    feed_accel_x = limAccX.toInt();
    feed_accel_y = limAccY.toInt();
  } 
  else if (message.equals("h")) {
      home_now = true;
  } 
  else if (message.equals("c")) {
      if (homing_performed == true) {
        center_now = true;
        digitalWrite(enable_pin, LOW);
      }
  } 
  else if (message.startsWith("m|")) {
    int mxSeparator = message.indexOf("|", 2);
    if (mxSeparator == -1) return;
    int mySeparator = message.indexOf("|", mxSeparator + 1);
    if (mySeparator == -1) return;
    int mspeedXSeparator = message.indexOf("|", mySeparator + 1);
    if (mspeedXSeparator == -1) return;

    String mxStr = message.substring(2, mxSeparator);
    String myStr = message.substring(mxSeparator + 1, mySeparator);
    String mspdX = message.substring(mySeparator + 1, mspeedXSeparator);
    String mspdY = message.substring(mspeedXSeparator + 1);
    TravelX = mxStr.toInt();
    TravelY = myStr.toInt();
    SpeedX = mspdX.toInt();
    SpeedY = mspdY.toInt();
    if (homing_performed == true) {
      measure_now = true;
      digitalWrite(enable_pin, LOW);
    }
  }
}

void requestEvent() {
  if (home_found == true) {
    Wire.write(1);
    delay(100);
    Wire.write(0);
    home_found = false;
    send_position = false;
  }
  if (jog_in_pos == true) {
    Wire.write(2);
    delay(100);
    Wire.write(0);
    jog_in_pos = false;
    send_position = false;
  }
  if (over_axis_limit == true) {
    Wire.write(3);
    delay(100);
    Wire.write(0);
    over_axis_limit = false;
    send_position = false;
  }
  if (limit_switch_hit == true) {
    Wire.write(4);
    delay(100);
    Wire.write(0);
    limit_switch_hit = false;
    send_position = false;
  }
  if (center_in_pos == true) {
    Wire.write(5);
    delay(100);
    Wire.write(0);
    center_in_pos = false;
    send_position = false;
  }
  if (send_position == true) {
    a = stepperX.currentPosition();
    b = stepperY.currentPosition();
    String position = String(a) + "," + String(b) + " ";
    char buffer[position.length() + 1];
    position.toCharArray(buffer, position.length() + 1);
    Wire.write(buffer);
  }
}