#include <Wire.h>
#include <Servo.h>

#define emergencyPin 3

const int hallSensorPin = 2; // Hall effect sensor input pin
const int motorPin = 9;
Servo esc;

volatile unsigned long lastPulseTime;
volatile unsigned long pulseInterval;
volatile long rpm;
int minPWM = 1000;
int maxPWM = 2000;
int desiredPWM = 0;
int emergencyPWM = 900;
unsigned long smoothStartTime = 0; // Start time of smooth start
unsigned long smoothStartDuration = 3000; // Smooth start duration in milliseconds
bool smoothStart = false;
bool motorRunning = false;

void setup() {
  Serial.begin(115200);
  Wire.begin(15);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  pinMode(hallSensorPin, INPUT);
  pinMode(emergencyPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(hallSensorPin), countPulse, FALLING);
  esc.attach(motorPin); // Attach servo to the motor pin
  smoothStartTime = millis();
  esc.writeMicroseconds(minPWM);
}

void receiveEvent(int numBytes) {
  String command = "";
  while (Wire.available()) {
    char c = Wire.read();
    if (c == ' ') {
      break;
    } 
    command += c;
  }
  //Serial.println(command);
  parseReceivedMessage(command);
}

void parseReceivedMessage(String message) {
  if (message.startsWith("start|")) {
    int separatorIndex1 = message.indexOf("|", 5);
    if (separatorIndex1 == -1) return;

    String thr_val = message.substring(separatorIndex1 + 1);
    desiredPWM = thr_val.toInt();
    motorRunning = true;
    //Serial.println(desiredPWM);
  }
  else if (message.startsWith("stop")) {
    desiredPWM = minPWM;
    motorRunning = false;
    //Serial.println(desiredPWM);
  }
  else if (message.startsWith("min|")) {
    int separatorIndex1 = message.indexOf("|", 3);
    if (separatorIndex1 == -1) return;

    String min_val = message.substring(separatorIndex1 + 1);
    minPWM = min_val.toInt();
    //Serial.println(minPWM);
  }
  else if (message.startsWith("max|")) {
    int separatorIndex1 = message.indexOf("|", 3);
    if (separatorIndex1 == -1) return;

    String max_val = message.substring(separatorIndex1 + 1);
    maxPWM = max_val.toInt();
    //Serial.println(maxPWM);
  }
  else if (message.startsWith("test|")) {
    int separatorIndex1 = message.indexOf("|", 4);
    if (separatorIndex1 == -1) return;

    String thr_val = message.substring(separatorIndex1 + 1);
    desiredPWM = thr_val.toInt();
    motorRunning = true;
    //Serial.println(desiredPWM);
  }
}

void requestEvent() {
  String output = String(rpm);
  char buffer_out[output.length() + 1];
  output.toCharArray(buffer_out, output.length() + 1);
  Wire.write(buffer_out);
}

void loop()
{
    int out = constrain(out, minPWM, maxPWM);

    if (motorRunning) {
      unsigned long currentTime = millis();
      unsigned long elapsedTime = currentTime - smoothStartTime;

      // Calculate setpoint (desired RPM) based on smooth start
        if (elapsedTime < smoothStartDuration && smoothStart == false)
        {   
            // Smoothly increase output from 1000 to desired PWM value
            out = map(elapsedTime, 0, smoothStartDuration, minPWM, desiredPWM);
            smoothStart = true;
        }
        else
        {
            out = desiredPWM; // Set output to desired PWM value
            out = constrain(out, minPWM, maxPWM);
        }
    } else {
        out = minPWM; // Set output to minimum throttle when motor is stopped
        smoothStart = false;
        rpm = 0;
    }
    if (digitalRead(emergencyPin) == 1) {
      out = emergencyPWM;
      motorRunning = false;
    }
    esc.writeMicroseconds(out);
}

void countPulse()
{
    unsigned long currentTime = micros();
    detachInterrupt(digitalPinToInterrupt(hallSensorPin)); // Stop counting pulses
    pulseInterval = currentTime - lastPulseTime;
    lastPulseTime = currentTime;

    if (pulseInterval != 0)
    {
        rpm = 60000000UL / pulseInterval; // Calculate RPM based on pulse interval
    }
    attachInterrupt(digitalPinToInterrupt(hallSensorPin), countPulse, FALLING);
}
