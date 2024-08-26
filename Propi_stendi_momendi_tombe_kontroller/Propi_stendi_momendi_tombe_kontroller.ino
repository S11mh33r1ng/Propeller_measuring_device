#include <Wire.h>
#include <HX711_ADC.h>

//  calibrationValue_1 = 293996.12; // uncomment this if you want to set this value in the sketch
//  calibrationValue_1 = -877.59; //1 kg load cell
//  calibrationValue_1 = -897.59; // 
//  calibrationValue_2 = 295794.62; // uncomment this if you want to set this value in the sketch
//  calibrationValue_2 = 936.54; //5kg load cell
//  calibrationValue_3 = 301789.03; // 15 kg THR load celli väärtus
//  calibrationValue_3 = 878.00; //878.00 viimane väärtus 5 kg anduriga; //876.00

//pins:
const int TRQ_L_CLK = 3;
const int TRQ_R_CLK = 4;
const int THR_CLK = 5;
const int TRQ_L_DO = 6; 
const int TRQ_R_DO = 7;
const int THR_DO = 8; 

//HX711 constructor (dout pin, sck pin)
HX711_ADC Torque_L(TRQ_L_DO, TRQ_L_CLK); 
HX711_ADC Torque_R(TRQ_R_DO, TRQ_R_CLK);
HX711_ADC Thrust(THR_DO, THR_CLK);

bool measure_L = false;
bool measure_R = false;
bool stream_now = false;
bool stream_now_test = false;
bool init_done = false;
bool tare_now = false;
bool tare_done = false;
bool calLeft_now = false;
bool calLeft_found = false;
bool calRight_now = false;
bool calRight_found = false;
bool calThrust_now = false;
bool calThrust_found = false;
bool send_output = false;
bool send_output_test = false;
bool init_now = false;
bool init_thrCalVal = false;
bool init_trqLCalVal = false;
bool init_trqRCalVal = false;
bool init_setArmLength = false;
float g_const = 9.8066520482;
float newCalibrationValue_L;
float newCalibrationValue_R;
float newCalibrationValue_T;
volatile float a;
volatile float b;
volatile float armLength;
volatile float trqLCalVal; 
volatile float trqRCalVal; 
volatile float thrCalVal; 
volatile long trq = 0;
volatile long thr = 0;
volatile char armDirection;
String output;

void setup() {
  Wire.begin(5);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
}

long update_thrust() { 
  Thrust.update();
  thr = (Thrust.getData() * g_const);
  return thr;
}
long update_torque() { 
  if (measure_L == true) {
    Torque_L.update();
    a = Torque_L.getData();
    trq = (abs((a / 1000) * g_const * (armLength / 1000.0))) * 1000;
  }
  else if (measure_R == true) {
    Torque_R.update();
    b = Torque_R.getData();
    trq = (abs((b / 1000) * g_const * (armLength / 1000.0))) * 1000;
  }
  else {
    trq = 0;
  }
  return trq;
}

void tare_all() {
  Torque_L.tare();
  Torque_R.tare();
  Thrust.tare();
  tare_now = false;
  tare_done = true;
  Serial.println("tare done");
}

void loop() {
  if (init_thrCalVal == true && init_trqLCalVal == true && init_trqRCalVal == true && init_setArmLength == true) {
    init_thrCalVal = false;
    init_trqLCalVal = false;
    init_trqRCalVal = false;
    init_setArmLength = false;
    init_now = true;
  }
  if (tare_now == true && init_done == true) {
    tare_all();
  }
  if (stream_now == true && init_done == true) {
    update_thrust();
    update_torque();
    send_output = true;
  }
  if (stream_now_test == true) {
    if (measure_L == true) {
      Torque_L.update();
      a = Torque_L.getData();
      b = 0;
      trq = (abs((a / 1000) * g_const * (armLength / 1000.0))) * 1000;
    }
    else if (measure_R == true) {
      Torque_R.update();
      a = 0;
      b = Torque_R.getData();
      trq = (abs((b / 1000) * g_const * (armLength / 1000.0))) * 1000;
    }
    else {
      a = 0;
      b = 0;
      trq = 0;
    }
    update_thrust();
    send_output_test = true;
  }
  if (init_now == true) {
    Torque_L.begin();
    Torque_R.begin();
    Thrust.begin();
    Torque_R.setReverseOutput();
    unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
    boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
    byte loadcell_1_rdy = 0;
    byte loadcell_2_rdy = 0;
    byte loadcell_3_rdy = 0;
    while ((loadcell_1_rdy + loadcell_3_rdy) < 2) { 
      if (!loadcell_1_rdy) loadcell_1_rdy = Torque_L.startMultiple(stabilizingtime, _tare);
      if (!loadcell_2_rdy) loadcell_2_rdy = Torque_R.startMultiple(stabilizingtime, _tare);
      if (!loadcell_3_rdy) loadcell_3_rdy = Thrust.startMultiple(stabilizingtime, _tare);
    }
    Torque_L.setCalFactor(trqLCalVal); 
    Torque_R.setCalFactor(trqRCalVal); 
    Thrust.setCalFactor(thrCalVal); 
    init_done = true;
    init_now = false;
    Serial.println("init done");
  }
}
  
void receiveEvent(int numBytes) {
  String command = "";
  while (Wire.available()) {
    digitalWrite(LED_BUILTIN, HIGH);
    char c = Wire.read();
    digitalWrite(LED_BUILTIN, LOW);
    if (c == ' ') {
      break;
    } 
    command += c;
  }
  Serial.println(command);
  parseReceivedMessage(command);
}

void parseReceivedMessage(String message) {
  if (message.startsWith("tare")) {
    tare_now = true;
    tare_done = false;
  }
  else if (message.startsWith("calLeft")) { 
    calLeft_now = false;
    Torque_L.tare();
    Serial.println(message);
    calLeft_now = true;
  }
  else if (message.startsWith("calRight")) { 
    calRight_now = false;
    Torque_R.tare();
    Serial.println(message);
    calRight_now = true;
  }
  else if (message.startsWith("calThrust")) { 
    calThrust_now = false;
    Thrust.tare();
    Serial.println(message);
    calThrust_now = true;
  }
  else if (message.startsWith("calMass|")) { 
    if (calLeft_now == true) {
      int separatorIndex1 = message.indexOf("|", 7);
      if (separatorIndex1 == -1) return;
      String known_mass_LStr = message.substring(separatorIndex1 + 1);
      float known_mass_L = known_mass_LStr.toFloat();
      Serial.println(known_mass_L);
      if (known_mass_L != 0) {
        Torque_L.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
        newCalibrationValue_L = Torque_L.getNewCalibration(known_mass_L); //get the new calibration value
        calLeft_now = false;
        calLeft_found = true;
        Serial.println(newCalibrationValue_L);
        return newCalibrationValue_L;
      }
    }
    if (calRight_now == true) {
      int separatorIndex1 = message.indexOf("|", 7);
      if (separatorIndex1 == -1) return;
      String known_mass_RStr = message.substring(separatorIndex1 + 1);
      float known_mass_R = known_mass_RStr.toFloat();
      Serial.println(known_mass_R);
      if (known_mass_R != 0) {
        Torque_R.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
        newCalibrationValue_R = Torque_R.getNewCalibration(known_mass_R); //get the new calibration value
        calRight_now = false;
        calRight_found = true;
        Serial.println(newCalibrationValue_R);
        return newCalibrationValue_R;
      }
    }
    if (calThrust_now == true) {
      int separatorIndex1 = message.indexOf("|", 7);
      if (separatorIndex1 == -1) return;
      String known_mass_TStr = message.substring(separatorIndex1 + 1);
      float known_mass_T = known_mass_TStr.toFloat();
      Serial.println(known_mass_T);
      if (known_mass_T != 0) {
        Thrust.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
        newCalibrationValue_T = Thrust.getNewCalibration(known_mass_T); //get the new calibration value
        calThrust_now = false;
        calThrust_found = true;
        Serial.println(newCalibrationValue_T);
        return newCalibrationValue_T;
      }
    }
  }
  else if (message.startsWith("measureL")) { 
    measure_L = true;
    measure_R = false;
    Serial.println("L");
  }
  else if (message.startsWith("measureR")) { 
    measure_L = false;
    measure_R = true;
    Serial.println("R");
  }
  else if (message.startsWith("measure0")) { 
    measure_L = false;
    measure_R = false;
    Serial.println("0");
  }
  else if (message.startsWith("setThrCalVal|")) {
    int separatorIndex1 = message.indexOf("|", 12);
    if (separatorIndex1 == -1) return;

    String setThrCalValStr = message.substring(separatorIndex1 + 1);
    
    thrCalVal = setThrCalValStr.toFloat();
    if (init_done == true) {
      Thrust.setCalFactor(thrCalVal);
      Serial.println(thrCalVal);
    } else {
      init_thrCalVal = true;
    } 
  }
  else if (message.startsWith("setTrqLCalVal|")) {
    int separatorIndex1 = message.indexOf("|", 13);
    if (separatorIndex1 == -1) return;

    String setTrqLCalValStr = message.substring(separatorIndex1 + 1);
    
    trqLCalVal = setTrqLCalValStr.toFloat();
    if (init_done == true) {
      Torque_L.setCalFactor(trqLCalVal);
    } else {
      init_trqLCalVal = true;
    }
  }
  else if (message.startsWith("setTrqRCalVal|")) {
    int separatorIndex1 = message.indexOf("|", 13);
    if (separatorIndex1 == -1) return;

    String setTrqRCalValStr = message.substring(separatorIndex1 + 1);
    
    trqRCalVal = setTrqRCalValStr.toFloat();
    if (init_done == true) {
      Torque_R.setCalFactor(trqRCalVal);
    } else {
      init_trqRCalVal = true;
    }
  }
  else if (message.startsWith("setArmLength|")) {
    int separatorIndex1 = message.indexOf("|", 11);
    if (separatorIndex1 == -1) return;

    String setArmLengthStr = message.substring(separatorIndex1 + 1);
    
    armLength = setArmLengthStr.toFloat();
    init_setArmLength = true;
    Serial.println(armLength);
  }
  if (message.startsWith("streamStart")) {
    stream_now = true;
  }
  if (message.startsWith("streamStop")) {
    stream_now = false;
    send_output = false;
  } 
  if (message.startsWith("ON")) {
    stream_now_test = true;
  }
  if (message.startsWith("OFF")) {
    stream_now_test = false;
    send_output_test = false;
  } 
}

void requestEvent() {
  if (tare_done == true) {
    Wire.write(6);
    delay(100);
    Wire.write(0);
    tare_done = false;
  }
  if (calLeft_found == true) {
    char buffer[32];
    dtostrf(newCalibrationValue_L, 10, 2, buffer);
    Wire.write(buffer);
    Serial.println(String(buffer));
    calLeft_found = false;
  }
  if (calRight_found == true) {
    char buffer[32];
    dtostrf(newCalibrationValue_R, 10, 2, buffer);
    Wire.write(buffer);
    Serial.println(String(buffer));
    calRight_found = false;
  }
  if (calThrust_found == true) {
    char buffer[32];
    dtostrf(newCalibrationValue_T, 10, 2, buffer);
    Wire.write(buffer);
    Serial.println(String(buffer));
    calThrust_found = false;
  }
  if (send_output == true) {
    output = String(thr) + "," + String(trq);
    //Serial.println(output);
    char buffer_out[output.length() + 1];
    output.toCharArray(buffer_out, output.length() + 1);
    Wire.write(buffer_out);
    send_output = false;
  }
  if (send_output_test == true) {
    if (measure_L == true) {
      output = String(thr) + "," + String(a) + "," + String(trq);
      Serial.println(output);
      char buffer_out[output.length() + 1];
      output.toCharArray(buffer_out, output.length() + 1);
      Wire.write(buffer_out);
    }
    if (measure_R == true) {
      output = String(thr) + "," + String(b) + "," + String(trq);
      Serial.println(output);
      char buffer_out[output.length() + 1];
      output.toCharArray(buffer_out, output.length() + 1);
      Wire.write(buffer_out);
    }
    else if (measure_L == false && measure_R == false) {
      output = String(thr) + ",0,0";
      Serial.println(output);
      char buffer_out[output.length() + 1];
      output.toCharArray(buffer_out, output.length() + 1);
      Wire.write(buffer_out);
    }
    send_output_test = false;
  }
}
