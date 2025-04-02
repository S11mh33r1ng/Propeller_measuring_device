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
//const int TRQ_L_CLK = 3;
const int TRQ_CLK = 4;
const int THR_CLK = 5;
//const int TRQ_L_DO = 6; 
const int TRQ_DO = 7;
const int THR_DO = 8; 

//HX711 constructor (dout pin, sck pin)
//HX711_ADC Torque(TRQ_L_DO, TRQ_L_CLK); 
HX711_ADC Torque(TRQ_DO, TRQ_CLK);
HX711_ADC Thrust(THR_DO, THR_CLK);

bool stream_now = false;
bool stream_now_test = false;
bool init_done = false;
bool tare_now = false;
bool tare_done = false;
bool calTrq_now = false;
bool calTrq_found = false;
bool calThrust_now = false;
bool calThrust_found = false;
bool send_output = false;
bool send_output_test = false;
bool init_now = false;
bool init_thrCalVal = false;
bool init_trqCalVal = false;
bool init_setThrArmLength = false;
bool init_setTrqArmLength = false;
float g_const = 9.8066520482;
float newCalibrationValue_Trq;
float newCalibrationValue_Thr;
volatile float raw_trq;
volatile float trqArmLength;
volatile float trqCalVal; 
volatile float thrArmLength; 
volatile float thrCalVal; 
volatile long trq = 0;
volatile long thr = 0;
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
  raw_thr = Thrust.getData();
  thr = (abs((raw_thr / 1000) * g_const)/thrArmLength) * 1000;
  return thr;
}
long update_torque() { 
  Torque.update();
  raw_trq = Torque.getData();
  //Serial.println(raw_trq);
  trq = (abs((raw_trq / 1000) * g_const * (trqArmLength / 1000.0))) * 1000;
  return trq;
}

void tare_all() {
  Torque.tare();
  Thrust.tare();
  tare_now = false;
  tare_done = true;
  Serial.println("tare done");
}

void loop() {
  if (init_thrCalVal == true && init_trqCalVal == true && init_setThrArmLength == true && init_setTrqArmLength == true) {
    init_thrCalVal = false;
    init_trqCalVal = false;
    init_setThrArmLength = false;
    init_setTrqArmLength = false;
    init_now = true;
    Serial.println(init_now);
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
    //Torque.update();
    //a = Torque.getData();
    //trq = (abs((a / 1000) * g_const * (trqArmLength / 1000.0))) * 1000;
    update_torque();
    update_thrust();
    send_output_test = true;
  }
  if (init_now == true) {
    Torque.begin();
    Thrust.begin();
    //Torque.setReverseOutput();
    unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
    boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
    byte loadcell_1_rdy = 0;
    byte loadcell_2_rdy = 0;
    while ((loadcell_1_rdy + loadcell_2_rdy) < 2) { 
      if (!loadcell_1_rdy) loadcell_1_rdy = Torque.startMultiple(stabilizingtime, _tare);
      if (!loadcell_2_rdy) loadcell_2_rdy = Thrust.startMultiple(stabilizingtime, _tare);
    }
    Torque.setCalFactor(trqCalVal);
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
  else if (message.startsWith("calTrq")) { 
    calTrq_now = false;
    Torque.tare();
    Serial.println(message);
    calTrq_now = true;
  }
  else if (message.startsWith("calThrust")) { 
    calThrust_now = false;
    Thrust.tare();
    Serial.println(message);
    calThrust_now = true;
  }
  else if (message.startsWith("calMass|")) { 
    if (calTrq_now == true) {
      int separatorIndex1 = message.indexOf("|");
      if (separatorIndex1 == -1) return;
      String known_mass_TrqStr = message.substring(separatorIndex1 + 1);
      float known_mass_Trq = known_mass_TrqStr.toFloat();
      Serial.println(known_mass_Trq);
      if (known_mass_Trq != 0) {
        Torque.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
        newCalibrationValue_Trq = Torque.getNewCalibration(known_mass_Trq); //get the new calibration value
        calTrq_now = false;
        calTrq_found = true;
        Serial.println(newCalibrationValue_Trq);
        return newCalibrationValue_Trq;
      }
    }
    if (calThrust_now == true) {
      int separatorIndex1 = message.indexOf("|");
      if (separatorIndex1 == -1) return;
      String known_mass_ThrStr = message.substring(separatorIndex1 + 1);
      float known_mass_Thr = known_mass_ThrStr.toFloat();
      Serial.println(known_mass_Thr);
      if (known_mass_Thr != 0) {
        Thrust.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
        newCalibrationValue_Thr = Thrust.getNewCalibration(known_mass_Thr); //get the new calibration value
        calThrust_now = false;
        calThrust_found = true;
        Serial.println(newCalibrationValue_Thr);
        return newCalibrationValue_Thr;
      }
    }
  }
  else if (message.startsWith("setThrCalVal|")) {
    int separatorIndex1 = message.indexOf("|");
    if (separatorIndex1 == -1) return;

    String setThrCalValStr = message.substring(separatorIndex1 + 1);
    
    thrCalVal = setThrCalValStr.toFloat();
    if (init_done == true) {
      Thrust.setCalFactor(thrCalVal);
      Serial.println(thrCalVal);
    } else {
      init_thrCalVal = true;
      Serial.println(init_thrCalVal);
    } 
  }
  else if (message.startsWith("setTrqCalVal|")) {
    int separatorIndex1 = message.indexOf("|");
    if (separatorIndex1 == -1) return;

    String setTrqCalValStr = message.substring(separatorIndex1 + 1);
    
    trqCalVal = setTrqCalValStr.toFloat();
    if (init_done == true) {
      Torque.setCalFactor(trqCalVal);
      Serial.println(trqCalVal);
    } else {
      init_trqCalVal = true;
      Serial.println(init_trqCalVal);
    }
  }
  else if (message.startsWith("setThrArmLength|")) {
    int separatorIndex1 = message.indexOf("|");
    if (separatorIndex1 == -1) return;

    String setThrArmLengthStr = message.substring(separatorIndex1 + 1);
    
    thrArmLength = setThrArmLengthStr.toFloat();
    if (init_done == true) {
      Serial.println(thrArmLength);
    } else {
      init_setThrArmLength = true;
      Serial.println(init_setThrArmLength);
    }
  }
  else if (message.startsWith("setTrqArmLength|")) {
    int separatorIndex1 = message.indexOf("|");
    if (separatorIndex1 == -1) return;

    String setTrqArmLengthStr = message.substring(separatorIndex1 + 1);
    
    trqArmLength = setTrqArmLengthStr.toFloat();
    if (init_done == true) {
      Serial.println(trqArmLength);
    } else {
      init_setTrqArmLength = true;
      Serial.println(init_setTrqArmLength);
    }
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
  if (calTrq_found == true) {
    char buffer[32];
    dtostrf(newCalibrationValue_Trq, 10, 2, buffer);
    Wire.write(buffer);
    Serial.println(String(buffer));
    calTrq_found = false;
  }
  if (calThrust_found == true) {
    char buffer[32];
    dtostrf(newCalibrationValue_Thr, 10, 2, buffer);
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
    output = String(thr) + "," + String(raw_trq) + "," + String(trq) + "," + String(raw_thr);
    //Serial.println(output);
    char buffer_out[output.length() + 1];
    output.toCharArray(buffer_out, output.length() + 1);
    Wire.write(buffer_out);
    send_output_test = false;
  }
}
