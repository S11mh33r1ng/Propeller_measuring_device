#include <rtos.h>
#include <mbed.h>
#include <Servo.h>
#include <Thread.h>
#include <ThreadController.h>
#include "RpmController.h"
#include "ThrustTorqueController.h"

struct InitParams;

volatile float airspeed_raw;
volatile float aoa_raw;
volatile float aoss_raw;
volatile float static_pressure_raw;
volatile bool pitot_data_valid = false;

// -----------------------------------------------------------------------------
// AoA trim-relative math helpers
//   - trimDeg is treated as "0 deg" reference
//   - commanded values from pitot/controller are +/- relative to trim
// -----------------------------------------------------------------------------
static float wrapSignedDeg(float deg) {
  while (deg <= -180.0f) deg += 360.0f;
  while (deg >  180.0f)  deg -= 360.0f;
  return deg;
}

static float aoaPhysicalDegFromRelative(float trimDeg, float relDeg) {
  // physical = trim + relative, wrapped to [-180..180]
  return wrapSignedDeg(trimDeg + relDeg);
}

static float aoaRelativeDegFromPhysical(float trimDeg, float physicalDeg) {
  // relative = physical - trim, wrapped to [-180..180]
  return wrapSignedDeg(physicalDeg - trimDeg);
}


// -----------------------------------------------------------------------------
// Init packet parsed parameters
// -----------------------------------------------------------------------------
// Some versions of this sketch use a helper (initControllersFrom) that takes a
// parsed init struct. The current build may not call it, but we keep this type
// so the sketch compiles cleanly.
struct InitParams {
  float fTrqArm = 0.0f;
  float fThrArm = 0.0f;
  float fTrqCal = 0.0f;
  float fThrCal = 0.0f;
  float sTrqArm = 0.0f;
  float sThrArm = 0.0f;
  float sTrqCal = 0.0f;
  float sThrCal = 0.0f;
  int   tSetup  = 1;
};

// Parse INIT payload.
// Expected 9 numeric values, separated by commas/spaces/semicolons:
//   fTrqArm,fThrArm,fTrqCal,fThrCal,sTrqArm,sThrArm,sTrqCal,sThrCal,tSetup
static bool parseInitPayload(const String& payload, InitParams& out) {
  float v[9];
  int n = 0;

  auto isSep = [](char c) {
    return (c == ',' || c == ';' || c == ' ' || c == '|' || c == '\t' || c == '\r' || c == '\n');
  };

  int i = 0;
  while (i < payload.length() && n < 9) {
    // Skip separators
    while (i < payload.length() && isSep(payload[i])) i++;
    if (i >= payload.length()) break;

    int start = i;
    while (i < payload.length() && !isSep(payload[i])) i++;

    String tok = payload.substring(start, i);
    tok.trim();
    if (tok.length() == 0) continue;

    v[n++] = tok.toFloat();
  }

  if (n != 9) return false;

  out.fTrqArm = v[0];
  out.fThrArm = v[1];
  out.fTrqCal = v[2];
  out.fThrCal = v[3];
  out.sTrqArm = v[4];
  out.sThrArm = v[5];
  out.sTrqCal = v[6];
  out.sThrCal = v[7];
  out.tSetup  = v[8];
  return true;
}


// -----------------------------------------------------------------------------
// CAN AoA servo (HITEC MDB961WP) configuration
// -----------------------------------------------------------------------------
#ifndef CAN_SERVO_BAUD
#define CAN_SERVO_BAUD 1000000
#endif

// Servo CAN settings (MDB961WP)
static constexpr uint8_t  AOA_CAN_SERVO_ID = 1;     // servo ID
static constexpr uint16_t AOA_CAN_ARB_ID = 0x001; // CAN BUS ID (11-bit). Your sniff showed 0x1. CAN_SERVO_ARB_ID

static constexpr uint8_t REG_POSITION_NEW = 0x1E;  // command position (new format)
static constexpr uint8_t REG_POSITION     = 0x0C;  // position now

// Servo frame is 180deg rotated vs your physical trim reference
static constexpr float SERVO_FRAME_OFFSET_DEG = 180.0f;
static constexpr float SERVO_COUNTS_PER_REV   = 16384.0f;
static constexpr float SERVO_DEG_PER_REV      = 360.0f;

// -----------------------------------------------------------------------------
// Servo motion smoothing
// -----------------------------------------------------------------------------
// These limits smooth the CAN position commands without changing the final target.
// Tune max acceleration first: lower values reduce bench shaking.
static constexpr uint32_t SERVO_CMD_PERIOD_MS = 10;    // 100 Hz command update for smaller visible steps
static constexpr float SERVO_CMD_EPS_DEG      = 0.02f; // final-position epsilon

// Minimum command movement before sending another CAN target.
// This reduces tiny uneven updates that can feel like twitching/jerk.
static constexpr float AOA_MIN_TX_DELTA_DEG   = 0.0f;  // v4: disabled; send every changed servo count
static constexpr float AOSS_MIN_TX_DELTA_DEG  = 0.0f;  // v4: disabled; AoSS ratio made this visibly steppy

static constexpr float AOA_MAX_VEL_DEG_S      = 100.0f;
static constexpr float AOA_MAX_ACCEL_DEG_S2   = 160.0f;

// AoSS tube angle is multiplied by AoSSratio at the servo shaft, so use gentler
// tube-angle limits here. Increase only if the bench stays stable.
static constexpr float AOSS_MAX_VEL_DEG_S     = 35.0f;
static constexpr float AOSS_MAX_ACCEL_DEG_S2  = 30.0f;

struct MotionProfile {
  float posDeg = 0.0f;
  float velDegS = 0.0f;

  // Keep this as a struct method, not a separate function.
  // Arduino's .ino preprocessor can generate function prototypes before this
  // struct is declared, causing compile errors with MotionProfile references.
  float moveTo(float targetDeg, float maxVelDegS, float maxAccelDegS2, uint32_t dtMs) {
    float dt = (float)dtMs / 1000.0f;
    if (dt <= 0.0f || maxVelDegS <= 0.0f || maxAccelDegS2 <= 0.0f) {
      return posDeg;
    }

    float error = targetDeg - posDeg;
    if (fabsf(error) < 0.0005f && fabsf(velDegS) < 0.0005f) {
      posDeg = targetDeg;
      velDegS = 0.0f;
      return posDeg;
    }

    float dir = (error >= 0.0f) ? 1.0f : -1.0f;
    float stoppingDist = (velDegS * velDegS) / (2.0f * maxAccelDegS2);

    if (fabsf(error) > stoppingDist) {
      velDegS += dir * maxAccelDegS2 * dt;
    } else {
      velDegS -= dir * maxAccelDegS2 * dt;
    }

    velDegS = constrain(velDegS, -maxVelDegS, maxVelDegS);

    float step = velDegS * dt;
    if (fabsf(step) >= fabsf(error)) {
      posDeg = targetDeg;
      velDegS = 0.0f;
    } else {
      posDeg += step;
    }

    return posDeg;
  }
};

static volatile bool aoaServoSettled = true;
static volatile bool aossServoSettled = true;

static constexpr uint8_t  AOSS_CAN_SERVO_ID = 2;
static constexpr uint16_t AOSS_CAN_ARB_ID = 0x002;

static constexpr uint8_t REG_TURN_COUNT   = 0x18;  // signed turns (int16)
static constexpr uint8_t REG_POSITION_NOW = 0x0C;  // same as REG_POSITION
static constexpr uint8_t REG_TURN_NEW = 0x24;  // command turn in TURN mode
static constexpr uint8_t REG_RUN_MODE     = 0x44;
static constexpr uint8_t REG_CONFIG_SAVE  = 0x70;
static constexpr uint8_t REG_POWER_CONFIG = 0x46;

// HITEC REG_POWER_CONFIG emergency-stop bits 10:9.
// 0x0200 releases motor torque, 0x0000 returns to normal servo control.
static constexpr uint16_t POWER_CONFIG_NORMAL     = 0x0000;
static constexpr uint16_t POWER_CONFIG_MOTOR_FREE = 0x0200;
static constexpr uint16_t POWER_CONFIG_MOTOR_HOLD = 0x0600;

static volatile uint16_t aossServoPosNow = 0;
static volatile int16_t  aossTurnNow     = 0;

static volatile float    aossServoDegNow = 0.0f;   // continuous servo shaft angle (deg)
static volatile float    aossTubeDegNow  = 0.0f;   // calibrated pitot tube physical angle (deg)

static float AoSSratio = 13.3f;   // can be changed at runtime via command if you want

// AoSS calibration + state
static float aossZeroOffsetDeg = 0.0f;    // servo continuous deg at tube=0
static int   aossDir = +1;                // +1 or -1

static float aossSensorRelAngle = 0.0f;   // commanded tube angle (deg, trim-relative)
static float aossMeasuredRelDeg = 0.0f;   // measured tube angle (deg, trim-relative)
static float aossTrimDeg = 0.0f;
static volatile float aossPitotDegNow = 0.0f;
static volatile float aossPitotDegTrimmed = 0.0f;

// Asymmetric tube travel limits (tube degrees)
static float aossMinTubeDeg = -32.0f;
static float aossMaxTubeDeg = 0.0f;

// Portenta CAN1 pins need converting to PinName
static mbed::CAN canServoBus(
  digitalPinToPinName(CAN1_RX),
  digitalPinToPinName(CAN1_TX),
  CAN_SERVO_BAUD
);

// State for AoA CAN servo
static volatile uint16_t aoaServoPosCmd = 0;
static volatile uint16_t aoaServoPosNow = 0;
static volatile float    aoaMeasuredRelDeg = 0.0f;  // signed deg relative to trim
static float             aoaSensorRelAngle = 0.0f;  // commanded relative angle (signed)

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

// -----------------------------------------------------------------------------
// CAN AoA servo helpers (MDB961WP)
// -----------------------------------------------------------------------------
static float wrapDeg(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

// // Map a difference to [-180, +180)
// static float wrapSignedDeg(float deg) {
//   deg = fmodf(deg, 360.0f);
//   if (deg >= 180.0f) deg -= 360.0f;
//   if (deg <  -180.0f) deg += 360.0f;
//   return deg;
// }

static void aossCanCommandServoTubeDeg_NoTurn(float tubeRelDeg) {
  float servoDeg = aossServoDegFromTubeDeg(tubeRelDeg);   // your existing mapping
  float within = fmodf(servoDeg, 360.0f);
  if (within < 0) within += 360.0f;
  uint16_t pos = degToCounts360(within);
  hitecWriteReg16(AOSS_CAN_ARB_ID, AOSS_CAN_SERVO_ID, REG_POSITION_NEW, pos);
}

static uint16_t degToCounts360(float deg) {
  deg = wrapDeg(deg);
  float c = deg * (SERVO_COUNTS_PER_REV / SERVO_DEG_PER_REV);
  if (c < 0.0f) c = 0.0f;
  if (c > 16383.0f) c = 16383.0f;
  return (uint16_t)(c + 0.5f);
}

static float countsToDeg360(uint16_t counts) {
  return ((float)counts) * (SERVO_DEG_PER_REV / SERVO_COUNTS_PER_REV);
}

static float physicalToServoFrameDeg(float physicalDeg) {
  return wrapDeg(physicalDeg + SERVO_FRAME_OFFSET_DEG);
}

static float servoFrameToPhysicalDeg(float servoDeg) {
  return wrapDeg(servoDeg - SERVO_FRAME_OFFSET_DEG);
}

static float aossPosCountsToDeg(uint16_t posCounts) {
  // Same scaling as your AoA: 16384 counts = 360 deg
  return countsToDeg360(posCounts);
}

static float aossMultiTurnServoDeg(int16_t turns, uint16_t posCounts) {
  return (float)turns * 360.0f + aossPosCountsToDeg(posCounts);
}

static float aossTubeDegFromServoDeg(float servoDeg) {
  // Convert CONTINUOUS servo shaft degrees -> tube degrees (trim-relative, 0 at trim)
  // AoSSratio = servoDeg / tubeDeg
  return (float)aossDir * (servoDeg - aossZeroOffsetDeg) / AoSSratio;
}

static float aossServoDegFromTubeDeg(float tubeDeg) {
  // Convert tube trim-relative degrees -> CONTINUOUS servo shaft degrees
  return aossZeroOffsetDeg + (float)aossDir * tubeDeg * AoSSratio;
}

// Set AoSS "zero": compute aossZeroOffsetDeg so that the CURRENT trimmed pitot AoSS becomes 0 deg.
// After calling this, commanding aossCanCommandServoRelDeg(0) will drive the tube to the trimmed-zero.
static void aossSetZeroFromCurrent() {
  // Make sure we have a fresh servo readback
  aossCanReadbackUpdate();
  
  float pitotAoss;
  pitotMutex.lock();
  pitotAoss = aoss_raw;
  pitotMutex.unlock();

  float tubeTrimmed = pitotAoss - aossTrimDeg;

  // Current servo continuous angle should correspond to current tubeTrimmed.
  // servoDegNow = aossZeroOffsetDeg + dir * tubeTrimmed * AoSSratio
  // => aossZeroOffsetDeg = aossServoDegNow - dir * tubeTrimmed * AoSSratio
  aossZeroOffsetDeg = aossServoDegNow - (float)aossDir * tubeTrimmed * AoSSratio;

  Serial.print("AoSS zero set. aossZeroOffsetDeg=");
  Serial.print(aossZeroOffsetDeg, 3);
  Serial.print(" trimDeg=");
  Serial.print(aossTrimDeg, 3);
  Serial.print(" pitotTrimmed=");
  Serial.println(tubeTrimmed, 3);
}

static void canFlushRx() {
  mbed::CANMessage rx;
  while (canServoBus.read(rx)) { /* discard */ }
}

static void hitecWriteReg16(uint16_t arbId, uint8_t servoId, uint8_t regAddr, uint16_t value) {
  uint8_t d[8] = {0};
  d[0] = 'w';
  d[1] = servoId;
  d[2] = regAddr;
  d[3] = (uint8_t)(value & 0xFF);
  d[4] = (uint8_t)((value >> 8) & 0xFF);

  mbed::CANMessage msg(arbId, (const char*)d, 5, CANData, CANStandard);
  (void)canServoBus.write(msg);
}


static void aoaServoMotorFree() {
  hitecWriteReg16(AOA_CAN_ARB_ID, AOA_CAN_SERVO_ID, REG_POWER_CONFIG, POWER_CONFIG_MOTOR_FREE);
  Serial.println("AoA servo motor FREE command sent");
}

static void aoaServoMotorEnable() {
  hitecWriteReg16(AOA_CAN_ARB_ID, AOA_CAN_SERVO_ID, REG_POWER_CONFIG, POWER_CONFIG_NORMAL);
  Serial.println("AoA servo motor ENABLE command sent");
}

static void aoaServoMotorHold() {
  hitecWriteReg16(AOA_CAN_ARB_ID, AOA_CAN_SERVO_ID, REG_POWER_CONFIG, POWER_CONFIG_MOTOR_HOLD);
  Serial.println("AoA servo motor HOLD command sent");
}

static void hitecWriteReg16x2(uint16_t arbId, uint8_t servoId,
                             uint8_t regA, uint16_t valA,
                             uint8_t regB, uint16_t valB) {
  uint8_t d[8] = {0};
  d[0] = 'W';        // write 2 registers in one packet
  d[1] = servoId;

  d[2] = regA;
  d[3] = (uint8_t)(valA & 0xFF);
  d[4] = (uint8_t)((valA >> 8) & 0xFF);

  d[5] = regB;
  d[6] = (uint8_t)(valB & 0xFF);
  d[7] = (uint8_t)((valB >> 8) & 0xFF);

  mbed::CANMessage msg(arbId, (const char*)d, 8, CANData, CANStandard);
  (void)canServoBus.write(msg);
}

static bool hitecReadReg16(uint16_t arbId, uint8_t servoId, uint8_t regAddr, uint16_t &outValue, uint32_t timeoutMs = 50) {
  canFlushRx();

  uint8_t d[8] = {0};
  d[0] = 'r';
  d[1] = servoId;
  d[2] = regAddr;

  mbed::CANMessage tx(arbId, (const char*)d, 3, CANData, CANStandard);
  if (!canServoBus.write(tx)) return false;

  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    mbed::CANMessage rx;
    if (canServoBus.read(rx)) {
      // Expected response: ['v' or 'V', SERVO_ID, REG_ADDR, DATA_L, DATA_H, ...]
      if (rx.len >= 5 &&
          ((uint8_t)rx.data[0] == 'v' || (uint8_t)rx.data[0] == 'V') &&
          (uint8_t)rx.data[1] == servoId &&
          (uint8_t)rx.data[2] == regAddr) {
        uint8_t lo = (uint8_t)rx.data[3];
        uint8_t hi = (uint8_t)rx.data[4];
        outValue = (uint16_t)(lo | (hi << 8));
        return true;
      }
    }
  }
  return false;
}

static uint16_t physicalDegToServoCounts(float physicalDeg) {
  float servoDeg = physicalToServoFrameDeg(physicalDeg);
  return degToCounts360(servoDeg);
}

static float servoCountsToPhysicalDeg(uint16_t counts) {
  float servoDeg = countsToDeg360(counts);
  return servoFrameToPhysicalDeg(servoDeg);
}

// Command expects PHYSICAL degrees (your trim reference frame).
// The target is still exact, but the command sent to the servo is acceleration-
// limited so the bench does not receive a sharp step input.
static void aoaCanCommandServoDeg(float physicalDegTarget) {
  static uint32_t lastUpdate = 0;
  static uint32_t lastTx = 0;
  static uint16_t lastCmd = 0xFFFF;
  static float lastTxDeg = NAN;
  static MotionProfile motion;

  uint32_t now = millis();

  if (lastUpdate == 0) {
    lastUpdate = now;
    motion.posDeg = physicalDegTarget;
    motion.velDegS = 0.0f;
  }

  uint32_t dt = now - lastUpdate;
  if (dt == 0) return;
  lastUpdate = now;

  // Use the shortest angular path across 0/360 deg.
  float targetUnwrapped = motion.posDeg + wrapSignedDeg(physicalDegTarget - motion.posDeg);
  float physicalDegCmd = motion.moveTo(targetUnwrapped,
                                   AOA_MAX_VEL_DEG_S,
                                   AOA_MAX_ACCEL_DEG_S2,
                                   dt);

  uint16_t pos = physicalDegToServoCounts(physicalDegCmd);
  aoaServoPosCmd = pos;

  aoaServoSettled = (fabsf(wrapSignedDeg(physicalDegTarget - physicalDegCmd)) < 0.2f &&
                     fabsf(motion.velDegS) < 2.0f);

  bool forceFinalTx = fabsf(wrapSignedDeg(physicalDegTarget - physicalDegCmd)) < SERVO_CMD_EPS_DEG;
  bool firstTx = isnan(lastTxDeg);

  // v4: send every changed servo count at a steady 100 Hz. The previous min-delta
  // skipped too many small commands, which made the tube move in visible chunks.
  if ((now - lastTx) >= SERVO_CMD_PERIOD_MS &&
      (firstTx || pos != lastCmd || (forceFinalTx && pos != lastCmd))) {
    hitecWriteReg16(AOA_CAN_ARB_ID, AOA_CAN_SERVO_ID, REG_POSITION_NEW, pos);
    lastCmd = pos;
    lastTxDeg = physicalDegCmd;
    lastTx = now;
  }
}

static void aossCanCommandServoRelDeg(float tubeRelDegTarget) {
  static uint32_t lastUpdate = 0;
  static uint32_t lastTx = 0;
  static int16_t lastTurn = -32768;
  static uint16_t lastPos = 0xFFFF;
  static float lastTxTubeDeg = NAN;
  static MotionProfile motion;

  uint32_t now = millis();

  if (lastUpdate == 0) {
    lastUpdate = now;
    motion.posDeg = tubeRelDegTarget;
    motion.velDegS = 0.0f;
  }

  uint32_t dt = now - lastUpdate;
  if (dt == 0) return;
  lastUpdate = now;

  float tubeRelDegCmd = motion.moveTo(tubeRelDegTarget,
                                  AOSS_MAX_VEL_DEG_S,
                                  AOSS_MAX_ACCEL_DEG_S2,
                                  dt);

  // tubeRelDegCmd = desired pitot tube physical angle relative to trim (deg)
  float servoDegCont = aossServoDegFromTubeDeg(tubeRelDegCmd);

  // Convert continuous servo degrees -> turn + position counts
  int16_t turns = (int16_t)floorf(servoDegCont / 360.0f);
  float within = servoDegCont - (float)turns * 360.0f;
  if (within < 0.0f) { within += 360.0f; turns -= 1; }

  uint16_t pos = degToCounts360(within);
  uint16_t turnRaw = (uint16_t)(int16_t)turns;  // preserve negatives

  aossServoSettled = (fabsf(tubeRelDegTarget - tubeRelDegCmd) < 0.2f &&
                      fabsf(motion.velDegS) < 1.0f);

  bool forceFinalTx = fabsf(tubeRelDegTarget - tubeRelDegCmd) < SERVO_CMD_EPS_DEG;
  bool firstTx = isnan(lastTxTubeDeg);

  // v4: send every changed servo count/turn at a steady 100 Hz. The previous
  // AoSS min-delta was in tube degrees; after AoSSratio it became visible servo steps.
  if ((now - lastTx) >= SERVO_CMD_PERIOD_MS &&
      (firstTx || turns != lastTurn || pos != lastPos ||
       (forceFinalTx && (turns != lastTurn || pos != lastPos)))) {
    hitecWriteReg16x2(AOSS_CAN_ARB_ID, AOSS_CAN_SERVO_ID,
                      REG_TURN_NEW,     turnRaw,
                      REG_POSITION_NEW, pos);
    lastTurn = turns;
    lastPos = pos;
    lastTxTubeDeg = tubeRelDegCmd;
    lastTx = now;
  }
}

// Readback actual position and update aoaMeasuredRelDeg (signed relative to trimAoA)
extern float trimAoA;

void aoaCanReadbackUpdate() {
  static uint32_t lastRx = 0;
  uint32_t now = millis();
  if ((now - lastRx) < 50) return; // 20 Hz
  lastRx = now;

  uint16_t pos = 0;
  if (hitecReadReg16(AOA_CAN_ARB_ID, AOA_CAN_SERVO_ID, REG_POSITION, pos, 20)) {
    aoaServoPosNow = pos;
    float nowPhys = servoCountsToPhysicalDeg(pos);
    aoaMeasuredRelDeg = wrapSignedDeg(nowPhys - trimAoA);
  }
}

static void aossCanReadbackUpdate() {
  static uint32_t lastRx = 0;
  uint32_t now = millis();
  if ((now - lastRx) < 50) return; // 20 Hz
  lastRx = now;

  uint16_t posU = 0;
  uint16_t turnU = 0;

  // Read intra-turn position + turn count
  if (hitecReadReg16(AOSS_CAN_ARB_ID, AOSS_CAN_SERVO_ID, REG_POSITION_NOW, posU, 20) &&
      hitecReadReg16(AOSS_CAN_ARB_ID, AOSS_CAN_SERVO_ID, REG_TURN_COUNT,   turnU, 20)) {

    int16_t turns = (int16_t)turnU;

    aossServoPosNow = posU;
    aossTurnNow = turns;

    float servoDeg = aossMultiTurnServoDeg(turns, posU);
    aossServoDegNow = servoDeg;

    float tubeDeg = aossTubeDegFromServoDeg(servoDeg);
    aossTubeDegNow = tubeDeg;
    aossMeasuredRelDeg = tubeDeg; // servo-derived tube angle (trim-relative)
  }
}

static void aossEnableTurnModeAndReset() {
  // 1) Set Multi-Turn mode
  hitecWriteReg16(AOSS_CAN_ARB_ID, AOSS_CAN_SERVO_ID, REG_RUN_MODE, 0);

  delay(10);

  // 2) Save configuration (write 0xFFFF)
  hitecWriteReg16(AOSS_CAN_ARB_ID, AOSS_CAN_SERVO_ID, REG_CONFIG_SAVE, 0xFFFF);

  delay(50);

  // 3) Software reset (bit0 = 1)
  hitecWriteReg16(AOSS_CAN_ARB_ID, AOSS_CAN_SERVO_ID, REG_POWER_CONFIG, 0x0001);

  // give it time to reboot
  delay(500);
}

static void aossPrintRunMode() {
  uint16_t v = 0;
  bool ok = hitecReadReg16(AOSS_CAN_ARB_ID, AOSS_CAN_SERVO_ID, REG_RUN_MODE, v, 100);
  Serial.print("AoSS RUN_MODE ok="); Serial.print(ok);
  Serial.print(" value="); Serial.println(v);
}

static void ensureAossLimitOrder(float &mn, float &mx) {
  if (mn > mx) {
    float t = mn;
    mn = mx;
    mx = t;
  }
}

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

Servo aoaServo; // (unused: replaced by CAN servo) 
Servo aossServo;
Servo esc1;
Servo esc2;

ButtonState safetySwitchState = BUTTON_UP;      // INPUT_PULLUP -> idle = HIGH
ButtonState emergencyButtonState = BUTTON_UP;   // idle/unpressed

bool limits_sent = false;
bool read_stream = false;
bool pitot_live_mode = false;       // When true, keep reading Pitot but do not move AoA/AoSS arms
bool pitot_live_stream = false;     // When true, print live Pitot data every received Pitot frame
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
bool aoa_enabled = true;
static constexpr float AOA_DISABLED_REL_DEG = -90.0f;  // Disabled AoA parks 90 deg down from trim
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
float trimAoA = -4.0;
int trimAoSS = 105;
int limAoA = 50;
int limMaxAoSS = 56;
int limMinAoSS = 108;
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
const float adjustmentStepAoA = 0.125;
const float adjustmentStepAoSS = 0.05;
//const float AoSSratio = 1.419;          //ratio between the degree command given and actual degrees (i.e. command is 44 deg, but actual movement of the probe is 31 deg)
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
void printPitotLive(const char* prefix);


// Convert a relative AoA measurement (deg, relative to trimAoA) to absolute AoA (deg).
static float calculateAbsoluteAoA(float aoaRelDeg) {
  return aoaRelDeg + trimAoA;
}

static float aoaTargetPhysicalDegForAxisState() {
  if (aoa_enabled) {
    // Normal AoA controller demand is limited by limAoA.
    return wrapSignedDeg(trimAoA + constrain(aoaSensorRelAngle, -(float)limAoA, (float)limAoA));
  }

  // Disabled park position intentionally bypasses limAoA.
  // Keep this signed so trimAoA=-4 and AOA_DISABLED_REL_DEG=-90 prints/commands -94,
  // not 266. The servo conversion later handles the 0..360 register format.
  return wrapSignedDeg(trimAoA + AOA_DISABLED_REL_DEG);
}

void setAoAAxisEnabled(bool enabled) {
  aoa_enabled = enabled;

  if (!aoa_enabled) {
    // Disable AoA control demand and release motor torque. This does not change
    // the servo's stored zero/center; it only stops the motor from holding.
    aoaSensorRelAngle = 0.0f;
    aoaServoMotorFree();
    Serial.println("AoA axis DISABLED: servo motor free, no position commands");
    return;
  }

  // Re-enable motor control and command the original AoA zero/trim position.
  // Do NOT adopt the free-moved position as a new zero. Motor_Free does not
  // erase the servo calibration; this restores the old trim reference.
  aoaServoMotorEnable();
  delay(20);

  aoaSensorRelAngle = 0.0f;
  aoaSensorPhysicalAngle = wrapSignedDeg(trimAoA);
  aoaCanCommandServoDeg(aoaSensorPhysicalAngle);

  Serial.print("AoA axis ENABLED: returning to old zero/trim physical deg: ");
  Serial.print(aoaSensorPhysicalAngle, 2);
  Serial.print(" rel-to-trim deg: ");
  Serial.println(aoaSensorRelAngle, 2);
}

bool isAoAAxisEnabled() {
  return aoa_enabled;
}

void parkAoAAxisDisabled() {
  // In disabled mode the AoA servo is motor-free, so do not send any position
  // demand. Keep the software relative demand reset for the next enable.
  aoaSensorRelAngle = 0.0f;
}

void aoaController() {
  float aoa, airspeed;

  pitotMutex.lock();
  aoa = aoa_raw;
  airspeed = airspeed_raw;
  pitotMutex.unlock();

  if (!aoa_enabled) {
    // Motor is free. Do not send AoA position commands while disabled; only read
    // back the actual position if the servo still reports it.
    parkAoAAxisDisabled();
  } else if (!pitot_live_mode) {
    adjustSensorAoAPosition(aoa, airspeed);
  }

  aoaCanReadbackUpdate();
  aossCanReadbackUpdate();
  absAoA = aoaMeasuredRelDeg;
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

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(safety_disconnected, INPUT_PULLUP);
  pinMode(emergency_pressed, INPUT_PULLUP);
  pinMode(emergency_stepper, OUTPUT);
  pinMode(beacon_output, OUTPUT);

  setAoAAxisEnabled(true);  // Set AoA CAN servo to trim (physical) to 90-degree position
  esc1.attach(first_esc);
  esc1.writeMicroseconds(ESC_MIN_US);
  esc2.attach(second_esc);
  esc2.writeMicroseconds(ESC_MIN_US);
  aossEnableTurnModeAndReset();
  delay(50);
  
  Serial.begin(115200); //Serial between UI
  Serial3.begin(115200); //Serial between Pitot' sensor
  Wire.begin(); //I2C to stepper controller

  // Assume AoSS is physically at 0 on boot (axis not manually movable).
  // We take current servo absolute (turn+pos) as the zero reference.
  aossCanReadbackUpdate();
  aossZeroOffsetDeg = aossServoDegNow;   // "this is tube=0"
  aossSensorRelAngle = 0.0f;

  Serial.print("AoSS boot zero assumed. aossZeroOffsetDeg=");
  Serial.println(aossZeroOffsetDeg, 2);

  thrustTorqueControllers[0] = new ThrustTorqueController(0);

  thread1.onRun(manageCommands);
  thread1.setInterval(0); 
  thread2->onRun(testReadThrTrq);
  thread2->setInterval(100); 
  thread3->onRun(readPitot); 
  thread3->setInterval(0); 
  thread4->onRun(aoaController); 
  thread4->setInterval(10); 
  thread5->onRun(assembleStream); 
  thread5->setInterval(5); //changing this value changes the resolution of the readings, don't set it to 0! 5 default
  thread6->onRun(checkEmergency); 
  thread6->setInterval(10);
  thread7->onRun(escPwmController); 
  thread7->setInterval(10);
  thread8->onRun(aossController); 
  thread8->setInterval(10);
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

/*// If nothing is coming from the Pitot serial, just force zeros
  if (Serial3.available() == 0) {
    pitotMutex.lock();
    airspeed_raw = 0.0f;
    aoa_raw      = 0.0f;
    aoss_raw     = 0.0f;
    static_pressure_raw = 0.0f;
    pitot_data_valid = false;
    pitotMutex.unlock();

    got_pitot_readings = true;

    if (read_stream) {
      Serial.println("Pitot: 0 0 0 0");
    }
    return;
  }
*/
  
  // Try to read one line from Pitot
  String pitot = Serial3.readStringUntil('\n'); 

  int time     = pitot.indexOf(',');
  int date     = pitot.indexOf(',', time + 1);
  int as       = pitot.indexOf(',', date + 1);
  int ias      = pitot.indexOf(',', as + 1);
  int aoa_pos  = pitot.indexOf(',', ias + 1);
  int aoss_pos = pitot.indexOf(',', aoa_pos + 1);
  int pa       = pitot.indexOf(',', aoss_pos + 1);
  int sp       = pitot.indexOf(',', pa + 1);
  int tp       = pitot.indexOf(',', sp + 1);
  int cs       = pitot.indexOf(',', tp + 1);

  if (time != -1 && date != -1 && as != -1 && ias != -1 &&
      aoa_pos != -1 && aoss_pos != -1 && pa != -1 && sp != -1) {
    String airSpeedStr       = pitot.substring(date + 1, as);
    String aoaStr            = pitot.substring(ias + 1, aoa_pos);
    String aossStr           = pitot.substring(aoa_pos + 1, aoss_pos);
    String staticPressureStr = pitot.substring(pa + 1, sp);

    pitotMutex.lock();
    airspeed_raw        = airSpeedStr.toFloat();
    aoa_raw             = aoaStr.toFloat();
    aoss_raw            = aossStr.toFloat();
    static_pressure_raw = staticPressureStr.toFloat();
    pitot_data_valid    = true;
    pitotMutex.unlock();
  } else {
    pitotMutex.lock();
    pitot_data_valid = false;
    pitotMutex.unlock();
  }
  
  got_pitot_readings = true;

  if (read_stream) {
    Serial.println("Pitot: " + String(airspeed_raw) + " " + String(aoa_raw) + " " + String(aoss_raw) + " " + String(static_pressure_raw));
  }

  if (pitot_live_stream) {
    printPitotLive("pitotLive");
  }
}

void printPitotLive(const char* prefix) {
  float airspeed, aoa, aoss, staticPressure;
  bool valid;

  pitotMutex.lock();
  airspeed       = airspeed_raw;
  aoa            = aoa_raw;
  aoss           = aoss_raw;
  staticPressure = static_pressure_raw;
  valid          = pitot_data_valid;
  pitotMutex.unlock();

  Serial.print(prefix);
  Serial.print(valid ? "|OK|" : "|STALE|");
  Serial.print("airspeed=");
  Serial.print(airspeed, 3);
  Serial.print("|aoa=");
  Serial.print(aoa, 3);
  Serial.print("|aoss=");
  Serial.print(aoss, 3);
  Serial.print("|staticPressure=");
  Serial.println(staticPressure, 3);
}

// void readPitot() {
//   got_pitot_readings = false;
//   String pitot = Serial3.readStringUntil('\n'); 
//   String airspeedStr, aoaStr, aossStr;

//   int time = pitot.indexOf(',');
//   int date = pitot.indexOf(',', time + 1);
//   int as = pitot.indexOf(',', date + 1);
//   int ias = pitot.indexOf(',', as + 1);
//   int aoa_pos = pitot.indexOf(',', ias + 1);
//   int aoss_pos = pitot.indexOf(',', aoa_pos + 1);
//   int pa = pitot.indexOf(',', aoss_pos + 1);
//   int sp = pitot.indexOf(',', pa + 1);
//   int tp = pitot.indexOf(',', sp + 1);
//   int cs = pitot.indexOf(',', tp + 1);

//   if (as != -1 && aoa_pos != -1 && aoss_pos != -1 && date != -1) {
//     String airSpeedStr = pitot.substring(date + 1, as); 
//     String aoaStr = pitot.substring(ias + 1, aoa_pos); 
//     String aossStr = pitot.substring(aoa_pos + 1, aoss_pos); 
//     pitotMutex.lock();
//     airspeed_raw = airSpeedStr.toFloat();
//     aoa_raw = aoaStr.toFloat();
//     aoss_raw = aossStr.toFloat();
//     pitotMutex.unlock();
//   }
//   got_pitot_readings = true;
//   if (read_stream) {
//     Serial.println("Pitot: " + String(airspeed_raw) + " " + String(aoa_raw) + " " + String(aoss_raw));
//   }
// }

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
      float trimSeparator = command.indexOf("|", 2);
      if (trimSeparator == -1) return;
      String trimAoAStr = command.substring(trimSeparator + 1);
      trimAoA = trimAoAStr.toFloat();
      aoaCanCommandServoDeg((trimAoA - 10));
      delay(500); 
      aoaCanCommandServoDeg((trimAoA + 10));
      delay(500); 
      aoaSensorPhysicalAngle = aoaTargetPhysicalDegForAxisState();
      aoaCanCommandServoDeg(aoaSensorPhysicalAngle);
      Serial.print("trimAoA set: ");
      Serial.println(trimAoA);
    } else if (command.startsWith("AoAlim")) {
      int limaoaSeparator = command.indexOf("|", 2);
      if (limaoaSeparator == -1) return;
      String limStr = command.substring(limaoaSeparator + 1);
      limAoA = limStr.toInt();
      Serial.print("limAoA set: ");
      Serial.println(limAoA);
    } else if (command.startsWith("AoAaxis")) {
      int sep = command.indexOf("|", 2);
      if (sep == -1) return;

      String state = command.substring(sep + 1);
      state.trim();
      state.toLowerCase();

      if (state == "1" || state == "on" || state == "true" || state == "enable" || state == "enabled") {
        setAoAAxisEnabled(true);
      } else if (state == "0" || state == "off" || state == "false" || state == "disable" || state == "disabled") {
        setAoAAxisEnabled(false);
      } else {
        Serial.println("Invalid AoAaxis value. Use 1/on/enabled or 0/off/disabled.");
        return;
      }

      Serial.print("AoA axis ");
      Serial.println(isAoAAxisEnabled() ? "enabled" : "disabled (motor free)");
    } //else if (command.startsWith("trimAoSS")) {
      // int trimSeparator = command.indexOf("|", 2);
      // if (trimSeparator == -1) return;
      // String trimAoSSStr = command.substring(trimSeparator + 1);
      // trimAoSS = trimAoSSStr.toInt();
      // aossCanCommandServoRelDeg(trimAoSS);
      // Serial.print("trimAoSS set: ");
      // Serial.println(trimAoSS);
      else if (command.startsWith("trimAoSS")) {
      int trimSeparator = command.indexOf("|", 2);
      if (trimSeparator == -1) return;
      String trimStr = command.substring(trimSeparator + 1);

      // Trim is a sensor offset (pitot AoSS bias), NOT a movement command.
      aossTrimDeg = trimStr.toFloat();

      Serial.print("trimAoSS|OK|");
      Serial.println(aossTrimDeg, 3);
    } else if (command.startsWith("moveAoSS|")) {
      int pipe = command.indexOf('|');
      float tubeCmd = command.substring(pipe + 1).toFloat();

      // Optional: clamp to your tube limits if you want:
      if (tubeCmd < aossMinTubeDeg) tubeCmd = aossMinTubeDeg;
      if (tubeCmd > aossMaxTubeDeg) tubeCmd = aossMaxTubeDeg;

      aossCanCommandServoRelDeg(tubeCmd);
      aossSensorRelAngle = tubeCmd;

      Serial.print("moveAoSS|OK|");
      Serial.println(tubeCmd, 2);
    } else if (command.startsWith("moveAoSSServoAbs|")) {
      int pipe = command.indexOf('|');
      float servoAbsDeg = command.substring(pipe + 1).toFloat();

      // Convert servo->tube
      float tubeCmd = aossTubeDegFromServoDeg(servoAbsDeg);

      // Clamp tube physical limits
      if (tubeCmd < aossMinTubeDeg) tubeCmd = aossMinTubeDeg;
      if (tubeCmd > aossMaxTubeDeg) tubeCmd = aossMaxTubeDeg;

      // Command by tube (safe)
      aossCanCommandServoRelDeg(tubeCmd);
      aossSensorRelAngle = tubeCmd;

      Serial.print("moveAoSSServoAbs|OK|tube=");
      Serial.print(tubeCmd, 2);
      Serial.print("|servoReq=");
      Serial.println(servoAbsDeg, 2);
    } else if (command.startsWith("moveAoSSTubeAbs|")) {
      int pipe = command.indexOf('|');
      float tubeCmd = command.substring(pipe + 1).toFloat();

      // clamp PHYSICAL tube limits
      tubeCmd = clampf(tubeCmd, aossMinTubeDeg, aossMaxTubeDeg);

      // command it (this should ultimately go through tube->servo using ratio)
      aossCanCommandServoRelDeg(tubeCmd);
      aossSensorRelAngle = tubeCmd;

      Serial.print("moveAoSSTubeAbs|OK|tube=");
      Serial.print(tubeCmd, 2);
      Serial.print("|ratio=");
      Serial.println(AoSSratio, 4);
    } else if (command.startsWith("aossTest")) {
      aossCanCommandServoRelDeg(-10.0f);
      delay(500);
      aossCanCommandServoRelDeg(0.0f);
      Serial.println("AoSS CAN TURN test done");
    } else if (command.startsWith("setRunModeTurn")) {
      aossEnableTurnModeAndReset();
      aossPrintRunMode();
      Serial.println("AoSS TURN setup done (RUN_MODE=0, saved, reset).");
    } else if (command.startsWith("setAoSSRatio|")) {
      int pipe = command.indexOf('|');
      float r = command.substring(pipe + 1).toFloat();

      // choose sane bounds for your hardware
      r = clampf(r, 0.01f, 1000.0f);

      AoSSratio = r;

      Serial.print("setAoSSRatio|OK|ratio=");
      Serial.println(AoSSratio, 4);
      aossCanCommandServoRelDeg(aossSensorRelAngle); 
    } else if (command == "getAoSSRatio") {
      Serial.print("getAoSSRatio|OK|ratio=");
      Serial.println(AoSSratio, 4);
    } else if (command.startsWith("setAoSSLimits|")) {
      // format: setAoSSLimits|min|max
      int p1 = command.indexOf('|');
      int p2 = command.indexOf('|', p1 + 1);
      if (p1 < 0 || p2 < 0) {
        Serial.println("setAoSSLimits|ERR|format");
      } else {
        float mn = command.substring(p1 + 1, p2).toFloat();
        float mx = command.substring(p2 + 1).toFloat();

        ensureAossLimitOrder(mn, mx);

        // optional sanity bounds to prevent nonsense:
        // (choose something that cannot damage hardware)
        mn = clampf(mn, -90.0f, 90.0f);
        mx = clampf(mx, -90.0f, 90.0f);
        ensureAossLimitOrder(mn, mx);

        aossMinTubeDeg = mn;
        aossMaxTubeDeg = mx;

        // clamp current target to new limits + re-command
        aossSensorRelAngle = clampf(aossSensorRelAngle, aossMinTubeDeg, aossMaxTubeDeg);
        aossCanCommandServoRelDeg(aossSensorRelAngle);

        Serial.print("setAoSSLimits|OK|min=");
        Serial.print(aossMinTubeDeg, 2);
        Serial.print("|max=");
        Serial.println(aossMaxTubeDeg, 2);
      }
    } else if (command == "getAoSSLimits") {
      Serial.print("getAoSSLimits|OK|min=");
      Serial.print(aossMinTubeDeg, 2);
      Serial.print("|max=");
      Serial.println(aossMaxTubeDeg, 2);
    } else if (command.startsWith("enableAoSS")) {
      aoss_enabled = true;
      Serial.println("AoSSenabled");
    } else if (command.startsWith("disableAoSS")) {
      aoss_enabled = false;
      Serial.println("AoSSdisabled");
    } else if (command.startsWith("readAoA")) {
      uint16_t pos = 0;

      // Read actual servo position over CAN
      if (hitecReadReg16(AOA_CAN_ARB_ID, AOA_CAN_SERVO_ID, REG_POSITION, pos, 50)) {
        float physDeg = servoCountsToPhysicalDeg(pos);                 // physical angle (deg)
        float relDeg  = wrapSignedDeg(physDeg - (float)trimAoA);       // relative to trim (deg)

        Serial.print("readAoA|");
        Serial.print(pos);          // raw counts (useful for debugging)
        Serial.print("|");
        Serial.print(physDeg, 2);   // physical angle in deg
        Serial.print("|");
        Serial.print(relDeg, 2);    // physical relative-to-trim angle (0 at trim)
        Serial.println();
      } else {
        Serial.println("readAoA|ERR");
      }
    } else if (command.startsWith("readAoSS")) {
      // Force one immediate refresh (optional)
      aossCanReadbackUpdate();

      Serial.print("readAoSS|");
      Serial.print(aossServoPosNow);
      Serial.print("|");
      Serial.print(aossTurnNow);
      Serial.print("|");
      Serial.print(aossServoDegNow, 2);
      Serial.print("|");
      Serial.print(aossTubeDegNow, 2);
      Serial.println();

    } else if (command.startsWith("zeroAoSS")) {
      // User has aligned tube to true 0 using external measuring device.
      // We declare current absolute servo position as tube=0 reference.
      aossCanReadbackUpdate();
      aossZeroOffsetDeg = aossServoDegNow;
      aossSensorRelAngle = 0.0f;

      Serial.print("zeroAoSS|OK|");
      Serial.println(aossZeroOffsetDeg, 2);

    } else if (command.startsWith("aossDir|")) {
      int pipe = command.indexOf('|');
      String v = command.substring(pipe + 1);
      int d = v.toInt();
      aossDir = (d < 0) ? -1 : +1;
      Serial.print("aossDir|OK|");
      Serial.println(aossDir);

    } else if (command.startsWith("aossCal|")) {
      // External calibration: you measure tube angle physically (deg) and tell MCU.
      // We then solve: tubeDeg = dir*(servoDeg - zeroOffset)  => zeroOffset = servoDeg - dir*tubeDeg
      int pipe = command.indexOf('|');
      String v = command.substring(pipe + 1);
      float tubeDegMeasured = v.toFloat();

      aossCanReadbackUpdate();
      aossZeroOffsetDeg = aossServoDegNow - (float)aossDir * tubeDegMeasured;

      Serial.print("aossCal|OK|");
      Serial.print(aossZeroOffsetDeg, 2);
      Serial.print("|dir|");
      Serial.println(aossDir);
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
    } else if (command.startsWith("readPitotLive")) {
      // One-shot readout of the latest Pitot data. This command does not move the arms.
      printPitotLive("readPitotLive");
    } else if (command.startsWith("pitotLiveStart")) {
      // Verification mode: keep reading Pitot data, stream it to the UI, and hold AoA/AoSS arms still.
      pitot_live_mode = true;
      pitot_live_stream = true;
      Serial.println("pitotLiveStart|OK");
    } else if (command.startsWith("pitotLiveStop")) {
      // Leave verification mode and allow normal automatic AoA/AoSS tracking again.
      pitot_live_stream = false;
      pitot_live_mode = false;
      Serial.println("pitotLiveStop|OK");
    } else if (command.startsWith("pitotLiveHoldOn")) {
      pitot_live_mode = true;
      Serial.println("pitotLiveHoldOn|OK");
    } else if (command.startsWith("pitotLiveHoldOff")) {
      pitot_live_mode = false;
      Serial.println("pitotLiveHoldOff|OK");
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

void adjustSensorAoAPosition(float aoa, float airspeed) {
  static uint32_t lastAdjustMs = 0;

  // If the AoA axis is disabled, motor is free. Do not send position demand.
  if (!aoa_enabled) {
    parkAoAAxisDisabled();
    return;
  }

  // 1) If there's no airflow, park at trim
  if (airspeed <= 0.01f) {
    aoaSensorRelAngle = 0.0f;
    aoaSensorPhysicalAngle = aoaTargetPhysicalDegForAxisState();
    aoaCanCommandServoDeg(aoaSensorPhysicalAngle);
    return;
  }

  // 2) Airflow present: aoa is relative to current tube orientation.
  // Use a deadband and cooldown so the servo follows real limit excursions,
  // but does not chase turbulence/noise every loop.
  const float engageMarginDeg = 1.0f;
  const float controlDeadbandDeg = 0.3f;
  const uint32_t adjustCooldownMs = 30;
  const float hi = maxAoA - engageMarginDeg;
  const float lo = minAoA + engageMarginDeg;

  uint32_t now = millis();
  if ((now - lastAdjustMs) >= adjustCooldownMs) {
    if (aoa > hi + controlDeadbandDeg && aoaSensorRelAngle < (float)limAoA) {
      aoaSensorRelAngle -= adjustmentStepAoA;
      lastAdjustMs = now;
    } else if (aoa < lo - controlDeadbandDeg && aoaSensorRelAngle > -(float)limAoA) {
      aoaSensorRelAngle += adjustmentStepAoA;
      lastAdjustMs = now;
    }
  }

  // IMPORTANT: do NOT reset on aoa == 0.0f (0 is a valid relative AoA)
  aoaSensorPhysicalAngle = aoaTargetPhysicalDegForAxisState();
  aoaCanCommandServoDeg(aoaSensorPhysicalAngle);
}

void adjustSensorAoSSPosition(float aoss, float airspeed) {
  static uint32_t lastAdjustMs = 0;

  // If no airflow -> park at trim
  if (airspeed <= 0.01f) {
    aossSensorRelAngle = 0.0f;
    aossCanCommandServoRelDeg(aossSensorRelAngle);
    return;
  }

  const float engageMarginDeg = 0.5f;
  const float controlDeadbandDeg = 0.2f;
  const uint32_t adjustCooldownMs = 40;
  const float hi = maxAoSS - engageMarginDeg;
  const float lo = minAoSS + engageMarginDeg;

  uint32_t now = millis();
  if ((now - lastAdjustMs) >= adjustCooldownMs) {
    if (aoss > hi + controlDeadbandDeg) {
      aossSensorRelAngle += adjustmentStepAoSS;
      lastAdjustMs = now;
    } else if (aoss < lo - controlDeadbandDeg) {
      aossSensorRelAngle -= adjustmentStepAoSS;
      lastAdjustMs = now;
    }
  }

  // asymmetric clamp in tube degrees
  aossSensorRelAngle = constrain(aossSensorRelAngle, aossMinTubeDeg, aossMaxTubeDeg);

  aossCanCommandServoRelDeg(aossSensorRelAngle);
}

// void adjustSensorAoSSPosition(float aoss) { // Relative adjuctment, not absolute.
//   if (aoss > maxAoSS - adjustmentStepAoSS && aossSensorPhysicalAngle < limMinAoSS) {
//     aossSensorPhysicalAngle += adjustmentStepAoSS;
//     if (aossSensorPhysicalAngle >= limMinAoSS) {
//       aossSensorPhysicalAngle = limMinAoSS;
//     }
//     //Serial.println(aossSensorPhysicalAngle);
//     aossServo.write(aossSensorPhysicalAngle);
//   } if (aoss < minAoSS + adjustmentStepAoSS && aossSensorPhysicalAngle > limMaxAoSS) {
//       aossSensorPhysicalAngle -= adjustmentStepAoSS;
//       if (aossSensorPhysicalAngle <= limMaxAoSS) {
//         aossSensorPhysicalAngle = limMaxAoSS;
//       }
//       //Serial.println(aossSensorPhysicalAngle);
//       aossServo.write(aossSensorPhysicalAngle);
//   } if (aoss == 0.00) {
//       aossSensorPhysicalAngle = trimAoSS;
//       aossServo.write(aossSensorPhysicalAngle);
//   }
// }

// float calculateAbsoluteAoA(float /*aoa*/) {
//   // Absolute AoA is defined as signed angle relative to trim (trim = 0).
//   // Use the servo readback (aoaMeasuredRelDeg), not the last commanded.
//   return aoaMeasuredRelDeg;
// }

// float calculateAbsoluteAoSS(float aoss) {
//   // Calculate the absolute AoA based on the sensor's physical angle
//   return (aoss - (trimAoSS - aossSensorPhysicalAngle) / AoSSratio); // Adjust based on the midpoint being 90 degrees
// }

float calculateAbsoluteAoSS(float /*aoss*/) {
  return aossMeasuredRelDeg; // 0 at trim
}

void aossController() {
  float aoss, airspeed;
  pitotMutex.lock();
  aoss = aoss_raw;
  airspeed = airspeed_raw;
  pitotMutex.unlock();

  // Pitot AoSS is what the user cares about. Apply user trim so that 0 deg becomes the trimmed reference.
  aossPitotDegNow = aoss;
  aossPitotDegTrimmed = aoss - aossTrimDeg;

  if (aoss_enabled && !pitot_live_mode) {
    // Control logic should use trimmed AoSS.
    adjustSensorAoSSPosition(aossPitotDegTrimmed, airspeed);
  }

  // Report servo-derived AoSS (trim-relative, based on turn+pos readback).
  absAoSS = aossPitotDegTrimmed + aossMeasuredRelDeg;
}


// void aossController() {
//   if (aoss_enabled == true) {
//     float aoss = aoss_raw;               // Read current AoA
//     adjustSensorAoSSPosition(aoss);           // Adjust sensor position if near limits
//     absAoSS = calculateAbsoluteAoSS(aoss);  // Calculate the absolute AoA
//   }
//   else {
//     float aoss = aoss_raw;               // Read current AoA
//     //adjustSensorAoSSPosition(aoss);           // Adjust sensor position if near limits
//     absAoSS = calculateAbsoluteAoSS(aoss);  // Calculate the absolute AoA
//   }
// }

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
