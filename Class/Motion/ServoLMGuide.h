#pragma once

/*
 * ServoLMGuide.h — LM Guide ASDA-B3 por RS-485 Modbus RTU.
 * Requiere Parameters.h (alias MODBUS_ASDA_* / ASDA_*).
 * MotionRuntime debe estar definido antes de incluir este header.
 * Llamar lmguideBind() antes de usar.
 */

#include "Parameters.h"

#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#ifndef MOTION_CALLBACK_TYPES
#define MOTION_CALLBACK_TYPES
typedef void (*MotionPollFn)();
typedef void (*MotionYieldFn)(uint32_t ms);
typedef bool (*MotionSafetyFn)();
#endif

#ifndef MODBUS_ASDA_SLAVE_ID
static const uint8_t  MODBUS_ASDA_SLAVE_ID = 0x01;
static const uint32_t MODBUS_ASDA_BAUD     = 9600;
#endif

#ifndef ASDA_HOME_TORQUE_PCT_DEFAULT
static const uint16_t ASDA_HOME_TORQUE_PCT_DEFAULT = 10;
static const uint16_t ASDA_HOME_TIME_MS_DEFAULT    = 150;
static const float    ASDA_HOME_SPEED_RPM_DEFAULT  = 10.0f;
static const uint16_t ASDA_HOME_METHOD_FORWARD     = 0x0029;
static const uint16_t ASDA_HOME_METHOD_REVERSE     = 0x002A;
static const uint32_t ASDA_SERVO_ON_SETTLE_MS      = 2000;
#endif

#ifndef ASDA_JOG_DELTA_PUU
static const int32_t ASDA_JOG_DELTA_PUU = 2000;
#endif

// Pines UART: Parameters.h (PARAM_PIN_MODBUS_RX / TX).

static const uint16_t ASDA_REG_P0_046 = 0x005C;
static const uint16_t ASDA_REG_P1_087 = 0x01AE;
static const uint16_t ASDA_REG_P1_088 = 0x01B0;
static const uint16_t ASDA_REG_P2_030 = 0x023C;
static const uint16_t ASDA_REG_P5_004 = 0x0508;
static const uint16_t ASDA_REG_P5_005 = 0x050A;
static const uint16_t ASDA_REG_P5_006 = 0x050C;
static const uint16_t ASDA_REG_P5_007 = 0x050E;
static const uint16_t ASDA_REG_P5_016 = 0x0520;
static const uint16_t ASDA_REG_P5_020 = 0x0528;  // Accel time #0 (cut)
static const uint16_t ASDA_REG_P5_021 = 0x052A;  // Decel time #1 (cut)
static const uint16_t ASDA_REG_P5_022 = 0x052C;  // Accel time #2 (travel)
static const uint16_t ASDA_REG_P5_023 = 0x052E;  // Decel time #3 (travel)
static const uint16_t ASDA_REG_P5_060 = 0x0578;  // Speed #0 (cut)
static const uint16_t ASDA_REG_P5_061 = 0x057A;  // Speed #1 (travel)
// PATH bitfield: AUTO|DLY|SPD|DEC|ACC|OPT|TYPE — TYPE=SINGLE(2)
// PR1 cut:    ACC=#0 P5.020, DEC=#1 P5.021, SPD=#0 P5.060
static const uint32_t ASDA_PR1_PATH_DEF = 0x00001002u;
// PR2 travel: ACC=#2 P5.022, DEC=#3 P5.023, SPD=#1 P5.061
static const uint32_t ASDA_PR2_PATH_DEF = 0x00013202u;
static const uint16_t ASDA_REG_P6_000 = 0x0600;
static const uint16_t ASDA_REG_P6_001 = 0x0602;
static const uint16_t ASDA_REG_P6_002 = 0x0604;  // PR1 property
static const uint16_t ASDA_REG_P6_003 = 0x0606;  // PR1 position
static const uint16_t ASDA_REG_P6_004 = 0x0608;  // PR2 property
static const uint16_t ASDA_REG_P6_005 = 0x060A;  // PR2 position

static const uint16_t ASDA_DO_BIT_ZSPD = 0x0004;
static const uint16_t ASDA_DO_BIT_TPOS = 0x0010;  // P0.046 bit4 — target position reached
static const uint16_t ASDA_DO_BIT_HOME = 0x0100;

// Fábrica 770 PUU / 45 mm; campo: 550 mm pedido → 548 mm reales (app.py igual).
static const float ASDA_FACTORY_STEPS_PER_MM = (770.0f / 45.0f) * (550.0f / 548.0f);

static HardwareSerial ModbusSerial(2);

static MotionTune* gLmTune = nullptr;
static MotionRuntime* gLmRt = nullptr;
static volatile bool* gLmAbort = nullptr;
static MotionYieldFn gLmYield = nullptr;
static MotionPollFn gLmPoll = nullptr;
static MotionSafetyFn gLmSafety = nullptr;
static bool gLmProfilesApplied = false;
static uint16_t gLmModbusWrites = 0;  // contador por movimiento (medición)

static void lmPoll() {
  if (gLmPoll) gLmPoll();
}

static bool lmSafetyActive() {
  return gLmSafety ? gLmSafety() : false;
}

static bool lmAbortRequested() {
  return gLmAbort && *gLmAbort;
}

static void lmguideBind(MotionTune* tune, MotionRuntime* rt, volatile bool* abortFlag,
                        MotionYieldFn yieldFn, MotionPollFn pollFn, MotionSafetyFn safetyFn) {
  gLmTune = tune;
  gLmRt = rt;
  gLmAbort = abortFlag;
  gLmYield = yieldFn;
  gLmPoll = pollFn;
  gLmSafety = safetyFn;
}

static uint16_t modbusCrc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

static bool modbusParseRxOk(const uint8_t* tx, size_t txLen, const uint8_t* rx, size_t rxLen) {
  if (rxLen < 4) return false;
  uint16_t crc = modbusCrc16(rx, rxLen - 2);
  if (rx[rxLen - 2] != (uint8_t)(crc & 0xFF) || rx[rxLen - 1] != (uint8_t)(crc >> 8)) return false;
  if (rx[0] != MODBUS_ASDA_SLAVE_ID) return false;
  if (tx[1] == 0x06 && rxLen >= 8) return memcmp(tx, rx, 6) == 0;
  if (tx[1] == 0x10 && rxLen >= 8) return rx[1] == 0x10;
  if (tx[1] == 0x03 && rxLen >= 5) return rx[1] == 0x03;
  return false;
}

static bool modbusTxRx(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t* rxLen, size_t rxCap) {
  while (ModbusSerial.available()) ModbusSerial.read();
  // Poll breve: feed async del solape debe tickear entre TX Modbus.
  lmPoll();
  delay(5);
  lmPoll();
  ModbusSerial.write(tx, txLen);
  ModbusSerial.flush();

  uint32_t t0 = millis();
  size_t n = 0;
  while (millis() - t0 < 350) {
    lmPoll();
    while (ModbusSerial.available() && n < rxCap) {
      rx[n++] = (uint8_t)ModbusSerial.read();
    }
    if (n >= 5) {
      if (tx[1] == 0x03 && n >= (size_t)(5 + rx[2])) break;
      if ((tx[1] == 0x06 || tx[1] == 0x10) && n >= 8) break;
    }
    delay(1);
  }
  *rxLen = n;
  return modbusParseRxOk(tx, txLen, rx, n);
}

static bool modbusWriteReg16(uint16_t addr, uint16_t value) {
  uint8_t tx[8];
  tx[0] = MODBUS_ASDA_SLAVE_ID;
  tx[1] = 0x06;
  tx[2] = (uint8_t)(addr >> 8);
  tx[3] = (uint8_t)(addr & 0xFF);
  tx[4] = (uint8_t)(value >> 8);
  tx[5] = (uint8_t)(value & 0xFF);
  uint16_t crc = modbusCrc16(tx, 6);
  tx[6] = (uint8_t)(crc & 0xFF);
  tx[7] = (uint8_t)(crc >> 8);
  uint8_t rx[16];
  size_t rxLen = 0;
  bool ok = modbusTxRx(tx, 8, rx, &rxLen, sizeof(rx));
  if (ok) gLmModbusWrites++;
  return ok;
}

static bool modbusWriteReg32(uint16_t addr, int32_t value) {
  uint8_t tx[13];
  tx[0] = MODBUS_ASDA_SLAVE_ID;
  tx[1] = 0x10;
  tx[2] = (uint8_t)(addr >> 8);
  tx[3] = (uint8_t)(addr & 0xFF);
  tx[4] = 0x00;
  tx[5] = 0x02;
  tx[6] = 0x04;
  tx[7] = (uint8_t)((value >> 8) & 0xFF);
  tx[8] = (uint8_t)(value & 0xFF);
  tx[9] = (uint8_t)((value >> 24) & 0xFF);
  tx[10] = (uint8_t)((value >> 16) & 0xFF);
  uint16_t crc = modbusCrc16(tx, 11);
  tx[11] = (uint8_t)(crc & 0xFF);
  tx[12] = (uint8_t)(crc >> 8);
  uint8_t rx[16];
  size_t rxLen = 0;
  bool ok = modbusTxRx(tx, 13, rx, &rxLen, sizeof(rx));
  if (ok) gLmModbusWrites++;
  return ok;
}

static void lmguideInvalidateProfiles() {
  gLmProfilesApplied = false;
}

static bool lmguideProfilesApplied() {
  return gLmProfilesApplied;
}

static uint16_t lmguideLastModbusWrites() {
  return gLmModbusWrites;
}

static bool modbusReadReg32(uint16_t addr, int32_t* out) {
  uint8_t tx[8];
  tx[0] = MODBUS_ASDA_SLAVE_ID;
  tx[1] = 0x03;
  tx[2] = (uint8_t)(addr >> 8);
  tx[3] = (uint8_t)(addr & 0xFF);
  tx[4] = 0x00;
  tx[5] = 0x02;
  uint16_t crc = modbusCrc16(tx, 6);
  tx[6] = (uint8_t)(crc & 0xFF);
  tx[7] = (uint8_t)(crc >> 8);
  uint8_t rx[16];
  size_t rxLen = 0;
  if (!modbusTxRx(tx, 8, rx, &rxLen, sizeof(rx)) || rxLen < 9) return false;
  uint16_t lo = ((uint16_t)rx[3] << 8) | rx[4];
  uint16_t hi = ((uint16_t)rx[5] << 8) | rx[6];
  *out = (int32_t)((uint32_t)lo | ((uint32_t)hi << 16));
  return true;
}

static bool modbusReadReg16(uint16_t addr, uint16_t* out) {
  uint8_t tx[8];
  tx[0] = MODBUS_ASDA_SLAVE_ID;
  tx[1] = 0x03;
  tx[2] = (uint8_t)(addr >> 8);
  tx[3] = (uint8_t)(addr & 0xFF);
  tx[4] = 0x00;
  tx[5] = 0x01;
  uint16_t crc = modbusCrc16(tx, 6);
  tx[6] = (uint8_t)(crc & 0xFF);
  tx[7] = (uint8_t)(crc >> 8);
  uint8_t rx[16];
  size_t rxLen = 0;
  if (!modbusTxRx(tx, 8, rx, &rxLen, sizeof(rx)) || rxLen < 7) return false;
  *out = ((uint16_t)rx[3] << 8) | rx[4];
  return true;
}

static void modbusBeginHw() {
  ModbusSerial.begin(MODBUS_ASDA_BAUD, SERIAL_8N2, PARAM_PIN_MODBUS_RX, PARAM_PIN_MODBUS_TX);
}

static float lmClampSpeedRpm(float rpm) {
  if (rpm < PARAM_LM_PR_SPEED_RPM_MIN) return PARAM_LM_PR_SPEED_RPM_MIN;
  if (rpm > PARAM_LM_PR_SPEED_RPM_MAX) return PARAM_LM_PR_SPEED_RPM_MAX;
  return rpm;
}

static uint16_t lmClampRampMs(long ms) {
  if (ms < (long)PARAM_LM_RAMP_MS_MIN) return PARAM_LM_RAMP_MS_MIN;
  if (ms > (long)PARAM_LM_RAMP_MS_MAX) return PARAM_LM_RAMP_MS_MAX;
  return (uint16_t)ms;
}

static int32_t lmTolPuuFromMm(float tolMm) {
  return (int32_t)lroundf(tolMm * ASDA_FACTORY_STEPS_PER_MM);
}

static bool lmPositionWithinTol(int32_t posPuu, int32_t targetPuu, float tolMm) {
  int32_t d = posPuu - targetPuu;
  if (d < 0) d = -d;
  return d <= lmTolPuuFromMm(tolMm);
}

/* Escribe perfiles cut (PR1) y travel (PR2) una vez. No escribe posición ni trigger. */
static bool lmguideApplyProfiles() {
  if (!gLmTune) return false;
  float cutRpm = lmClampSpeedRpm(gLmTune->lmguide.prSpeedRpm);
  float travelRpm = lmClampSpeedRpm(gLmTune->lmguide.travelPrSpeedRpm);
  uint16_t cutAccel = lmClampRampMs((long)gLmTune->lmguide.accelMs);
  uint16_t cutDecel = lmClampRampMs((long)gLmTune->lmguide.decelMs);
  uint16_t travelAccel = lmClampRampMs((long)gLmTune->lmguide.travelAccelMs);
  uint16_t travelDecel = lmClampRampMs((long)gLmTune->lmguide.travelDecelMs);
  int32_t cutSpeed01 = (int32_t)(cutRpm * 10.0f);
  int32_t travelSpeed01 = (int32_t)(travelRpm * 10.0f);

  gLmProfilesApplied = false;
  if (!modbusWriteReg16(ASDA_REG_P5_020, cutAccel)) return false;
  if (!modbusWriteReg16(ASDA_REG_P5_021, cutDecel)) return false;
  if (!modbusWriteReg32(ASDA_REG_P5_060, cutSpeed01)) return false;
  if (!modbusWriteReg32(ASDA_REG_P6_002, (int32_t)ASDA_PR1_PATH_DEF)) return false;

  if (!modbusWriteReg16(ASDA_REG_P5_022, travelAccel)) return false;
  if (!modbusWriteReg16(ASDA_REG_P5_023, travelDecel)) return false;
  if (!modbusWriteReg32(ASDA_REG_P5_061, travelSpeed01)) return false;
  if (!modbusWriteReg32(ASDA_REG_P6_004, (int32_t)ASDA_PR2_PATH_DEF)) return false;

  gLmProfilesApplied = true;
  return true;
}

/* Emite movimiento. Con perfiles aplicados: solo posición + trigger (2 escrituras). */
static bool lmguideIssueMove(int32_t targetPuu, LmMoveProfile profile) {
  if (!gLmTune) return false;
  const bool cut = (profile == LM_PROFILE_CUT);

  if (gLmProfilesApplied) {
    uint16_t posReg = cut ? ASDA_REG_P6_003 : ASDA_REG_P6_005;
    uint16_t trigger = cut ? 1u : 2u;
    if (!modbusWriteReg32(posReg, targetPuu)) {
      gLmProfilesApplied = false;
      return false;
    }
    if (!modbusWriteReg16(ASDA_REG_P5_007, trigger)) {
      gLmProfilesApplied = false;
      return false;
    }
    return true;
  }

  // Fallback legacy: reescribe rampas del path pedido (App antigua / config inválida).
  float rpm = lmClampSpeedRpm(cut ? gLmTune->lmguide.prSpeedRpm
                                  : gLmTune->lmguide.travelPrSpeedRpm);
  uint16_t accelMs = lmClampRampMs(
      (long)(cut ? gLmTune->lmguide.accelMs : gLmTune->lmguide.travelAccelMs));
  uint16_t decelMs = lmClampRampMs(
      (long)(cut ? gLmTune->lmguide.decelMs : gLmTune->lmguide.travelDecelMs));
  int32_t speed01 = (int32_t)(rpm * 10.0f);
  if (cut) {
    if (!modbusWriteReg16(ASDA_REG_P5_020, accelMs)) return false;
    if (!modbusWriteReg16(ASDA_REG_P5_021, decelMs)) return false;
    if (!modbusWriteReg32(ASDA_REG_P5_060, speed01)) return false;
    if (!modbusWriteReg32(ASDA_REG_P6_002, (int32_t)ASDA_PR1_PATH_DEF)) return false;
    if (!modbusWriteReg32(ASDA_REG_P6_003, targetPuu)) return false;
    return modbusWriteReg16(ASDA_REG_P5_007, 1);
  }
  if (!modbusWriteReg16(ASDA_REG_P5_022, accelMs)) return false;
  if (!modbusWriteReg16(ASDA_REG_P5_023, decelMs)) return false;
  if (!modbusWriteReg32(ASDA_REG_P5_061, speed01)) return false;
  if (!modbusWriteReg32(ASDA_REG_P6_004, (int32_t)ASDA_PR2_PATH_DEF)) return false;
  if (!modbusWriteReg32(ASDA_REG_P6_005, targetPuu)) return false;
  return modbusWriteReg16(ASDA_REG_P5_007, 2);
}

// Espera DO.TPOS (P0.046). Más rápido que ZSPD + P5.016 en cada vuelta:
// un solo registro 16-bit hasta llegar; P5.016 solo una vez al confirmar.
// Moves cortos (2º corte dual ~0.25–1 mm) a veces no bajan TPOS: aceptar
// con TPOS+tol solo si hubo flanco OFF o recorrido ≥50 % del delta pedido.
// No aceptar TPOS+tol pegado en la partida (falso “llegué” sin moverse).
static bool lmguideWaitForTarget(int32_t targetPuu, float tolMm, uint32_t timeoutMs,
                                 int32_t startPuu, bool haveStart) {
  uint32_t t0 = millis();
  bool sawLeave = false;
  while (millis() - t0 < timeoutMs) {
    lmPoll();
    if (lmAbortRequested()) return false;
    if (lmSafetyActive()) return false;
    uint16_t dwo = 0;
    if (!modbusReadReg16(ASDA_REG_P0_046, &dwo)) return false;
    if ((dwo & ASDA_DO_BIT_TPOS) == 0) {
      sawLeave = true;
      // 1 ms + lmPoll: el feed async del solape debe tickear (láser/V2);
      // delay(5) seguido dejaba el alimentador en V1 y se pasaba el flanco.
      for (uint8_t s = 0; s < 5; s++) {
        lmPoll();
        delay(1);
      }
      continue;
    }
    int32_t pos = 0;
    if (!modbusReadReg32(ASDA_REG_P5_016, &pos)) return false;
    if (!lmPositionWithinTol(pos, targetPuu, tolMm)) {
      for (uint8_t s = 0; s < 5; s++) {
        lmPoll();
        delay(1);
      }
      continue;
    }
    if (!haveStart || startPuu == targetPuu || sawLeave) {
      return true;
    }
    int32_t cmd = targetPuu - startPuu;
    if (cmd < 0) cmd = -cmd;
    int32_t traveled = pos - startPuu;
    if (traveled < 0) traveled = -traveled;
    int32_t minTravel = cmd / 2;
    if (minTravel < 1) minTravel = 1;
    if (traveled >= minTravel) return true;
    for (uint8_t s = 0; s < 5; s++) {
      lmPoll();
      delay(1);
    }
  }
  return false;
}

static bool lmguideMove(int32_t targetPuu, LmMoveProfile profile) {
  if (!gLmRt || !gLmAbort) return false;
  *gLmAbort = false;
  gLmModbusWrites = 0;
  if (targetPuu != 0) gLmRt->lmguideHomed = false;
  // No saltar el comando por la tol de llegada (0.10 mm): un 2º corte dual con
  // offset 0.25–0.7 mm debe moverse. Solo omitir si ya está en el PUU pedido.
  for (uint8_t attempt = 1; attempt <= PARAM_LM_MOVE_MAX_ATTEMPTS; attempt++) {
    int32_t startPuu = 0;
    bool haveStart = modbusReadReg32(ASDA_REG_P5_016, &startPuu);
    if (haveStart && startPuu == targetPuu) {
      gLmRt->lmguideServoOn = true;
      if (targetPuu == 0) gLmRt->lmguideHomed = true;
      return true;
    }
    if (!lmguideIssueMove(targetPuu, profile)) return false;
    if (lmguideWaitForTarget(targetPuu, PARAM_LM_MOVE_TOLERANCE_MM,
                             PARAM_LM_MOVE_DONE_TIMEOUT_MS, startPuu, haveStart)) {
      gLmRt->lmguideServoOn = true;
      if (targetPuu == 0) gLmRt->lmguideHomed = true;
      return true;
    }
    if (lmAbortRequested()) return false;
    lmguideInvalidateProfiles();
    (void)modbusWriteReg16(ASDA_REG_P5_007, 1000);
    delay(80);
    uint32_t tw = millis();
    while (millis() - tw < 400u) {
      lmPoll();
      uint16_t dwo = 0;
      if (modbusReadReg16(ASDA_REG_P0_046, &dwo) && (dwo & ASDA_DO_BIT_ZSPD)) break;
      delay(15);
    }
  }
  return false;
}

/* Compat: movimiento con perfil de viaje (home / descarga / jog). */
static bool lmguideMovePr1(int32_t targetPuu) {
  return lmguideMove(targetPuu, LM_PROFILE_TRAVEL);
}

static bool lmguideServoOn() {
  if (!gLmRt) return false;
  if (!modbusWriteReg16(ASDA_REG_P2_030, 1)) return false;
  if (gLmYield) gLmYield(ASDA_SERVO_ON_SETTLE_MS);
  else delay(ASDA_SERVO_ON_SETTLE_MS);
  gLmRt->lmguideServoOn = true;
  return true;
}

static bool lmguideServoOff() {
  if (!gLmRt) return false;
  if (!modbusWriteReg16(ASDA_REG_P2_030, 0xFFFF)) return false;
  if (!modbusWriteReg16(ASDA_REG_P2_030, 0)) return false;
  gLmRt->lmguideServoOn = false;
  gLmRt->lmguideHomed = false;
  lmguideInvalidateProfiles();
  return true;
}

static bool lmguideStop() {
  if (gLmAbort) *gLmAbort = true;
  return modbusWriteReg16(ASDA_REG_P5_007, 1000);
}

static bool lmguideJog(int dir) {
  if (!gLmRt || !gLmRt->lmguideHomed) return false;
  int32_t pos = 0;
  if (!modbusReadReg32(ASDA_REG_P5_016, &pos)) return false;
  int32_t target = pos + dir * ASDA_JOG_DELTA_PUU;
  bool ok = lmguideMove(target, LM_PROFILE_TRAVEL);
  if (ok) gLmRt->lmguideHomed = false;
  return ok;
}

static bool lmguideJogDelta(int32_t deltaPuu) {
  if (!gLmRt || !gLmRt->lmguideHomed) return false;
  int32_t pos = 0;
  if (!modbusReadReg32(ASDA_REG_P5_016, &pos)) return false;
  bool ok = lmguideMove(pos + deltaPuu, LM_PROFILE_TRAVEL);
  if (ok) gLmRt->lmguideHomed = false;
  return ok;
}

/* Ya en home (HOME+ZSPD, PUU≈0): típico tras reinicio del ESP con LM vivo. */
static bool lmguideAlreadyAtHome() {
  if (!gLmRt) return false;
  int32_t pos = 0;
  if (!modbusReadReg32(ASDA_REG_P5_016, &pos)) return false;
  if (!lmPositionWithinTol(pos, 0, PARAM_LM_HOME_TOLERANCE_MM)) return false;
  uint16_t dwo = 0;
  if (!modbusReadReg16(ASDA_REG_P0_046, &dwo)) return false;
  return (dwo & ASDA_DO_BIT_HOME) != 0 && (dwo & ASDA_DO_BIT_ZSPD) != 0;
}

static bool lmguideHome() {
  if (!gLmRt || !gLmAbort) return false;
  *gLmAbort = false;
  /* Adoptar home existente; no repetir homing de torque si ya está en cero. */
  if (lmguideAlreadyAtHome()) {
    if (!lmguideServoOn()) return false;
    gLmRt->lmguideHomed = true;
    return true;
  }
  gLmRt->lmguideHomed = false;
  if (!lmguideServoOn()) return false;
  int32_t puuStart = 0;
  if (!modbusReadReg32(ASDA_REG_P5_016, &puuStart)) return false;
  int32_t maxTravelPuu = lmTolPuuFromMm(PARAM_LM_HOME_MAX_TRAVEL_MM);

  if (!modbusWriteReg16(ASDA_REG_P1_087, ASDA_HOME_TORQUE_PCT_DEFAULT)) return false;
  if (!modbusWriteReg16(ASDA_REG_P1_088, ASDA_HOME_TIME_MS_DEFAULT)) return false;
  if (!modbusWriteReg16(ASDA_REG_P5_004, ASDA_HOME_METHOD_FORWARD)) return false;
  int32_t speed01 = (int32_t)(ASDA_HOME_SPEED_RPM_DEFAULT * 10.0f);
  if (!modbusWriteReg32(ASDA_REG_P5_005, speed01)) return false;
  if (!modbusWriteReg32(ASDA_REG_P5_006, speed01)) return false;
  if (!modbusWriteReg32(ASDA_REG_P6_000, 0)) return false;
  if (!modbusWriteReg32(ASDA_REG_P6_001, 0)) return false;
  if (!modbusWriteReg16(ASDA_REG_P5_007, 0)) return false;

  uint32_t t0 = millis();
  while (millis() - t0 < PARAM_LM_HOME_DONE_TIMEOUT_MS) {
    lmPoll();
    if (lmAbortRequested()) {
      (void)modbusWriteReg16(ASDA_REG_P5_007, 1000);
      return false;
    }
    if (lmSafetyActive()) {
      (void)modbusWriteReg16(ASDA_REG_P5_007, 1000);
      return false;
    }
    int32_t posNow = 0;
    if (modbusReadReg32(ASDA_REG_P5_016, &posNow)) {
      int32_t delta = posNow - puuStart;
      if (delta < 0) delta = -delta;
      if (delta > maxTravelPuu) {
        (void)modbusWriteReg16(ASDA_REG_P5_007, 1000);
        return false;
      }
    }
    uint16_t dwo = 0;
    if (modbusReadReg16(ASDA_REG_P0_046, &dwo)) {
      if ((dwo & ASDA_DO_BIT_HOME) && (dwo & ASDA_DO_BIT_ZSPD)) {
        int32_t posEnd = 0;
        if (!modbusReadReg32(ASDA_REG_P5_016, &posEnd)) return false;
        if (!lmPositionWithinTol(posEnd, 0, PARAM_LM_HOME_TOLERANCE_MM)) return false;
        gLmRt->lmguideHomed = true;
        return true;
      }
    }
    delay(50);
  }
  return false;
}
