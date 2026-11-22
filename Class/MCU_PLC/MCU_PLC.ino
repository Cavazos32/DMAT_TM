/*
 * MCU_PLC.ino — MCU PLC (ESP32)
 * Válvulas por nivel: orden → ejecuta (settle/ms) → confirma con estado.
 * Blower por nivel; sensores con debounce.
 * Referencia: PlcIo_Uso.h. Parámetros: Parameters.h.
 *
 * Sin bus hacia el PLC físico. Órdenes por Serial (JSON, una línea).
 */

#include <Arduino.h>
#include <string.h>

#include "Parameters.h"
#include "PlcIo_Uso.h"

enum PlcValveId : uint8_t {
  PLC_VALVE_CUTTER_R = 0,
  PLC_VALVE_CUTTER_L,
  PLC_VALVE_GRIPPER,
  PLC_VALVE_HOLDER,
  PLC_VALVE_ENCODER,
  PLC_VALVE_COUNT
};

static const uint8_t kValvePin[PLC_VALVE_COUNT] = {
    PLC_PIN_OUT_CUTTER_R,
    PLC_PIN_OUT_CUTTER_L,
    PLC_PIN_OUT_GRIPPER,
    PLC_PIN_OUT_HOLDER,
    PLC_PIN_OUT_ENCODER,
};

static bool gTimedCutActive = false;
static uint8_t gTimedCutMask = 0;
static uint32_t gTimedCutT0 = 0;
static uint32_t gTimedCutMs = 0;

static bool gSettleActive = false;
static uint32_t gSettleT0 = 0;
static uint32_t gSettleMs = 0;

static bool gResetActive = false;
static uint32_t gResetT0 = 0;

static bool gBlowerOn = false;
static uint32_t gBlowerT0 = 0;
static uint32_t gBlowerMs = 0;

static uint8_t gLastRaw = 0;
static uint8_t gStable = 0;
static bool gStableValid = false;
static uint32_t gRawMs = 0;
static uint8_t gReportedFaults = 0xFF;
static bool gEncoderArmed = false;
static uint32_t gBootMs = 0;

static char gSerialLine[192];
static size_t gSerialLineLen = 0;

static void replyOk(const char* op, bool ok);
static void replyValveDone(bool ok);

// ── JSON plano (Serial) ─────────────────────────────────────────────────────

static bool jsonFindKey(const char* json, const char* key, const char** valOut) {
  char needle[32];
  snprintf(needle, sizeof(needle), "\"%s\"", key);
  const char* p = strstr(json, needle);
  if (!p) return false;
  p = strchr(p + strlen(needle), ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t') p++;
  *valOut = p;
  return true;
}

static long jsonReadLong(const char* json, const char* key, long defVal) {
  const char* p = nullptr;
  if (!jsonFindKey(json, key, &p)) return defVal;
  return strtol(p, nullptr, 10);
}

static bool tokenIs(const char* p, const char* lit) {
  size_t n = strlen(lit);
  if (strncmp(p, lit, n) != 0) return false;
  char c = p[n];
  return c == '"' || c == '\0' || c == ' ' || c == ',';
}

static void printBool(bool v) {
  Serial.print(v ? F("true") : F("false"));
}

static void printHexByte(uint8_t v) {
  Serial.print(F("\"0x"));
  if (v < 16) Serial.print('0');
  Serial.print(v, HEX);
  Serial.print('"');
}

// ── Salidas ─────────────────────────────────────────────────────────────────

static void valveOutputsIdle() {
  const uint8_t pins[] = {
      PLC_PIN_OUT_CUTTER_R, PLC_PIN_OUT_CUTTER_L, PLC_PIN_OUT_GRIPPER,
      PLC_PIN_OUT_HOLDER,   PLC_PIN_OUT_ENCODER,  PLC_PIN_OUT_RESET,
  };
  for (uint8_t i = 0; i < sizeof(pins); i++) {
    digitalWrite(pins[i], LOW);
  }
}

static void initIo() {
  const uint8_t outs[] = {
      PLC_PIN_OUT_CUTTER_R, PLC_PIN_OUT_CUTTER_L, PLC_PIN_OUT_GRIPPER,
      PLC_PIN_OUT_HOLDER,   PLC_PIN_OUT_ENCODER,  PLC_PIN_OUT_RESET,
      PLC_PIN_OUT_BLOWER,
  };
  for (uint8_t i = 0; i < sizeof(outs); i++) {
    pinMode(outs[i], OUTPUT);
  }
  valveOutputsIdle();
  digitalWrite(PLC_PIN_OUT_BLOWER, LOW);

  pinMode(PLC_PIN_IN_CUTTER, INPUT_PULLUP);
  pinMode(PLC_PIN_IN_GRIPPER, INPUT_PULLUP);
  pinMode(PLC_PIN_IN_HOLDER, INPUT_PULLUP);
  pinMode(PLC_PIN_IN_ENCODER, INPUT);
}

static bool setValves(const uint8_t* ids, uint8_t n, bool wantOn) {
  for (uint8_t i = 0; i < n; i++) {
    if (ids[i] >= PLC_VALVE_COUNT) return false;
  }
  for (uint8_t i = 0; i < n; i++) {
    digitalWrite(kValvePin[ids[i]], wantOn ? HIGH : LOW);
  }
  return true;
}

static bool applyValveName(const char* name, bool wantOn) {
  if (tokenIs(name, "holder_encoder")) {
    const uint8_t ids[] = {PLC_VALVE_HOLDER, PLC_VALVE_ENCODER};
    return setValves(ids, 2, wantOn);
  }
  if (tokenIs(name, "cutter_r")) {
    const uint8_t id = PLC_VALVE_CUTTER_R;
    return setValves(&id, 1, wantOn);
  }
  if (tokenIs(name, "cutter_l")) {
    const uint8_t id = PLC_VALVE_CUTTER_L;
    return setValves(&id, 1, wantOn);
  }
  if (tokenIs(name, "cutters")) {
    const uint8_t ids[] = {PLC_VALVE_CUTTER_R, PLC_VALVE_CUTTER_L};
    return setValves(ids, 2, wantOn);
  }
  if (tokenIs(name, "gripper")) {
    const uint8_t id = PLC_VALVE_GRIPPER;
    return setValves(&id, 1, wantOn);
  }
  if (tokenIs(name, "holder")) {
    const uint8_t id = PLC_VALVE_HOLDER;
    return setValves(&id, 1, wantOn);
  }
  if (tokenIs(name, "encoder")) {
    const uint8_t id = PLC_VALVE_ENCODER;
    return setValves(&id, 1, wantOn);
  }
  return false;
}

static bool startTimedCut(const char* name, uint32_t ms) {
  if (gTimedCutActive || gSettleActive || ms == 0 || ms > PLC_CUTTER_MAX_MS) return false;
  if (tokenIs(name, "cutters")) {
    gTimedCutMask = (1 << PLC_VALVE_CUTTER_R) | (1 << PLC_VALVE_CUTTER_L);
  } else if (tokenIs(name, "cutter_r")) {
    gTimedCutMask = (1 << PLC_VALVE_CUTTER_R);
  } else if (tokenIs(name, "cutter_l")) {
    gTimedCutMask = (1 << PLC_VALVE_CUTTER_L);
  } else {
    return false;
  }
  if (!applyValveName(name, true)) return false;
  gTimedCutActive = true;
  gTimedCutT0 = millis();
  gTimedCutMs = ms;
  return true;
}

static bool startSettle(uint32_t ms) {
  if (gTimedCutActive || gSettleActive) return false;
  if (ms == 0) {
    replyValveDone(true);
    return true;
  }
  if (ms > PLC_SETTLE_MAX_MS) return false;
  gSettleActive = true;
  gSettleT0 = millis();
  gSettleMs = ms;
  return true;
}

static void serviceSettle(uint32_t now) {
  if (!gSettleActive) return;
  if ((uint32_t)(now - gSettleT0) < gSettleMs) return;
  gSettleActive = false;
  gSettleMs = 0;
  replyValveDone(true);
}

static void serviceTimedCut(uint32_t now) {
  if (!gTimedCutActive) return;
  if ((uint32_t)(now - gTimedCutT0) < gTimedCutMs) return;
  for (uint8_t id = PLC_VALVE_CUTTER_R; id <= PLC_VALVE_CUTTER_L; id++) {
    if (gTimedCutMask & (1 << id)) digitalWrite(kValvePin[id], LOW);
  }
  gTimedCutActive = false;
  gTimedCutMask = 0;
  replyValveDone(true);
}

static void blowerOff() {
  digitalWrite(PLC_PIN_OUT_BLOWER, LOW);
  gBlowerOn = false;
}

static uint32_t blowerOn(uint32_t ms) {
  if (ms < PLC_BLOWER_MIN_MS) ms = PLC_BLOWER_MIN_MS;
  if (ms > PLC_BLOWER_MAX_MS) ms = PLC_BLOWER_MAX_MS;
  digitalWrite(PLC_PIN_OUT_BLOWER, HIGH);
  gBlowerOn = true;
  gBlowerT0 = millis();
  gBlowerMs = ms;
  return ms;
}

static void serviceBlower(uint32_t now) {
  if (!gBlowerOn) return;
  if ((uint32_t)(now - gBlowerT0) >= gBlowerMs) blowerOff();
}

static bool allOff() {
  blowerOff();
  for (uint8_t id = 0; id < PLC_VALVE_COUNT; id++) {
    digitalWrite(kValvePin[id], LOW);
  }
  gTimedCutActive = false;
  gTimedCutMask = 0;
  gSettleActive = false;
  gSettleMs = 0;
  return true;
}

static bool plcReset() {
  allOff();
  valveOutputsIdle();
  if (gResetActive) return false;
  digitalWrite(PLC_PIN_OUT_RESET, HIGH);
  gResetActive = true;
  gResetT0 = millis();
  return true;
}

static void serviceReset(uint32_t now) {
  if (!gResetActive) return;
  if ((uint32_t)(now - gResetT0) < PLC_RESET_PULSE_MS) return;
  digitalWrite(PLC_PIN_OUT_RESET, LOW);
  gResetActive = false;
  replyOk("reset", true);
}

// ── Sensores ────────────────────────────────────────────────────────────────

static uint8_t readSensorsRaw() {
  uint8_t mask = 0;
  if (digitalRead(PLC_PIN_IN_CUTTER) == LOW) mask |= PLC_BIT_CUTTER;
  if (digitalRead(PLC_PIN_IN_GRIPPER) == LOW) mask |= PLC_BIT_GRIPPER;
  if (digitalRead(PLC_PIN_IN_HOLDER) == LOW) mask |= PLC_BIT_HOLDER;
  if (digitalRead(PLC_PIN_IN_ENCODER) == LOW) mask |= PLC_BIT_ENCODER;
  return mask;
}

static uint8_t reportableFaults() {
  uint8_t mask = gStableValid ? gStable : 0;
  if (!gEncoderArmed) mask = (uint8_t)(mask & ~PLC_BIT_ENCODER);
  return mask;
}

static void printFaultJson(uint8_t mask) {
  Serial.print(F("{\"op\":\"fault\",\"mask\":"));
  Serial.print(mask);
  Serial.print(F(",\"cutter\":"));
  printBool((mask & PLC_BIT_CUTTER) != 0);
  Serial.print(F(",\"gripper\":"));
  printBool((mask & PLC_BIT_GRIPPER) != 0);
  Serial.print(F(",\"holder\":"));
  printBool((mask & PLC_BIT_HOLDER) != 0);
  Serial.print(F(",\"encoder\":"));
  printBool((mask & PLC_BIT_ENCODER) != 0);
  Serial.print(F(",\"err_cutter\":"));
  printHexByte(PLC_ERR_CUTTER);
  Serial.print(F(",\"err_gripper\":"));
  printHexByte(PLC_ERR_GRIPPER);
  Serial.print(F(",\"err_holder\":"));
  printHexByte(PLC_ERR_HOLDER);
  Serial.print(F(",\"err_encoder\":"));
  printHexByte(PLC_ERR_ENCODER);
  Serial.println(F("}"));
}

static void serviceSensors(uint32_t now) {
  if (digitalRead(PLC_PIN_IN_ENCODER) == HIGH) gEncoderArmed = true;

  uint8_t raw = readSensorsRaw();
  if (raw != gLastRaw) {
    gLastRaw = raw;
    gRawMs = now;
    return;
  }
  if ((uint32_t)(now - gRawMs) < PLC_SENSOR_DEBOUNCE_MS) return;

  gStable = raw;
  gStableValid = true;

  if ((uint32_t)(now - gBootMs) < PLC_BOOT_GRACE_MS) return;

  uint8_t reported = reportableFaults();
  if (reported == gReportedFaults) return;
  if (gReportedFaults == 0xFF && reported == 0) {
    gReportedFaults = 0;
    return;
  }
  gReportedFaults = reported;
  printFaultJson(reported);
}

static void printStatusJson() {
  bool grace = (uint32_t)(millis() - gBootMs) < PLC_BOOT_GRACE_MS;
  uint8_t sens = gStableValid ? gStable : 0;
  uint8_t faults = grace ? 0 : reportableFaults();

  Serial.print(F("{\"op\":\"status\""));
  Serial.print(F(",\"cutter_r\":"));
  printBool(digitalRead(PLC_PIN_OUT_CUTTER_R) == HIGH);
  Serial.print(F(",\"cutter_l\":"));
  printBool(digitalRead(PLC_PIN_OUT_CUTTER_L) == HIGH);
  Serial.print(F(",\"gripper\":"));
  printBool(digitalRead(PLC_PIN_OUT_GRIPPER) == HIGH);
  Serial.print(F(",\"holder\":"));
  printBool(digitalRead(PLC_PIN_OUT_HOLDER) == HIGH);
  Serial.print(F(",\"encoder\":"));
  printBool(digitalRead(PLC_PIN_OUT_ENCODER) == HIGH);
  Serial.print(F(",\"blower\":"));
  printBool(gBlowerOn);
  Serial.print(F(",\"timed_cut_busy\":"));
  printBool(gTimedCutActive);
  Serial.print(F(",\"settle_busy\":"));
  printBool(gSettleActive);
  Serial.print(F(",\"sens_mask\":"));
  Serial.print(sens);
  Serial.print(F(",\"sens_cutter\":"));
  printBool((sens & PLC_BIT_CUTTER) != 0);
  Serial.print(F(",\"sens_gripper\":"));
  printBool((sens & PLC_BIT_GRIPPER) != 0);
  Serial.print(F(",\"sens_holder\":"));
  printBool((sens & PLC_BIT_HOLDER) != 0);
  Serial.print(F(",\"sens_encoder\":"));
  printBool((sens & PLC_BIT_ENCODER) != 0);
  Serial.print(F(",\"fault_cutter\":"));
  printBool((faults & PLC_BIT_CUTTER) != 0);
  Serial.print(F(",\"fault_gripper\":"));
  printBool((faults & PLC_BIT_GRIPPER) != 0);
  Serial.print(F(",\"fault_holder\":"));
  printBool((faults & PLC_BIT_HOLDER) != 0);
  Serial.print(F(",\"fault_encoder\":"));
  printBool((faults & PLC_BIT_ENCODER) != 0);
  Serial.print(F(",\"encoder_armed\":"));
  printBool(gEncoderArmed);
  Serial.print(F(",\"boot_grace\":"));
  printBool(grace);
  Serial.println(F("}"));
}

// ── Serial ──────────────────────────────────────────────────────────────────

static bool jsonReadOn(const char* json, bool* out) {
  const char* p = nullptr;
  if (!jsonFindKey(json, "on", &p) || !out) return false;
  if (strncmp(p, "true", 4) == 0 || *p == '1') {
    *out = true;
    return true;
  }
  if (strncmp(p, "false", 5) == 0 || *p == '0') {
    *out = false;
    return true;
  }
  return false;
}

static void replyOk(const char* op, bool ok) {
  Serial.print(F("{\"op\":\""));
  Serial.print(op);
  Serial.print(F("\",\"ok\":"));
  printBool(ok);
  Serial.println(F("}"));
}

static void replyValveDone(bool ok) {
  bool grace = (uint32_t)(millis() - gBootMs) < PLC_BOOT_GRACE_MS;
  uint8_t faults = grace ? 0 : reportableFaults();

  Serial.print(F("{\"op\":\"valve\",\"ok\":"));
  printBool(ok);
  Serial.print(F(",\"cutter_r\":"));
  printBool(digitalRead(PLC_PIN_OUT_CUTTER_R) == HIGH);
  Serial.print(F(",\"cutter_l\":"));
  printBool(digitalRead(PLC_PIN_OUT_CUTTER_L) == HIGH);
  Serial.print(F(",\"gripper\":"));
  printBool(digitalRead(PLC_PIN_OUT_GRIPPER) == HIGH);
  Serial.print(F(",\"holder\":"));
  printBool(digitalRead(PLC_PIN_OUT_HOLDER) == HIGH);
  Serial.print(F(",\"encoder\":"));
  printBool(digitalRead(PLC_PIN_OUT_ENCODER) == HIGH);
  Serial.print(F(",\"blower\":"));
  printBool(gBlowerOn);
  Serial.print(F(",\"fault_cutter\":"));
  printBool((faults & PLC_BIT_CUTTER) != 0);
  Serial.print(F(",\"fault_gripper\":"));
  printBool((faults & PLC_BIT_GRIPPER) != 0);
  Serial.print(F(",\"fault_holder\":"));
  printBool((faults & PLC_BIT_HOLDER) != 0);
  Serial.print(F(",\"fault_encoder\":"));
  printBool((faults & PLC_BIT_ENCODER) != 0);
  Serial.println(F("}"));
}

static void handleSerialLine(char* line) {
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  if (strstr(line, "\"op\":\"ping\"") || strstr(line, "\"op\": \"ping\"")) {
    Serial.println(F("{\"op\":\"pong\"}"));
    return;
  }
  if (strstr(line, "\"op\":\"status\"")) {
    printStatusJson();
    return;
  }
  if (strstr(line, "\"op\":\"valve\"")) {
    const char* name = nullptr;
    bool on = false;
    bool ok = jsonFindKey(line, "name", &name) && *name == '"' && jsonReadOn(line, &on);
    uint32_t ms = (uint32_t)jsonReadLong(line, "ms", 0);
    uint32_t settleMs = (uint32_t)jsonReadLong(line, "settle_ms", 0);
    if (!ok) {
      replyValveDone(false);
      return;
    }
    if (on && ms > 0) {
      ok = (tokenIs(name + 1, "cutters") || tokenIs(name + 1, "cutter_r") ||
            tokenIs(name + 1, "cutter_l")) &&
           startTimedCut(name + 1, ms);
      if (!ok) replyValveDone(false);
      return;
    }
    if (!applyValveName(name + 1, on)) {
      replyValveDone(false);
      return;
    }
    if (!startSettle(settleMs)) replyValveDone(false);
    return;
  }
  if (strstr(line, "\"op\":\"blower_on\"")) {
    uint32_t ms = blowerOn((uint32_t)jsonReadLong(line, "ms", (long)PLC_BLOWER_DEFAULT_MS));
    Serial.print(F("{\"op\":\"blower_on\",\"ok\":true,\"ms\":"));
    Serial.print(ms);
    Serial.println(F("}"));
    return;
  }
  if (strstr(line, "\"op\":\"blower_off\"")) {
    blowerOff();
    replyOk("blower_off", true);
    return;
  }
  if (strstr(line, "\"op\":\"all_off\"")) {
    replyOk("all_off", allOff());
    return;
  }
  if (strstr(line, "\"op\":\"reset\"")) {
    if (!plcReset()) replyOk("reset", false);
    return;
  }

  Serial.println(F("{\"op\":\"error\",\"msg\":\"unknown_command\"}"));
}

static void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      gSerialLine[gSerialLineLen] = '\0';
      handleSerialLine(gSerialLine);
      gSerialLineLen = 0;
      continue;
    }
    if (gSerialLineLen + 1 < sizeof(gSerialLine)) {
      gSerialLine[gSerialLineLen++] = c;
    }
  }
}

// ── Arduino ─────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(PARAM_SERIAL_BAUD);
  initIo();
  gBootMs = millis();
  gLastRaw = readSensorsRaw();
  gRawMs = gBootMs;
  if (digitalRead(PLC_PIN_IN_ENCODER) == HIGH) gEncoderArmed = true;

  Serial.println(F("{\"op\":\"boot\",\"mcu\":\"plc\"}"));
  printStatusJson();
}

void loop() {
  pollSerial();
  uint32_t now = millis();
  serviceTimedCut(now);
  serviceSettle(now);
  serviceReset(now);
  serviceBlower(now);
  serviceSensors(now);
}
