#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>

#include "Parameters.h"
#include "Encoder.h"

struct MotionRuntime {
  bool feederReady[MOTION_SIDE_COUNT];
  bool feederServoOn[MOTION_SIDE_COUNT];
  bool lmguideServoOn;
  bool lmguideHomed;
};

#include "ServoFeeder.h"
#include "ServoLMGuide.h"

static MotionTune gTune;
static MotionData gData;
static MotionRuntime gRt;
static Preferences gPrefs;
static EncoderPCNT gEncoders;

static uint32_t gLastEncoderPollMs = 0;
static bool gFeedStageBusy = false;
static volatile bool gLmAbortRequested = false;
static volatile bool gFeedAbortRequested = false;

enum FeedStageError {
  FEED_STAGE_OK = 0,
  FEED_STAGE_ERR_BUSY,
  FEED_STAGE_ERR_NO_TARGET,
  FEED_STAGE_ERR_FEEDER,
  FEED_STAGE_ERR_LASER_R,
  FEED_STAGE_ERR_LASER_L,
  FEED_STAGE_ERR_ENC_R,
  FEED_STAGE_ERR_ENC_L,
  FEED_STAGE_ERR_COMP,
  FEED_STAGE_ERR_SAFETY,
  FEED_STAGE_ERR_STOP,
  FEED_STAGE_ERR_JOINT_R,
  FEED_STAGE_ERR_JOINT_L,
};

struct FeedStageSideResult {
  bool laserOn;
  bool encOk;
  int32_t countsMeasured;
};

struct FeedStageResult {
  FeedStageError error;
  FeedStageSideResult side[MOTION_SIDE_COUNT];
};

// Sin "side" y sin "mode" el ciclo usa ambos lados, lee encoder (counts) y el timeout de Parameters.h.
// "mode":"laser" (purga): cada lado sigue hasta su láser. No aplica compensación.
// "mode":"clean" alimenta el tiempo indicado y no se detiene en el láser.
// Si el láser de un lado ya está activo, ese lado no arranca. La compensación de ese lado
// se manda una sola vez mientras el láser siga activo.
struct FeedStageRequest {
  bool useR;
  bool useL;
  bool checkEncoder;
  bool ignoreLaser;
  uint32_t laserTimeoutMs;
};

static FeedStageResult gLastFeedStage = {};
static bool gFeedOffsetApplied[MOTION_SIDE_COUNT] = {false, false};

/* Diagnóstico: conteo al láser, tras Halt TX, asiento real y status CAN. */
struct LaserStopDiag {
  bool active;
  bool printed;
  uint32_t tLaserMs;
  uint32_t tLaserAbsMs;
  int32_t countsAtLaser;
  int32_t countsAfterHalt;
  int32_t countsSettled;
  uint32_t settleMs;
  bool stopOk;
  bool settled;
  bool statusOk;
  uint16_t statusWord;
  bool velOk;
  int32_t velActual;
  int32_t settleLast;
  uint32_t settleStableSinceMs;
};

static LaserStopDiag gLaserStopDiag[MOTION_SIDE_COUNT];

// ── Utilidades JSON plano (Serial) ──────────────────────────────────────────

static bool jsonFindKey(const char* json, const char* key, const char** valOut) {
  char needle[48];
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

static float jsonReadFloat(const char* json, const char* key, float defVal) {
  const char* p = nullptr;
  if (!jsonFindKey(json, key, &p)) return defVal;
  return strtof(p, nullptr);
}

static long jsonReadLong(const char* json, const char* key, long defVal) {
  const char* p = nullptr;
  if (!jsonFindKey(json, key, &p)) return defVal;
  return strtol(p, nullptr, 10);
}

static bool jsonReadBool(const char* json, const char* key, bool defVal) {
  const char* p = nullptr;
  if (!jsonFindKey(json, key, &p)) return defVal;
  if (strncmp(p, "true", 4) == 0 || *p == '1') return true;
  if (strncmp(p, "false", 5) == 0 || *p == '0') return false;
  return defVal;
}

static void runtimeInit(MotionRuntime* rt) {
  memset(rt, 0, sizeof(MotionRuntime));
}

static void dataLoadNvs() {
  motionDataSetDefaults(&gData);
  if (!gPrefs.begin("motion", true)) return;
  gData.targetLengthMm = gPrefs.getFloat("tgt_mm", gData.targetLengthMm);
  gData.toleranceMm = gPrefs.getFloat("tol_mm", gData.toleranceMm);
  gData.measureCompMm = gPrefs.getFloat("comp_mm", gData.measureCompMm);
  String model = gPrefs.getString("model", "");
  model.toCharArray(gData.modeloId, sizeof(gData.modeloId));
  gPrefs.end();
}

static void dataSaveNvs() {
  if (!gPrefs.begin("motion", false)) return;
  gPrefs.putFloat("tgt_mm", gData.targetLengthMm);
  gPrefs.putFloat("tol_mm", gData.toleranceMm);
  gPrefs.putFloat("comp_mm", gData.measureCompMm);
  gPrefs.putString("model", gData.modeloId);
  gPrefs.end();
}

static MotionSide sideFromToken(const char* tok) {
  if (!tok) return MOTION_SIDE_R;
  if (tok[0] == 'L' || tok[0] == 'l') return MOTION_SIDE_L;
  return MOTION_SIDE_R;
}

static void pollSerial();
static void motionYieldMs(uint32_t ms);
static bool safetyIsActive();

static EncoderPCNT::Reading encoderReading(MotionSide side) {
  return (side == MOTION_SIDE_R) ? gEncoders.right() : gEncoders.left();
}

static int32_t encoderCountsOfficial(MotionSide side) {
  int64_t c = encoderReading(side).count;
  if (c < 0) c = -c;
  if (c > 2147483647LL) c = 2147483647LL;
  return (int32_t)c;
}

static void encoderSetZero(MotionSide side) {
  if (side == MOTION_SIDE_R) gEncoders.setZeroRight();
  else gEncoders.setZeroLeft();
}

static bool encoderResetHw(MotionSide side) {
  return (side == MOTION_SIDE_R) ? gEncoders.resetRight() : gEncoders.resetLeft();
}

static bool feedApplyOffsetOnce(MotionSide side) {
  if (gFeedOffsetApplied[side]) return true;
  if (!feederApplySideComp(side)) return false;
  int32_t counts = (int32_t)lroundf(gTune.feeder[side].compensationMm * SERVO_FEED_STEPS_PER_MM_S);
  if (counts != 0) gFeedOffsetApplied[side] = true;
  return true;
}

static void laserStopDiagResetAll() {
  memset(gLaserStopDiag, 0, sizeof(gLaserStopDiag));
}

static void laserStopDiagBegin(MotionSide side, uint32_t tLaserMs, int32_t countsAtLaser,
                               bool stopOk, int32_t countsAfterHalt) {
  LaserStopDiag& d = gLaserStopDiag[side];
  d = {};
  d.active = true;
  d.tLaserMs = tLaserMs;
  d.tLaserAbsMs = millis();
  d.countsAtLaser = countsAtLaser;
  d.countsAfterHalt = countsAfterHalt;
  d.countsSettled = countsAfterHalt;
  d.stopOk = stopOk;
  d.settleLast = countsAfterHalt;
  d.settleStableSinceMs = d.tLaserAbsMs;
}

static void printLaserStopTrace(MotionSide side, const LaserStopDiag& d) {
  char s = (side == MOTION_SIDE_R) ? 'R' : 'L';
  int32_t overrun = d.countsSettled - d.countsAtLaser;
  Serial.printf(
      "{\"op\":\"laser_stop_trace\",\"side\":\"%c\",\"t_laser_ms\":%lu,"
      "\"counts_at_laser\":%ld,\"counts_after_halt\":%ld,\"counts_settled\":%ld,"
      "\"overrun_counts\":%ld,\"settle_ms\":%lu,\"stop_ok\":%s,"
      "\"status_ok\":%s,\"status_word\":%u,\"vel_ok\":%s,\"vel_actual\":%ld}\n",
      s, (unsigned long)d.tLaserMs, (long)d.countsAtLaser, (long)d.countsAfterHalt,
      (long)d.countsSettled, (long)overrun, (unsigned long)d.settleMs,
      d.stopOk ? "true" : "false", d.statusOk ? "true" : "false",
      (unsigned)d.statusWord, d.velOk ? "true" : "false", (long)d.velActual);
}

static void laserStopDiagPoll(uint32_t nowMs) {
  for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
    LaserStopDiag& d = gLaserStopDiag[i];
    if (!d.active || d.printed || d.settled) continue;

    int32_t c = encoderCountsOfficial((MotionSide)i);
    if (c != d.settleLast) {
      d.settleLast = c;
      d.settleStableSinceMs = nowMs;
    }
    bool stable = (nowMs - d.settleStableSinceMs) >= PARAM_LASER_STOP_STABLE_MS;
    bool timedOut = (nowMs - d.tLaserAbsMs) >= PARAM_LASER_STOP_SETTLE_MS;
    if (!stable && !timedOut) continue;

    d.countsSettled = d.settleLast;
    d.settleMs = nowMs - d.tLaserAbsMs;
    d.settled = true;
  }
}

static void laserStopDiagFinalize() {
  uint32_t t0 = millis();
  while ((millis() - t0) < PARAM_LASER_STOP_SETTLE_MS) {
    bool anyPending = false;
    for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
      if (gLaserStopDiag[i].active && !gLaserStopDiag[i].settled && !gLaserStopDiag[i].printed)
        anyPending = true;
    }
    if (!anyPending) break;
    motionPollEncoders();
    pollSerial();
    laserStopDiagPoll(millis());
    delay(1);
  }

  for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
    LaserStopDiag& d = gLaserStopDiag[i];
    if (!d.active || d.printed) continue;
    MotionSide side = (MotionSide)i;
    if (!d.settled) {
      gEncoders.update();
      d.countsSettled = encoderCountsOfficial(side);
      d.settleMs = millis() - d.tLaserAbsMs;
      d.settled = true;
    }
    uint16_t sw = 0;
    d.statusOk = canReadStatusWord(side, &sw, PARAM_LASER_STOP_STATUS_MS);
    d.statusWord = sw;
    int32_t vel = 0;
    d.velOk = canReadSdoI32(feederNode(side), SERVO_OD_VELOCITY_ACTUAL, 0, &vel,
                            PARAM_LASER_STOP_STATUS_MS);
    d.velActual = vel;
    printLaserStopTrace(side, d);
    d.printed = true;
  }
}

// ── FeedStage (Bloque 1 — alimentación y medición) ──────────────────────────

static void initLasersHw() {
  pinMode(LASER_PIN_R, INPUT_PULLUP);
  pinMode(LASER_PIN_L, INPUT);
  pinMode(JOINT_PIN_R, INPUT_PULLUP);
  pinMode(JOINT_PIN_L, INPUT_PULLUP);
  pinMode(SAFETY_PIN, INPUT_PULLUP);
}

static bool safetyIsActive() {
  return digitalRead(SAFETY_PIN) == HIGH;
}

static bool laserIsOn(MotionSide side) {
  int pin = (side == MOTION_SIDE_R) ? LASER_PIN_R : LASER_PIN_L;
  int level = digitalRead(pin);
  if (LASER_ACTIVE_HIGH) return level == HIGH;
  return level == LOW;
}

static bool gJointOn[MOTION_SIDE_COUNT] = {false, false};
static uint32_t gJointEdgeMs[MOTION_SIDE_COUNT] = {0, 0};

static bool jointRawOn(MotionSide side) {
  int pin = (side == MOTION_SIDE_R) ? JOINT_PIN_R : JOINT_PIN_L;
  int level = digitalRead(pin);
  if (JOINT_ACTIVE_HIGH) return level == HIGH;
  return level == LOW;
}

/** Filtra picos: el joint solo cuenta ON/OFF si el nivel se mantiene PARAM_JOINT_DEBOUNCE_MS. */
static void jointPoll(uint32_t nowMs) {
  for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
    bool raw = jointRawOn((MotionSide)i);
    if (raw == gJointOn[i]) {
      gJointEdgeMs[i] = 0;
      continue;
    }
    if (gJointEdgeMs[i] == 0) {
      gJointEdgeMs[i] = nowMs;
    } else if ((nowMs - gJointEdgeMs[i]) >= PARAM_JOINT_DEBOUNCE_MS) {
      gJointOn[i] = raw;
      gJointEdgeMs[i] = 0;
    }
  }
}

static bool jointIsOn(MotionSide side) {
  jointPoll(millis());
  return gJointOn[side];
}

static void motionPollEncoders() {
  gEncoders.update();
}

static const char* feedStageErrorName(FeedStageError err) {
  switch (err) {
    case FEED_STAGE_OK:
      return "ok";
    case FEED_STAGE_ERR_BUSY:
      return "busy";
    case FEED_STAGE_ERR_NO_TARGET:
      return "no_target";
    case FEED_STAGE_ERR_FEEDER:
      return "feeder_not_ready";
    case FEED_STAGE_ERR_LASER_R:
      return "laser_timeout_r";
    case FEED_STAGE_ERR_LASER_L:
      return "laser_timeout_l";
    case FEED_STAGE_ERR_ENC_R:
      return "encoder_tolerance_r";
    case FEED_STAGE_ERR_ENC_L:
      return "encoder_tolerance_l";
    case FEED_STAGE_ERR_COMP:
      return "comp_move";
    case FEED_STAGE_ERR_SAFETY:
      return "safety";
    case FEED_STAGE_ERR_STOP:
      return "stopped";
    case FEED_STAGE_ERR_JOINT_R:
      return "joint_r";
    case FEED_STAGE_ERR_JOINT_L:
      return "joint_l";
    default:
      return "unknown";
  }
}

static FeedStageRequest feedStageDefaultRequest() {
  FeedStageRequest req;
  req.useR = true;
  req.useL = true;
  req.checkEncoder = gTune.validateEncoder;
  req.ignoreLaser = false;
  req.laserTimeoutMs = PARAM_FEED_STAGE_LASER_TIMEOUT_MS;
  return req;
}

static FeedStageResult FeedStage(FeedStageRequest req) {
  FeedStageResult out = {};
  gFeedAbortRequested = false;
  gLmAbortRequested = false;
  if (gFeedStageBusy) {
    out.error = FEED_STAGE_ERR_BUSY;
    return out;
  }
  if ((req.useR && !gRt.feederReady[MOTION_SIDE_R]) ||
      (req.useL && !gRt.feederReady[MOTION_SIDE_L])) {
    out.error = FEED_STAGE_ERR_FEEDER;
    return out;
  }
  if (safetyIsActive()) {
    out.error = FEED_STAGE_ERR_SAFETY;
    return out;
  }

  gFeedStageBusy = true;
  laserStopDiagResetAll();

  bool sideAtLaser[MOTION_SIDE_COUNT] = {false, false};
  if (req.ignoreLaser) {
    if (req.useR) gFeedOffsetApplied[MOTION_SIDE_R] = false;
    if (req.useL) gFeedOffsetApplied[MOTION_SIDE_L] = false;
  } else {
    if (req.useR) {
      sideAtLaser[MOTION_SIDE_R] = laserIsOn(MOTION_SIDE_R);
      if (!sideAtLaser[MOTION_SIDE_R]) gFeedOffsetApplied[MOTION_SIDE_R] = false;
    }
    if (req.useL) {
      sideAtLaser[MOTION_SIDE_L] = laserIsOn(MOTION_SIDE_L);
      if (!sideAtLaser[MOTION_SIDE_L]) gFeedOffsetApplied[MOTION_SIDE_L] = false;
    }
  }

  bool runR = req.useR && !sideAtLaser[MOTION_SIDE_R];
  bool runL = req.useL && !sideAtLaser[MOTION_SIDE_L];
  bool sideFeedStopped[MOTION_SIDE_COUNT] = {!runR, !runL};
  bool sideValidated[MOTION_SIDE_COUNT] = {!req.useR, !req.useL};

  // PCNT continuo: una marca relativa por lado al inicio del FeedStage.
  gEncoders.update();
  if (req.useR) encoderSetZero(MOTION_SIDE_R);
  if (req.useL) encoderSetZero(MOTION_SIDE_L);

  if (sideAtLaser[MOTION_SIDE_R]) {
    out.side[MOTION_SIDE_R].laserOn = true;
    out.side[MOTION_SIDE_R].countsMeasured = encoderCountsOfficial(MOTION_SIDE_R);
    out.side[MOTION_SIDE_R].encOk = true;
    sideValidated[MOTION_SIDE_R] = true;
  }
  if (sideAtLaser[MOTION_SIDE_L]) {
    out.side[MOTION_SIDE_L].laserOn = true;
    out.side[MOTION_SIDE_L].countsMeasured = encoderCountsOfficial(MOTION_SIDE_L);
    out.side[MOTION_SIDE_L].encOk = true;
    sideValidated[MOTION_SIDE_L] = true;
  }

  FeedStageError fail = FEED_STAGE_OK;
  if (runR) {
    if (!feederPreparePv(MOTION_SIDE_R)) fail = FEED_STAGE_ERR_FEEDER;
  }
  if (fail == FEED_STAGE_OK && runL) {
    if (!feederPreparePv(MOTION_SIDE_L)) fail = FEED_STAGE_ERR_FEEDER;
  }
  if (fail == FEED_STAGE_OK && (runR || runL)) {
    motionYieldMs(SERVO_CAN_CIA402_SHORT_MS);
    /* Stop puede llegar en el yield: no rearmar PV ni limpiar Halt. */
    if (gLmAbortRequested || gFeedAbortRequested) {
      fail = FEED_STAGE_ERR_STOP;
    } else {
      if (runR && !feederSetPvVelocity(MOTION_SIDE_R, feederFeedVelocity(MOTION_SIDE_R)))
        fail = FEED_STAGE_ERR_FEEDER;
      if (fail == FEED_STAGE_OK && runL &&
          !feederSetPvVelocity(MOTION_SIDE_L, feederFeedVelocity(MOTION_SIDE_L)))
        fail = FEED_STAGE_ERR_FEEDER;
    }
  }
  if (fail == FEED_STAGE_OK && (runR || runL)) {
    if (gLmAbortRequested || gFeedAbortRequested) {
      fail = FEED_STAGE_ERR_STOP;
    } else {
      if (runR && !feederReleasePv(MOTION_SIDE_R)) fail = FEED_STAGE_ERR_FEEDER;
      if (fail == FEED_STAGE_OK && runL && !feederReleasePv(MOTION_SIDE_L))
        fail = FEED_STAGE_ERR_FEEDER;
    }
  }

  uint32_t tStart = millis();
  while (fail == FEED_STAGE_OK) {
    pollSerial();
    uint32_t nowMs = millis();
    motionPollEncoders();

    if (gLmAbortRequested || gFeedAbortRequested) {
      fail = FEED_STAGE_ERR_STOP;
      break;
    }

    if (safetyIsActive()) {
      fail = FEED_STAGE_ERR_SAFETY;
      break;
    }

    // Joint: detener el lado afectado y abortar FeedStage con error por lado.
    if (req.useR && jointIsOn(MOTION_SIDE_R)) {
      if (!sideFeedStopped[MOTION_SIDE_R]) {
        feederStop(MOTION_SIDE_R);
        canSdoDownloadU32(feederNode(MOTION_SIDE_R), SERVO_OD_TARGET_VELOCITY, 0, 0);
        sideFeedStopped[MOTION_SIDE_R] = true;
      }
      fail = FEED_STAGE_ERR_JOINT_R;
      break;
    }
    if (req.useL && jointIsOn(MOTION_SIDE_L)) {
      if (!sideFeedStopped[MOTION_SIDE_L]) {
        feederStop(MOTION_SIDE_L);
        canSdoDownloadU32(feederNode(MOTION_SIDE_L), SERVO_OD_TARGET_VELOCITY, 0, 0);
        sideFeedStopped[MOTION_SIDE_L] = true;
      }
      fail = FEED_STAGE_ERR_JOINT_L;
      break;
    }

    if (req.ignoreLaser) {
      if (nowMs - tStart >= req.laserTimeoutMs) {
        out.error = FEED_STAGE_OK;
        break;
      }
    } else {
    for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
      if (sideValidated[i]) continue;
      MotionSide side = (MotionSide)i;

      if (!sideFeedStopped[i] && laserIsOn(side)) {
        gEncoders.update();
        int32_t countsAtLaser = encoderCountsOfficial(side);
        bool stopOk = feederStop(side);
        stopOk = canSdoDownloadU32(feederNode(side), SERVO_OD_TARGET_VELOCITY, 0, 0) && stopOk;
        gEncoders.update();
        int32_t countsAfterHalt = encoderCountsOfficial(side);
        laserStopDiagBegin(side, nowMs - tStart, countsAtLaser, stopOk, countsAfterHalt);
        if (!stopOk) {
          fail = FEED_STAGE_ERR_FEEDER;
          break;
        }
        sideFeedStopped[i] = true;
        out.side[i].laserOn = true;
        out.side[i].countsMeasured = countsAfterHalt;
        out.side[i].encOk = true;
        sideValidated[i] = true;
      }

      if (!sideFeedStopped[i] && (nowMs - tStart >= req.laserTimeoutMs)) {
        fail = (side == MOTION_SIDE_R) ? FEED_STAGE_ERR_LASER_R : FEED_STAGE_ERR_LASER_L;
        break;
      }
    }
    }

    if (fail != FEED_STAGE_OK) break;

    laserStopDiagPoll(nowMs);

    if (sideValidated[MOTION_SIDE_R] && sideValidated[MOTION_SIDE_L]) {
      out.error = FEED_STAGE_OK;
      break;
    }
    delay(1);
  }

  if (!feederStopAll() && fail == FEED_STAGE_OK) fail = FEED_STAGE_ERR_FEEDER;
  laserStopDiagFinalize();
  if (fail != FEED_STAGE_OK) {
    out.error = fail;
  } else if (req.checkEncoder) {
    bool compOk = true;
    if (req.useR) compOk = feedApplyOffsetOnce(MOTION_SIDE_R);
    if (compOk && req.useL) compOk = feedApplyOffsetOnce(MOTION_SIDE_L);
    if (!compOk) out.error = FEED_STAGE_ERR_COMP;
  }

  gLastFeedStage = out;
  gFeedStageBusy = false;
  return out;
}

static void printFeedStageResultJson(const FeedStageResult& r) {
  Serial.print(F("{\"op\":\"feed_stage\",\"ok\":"));
  Serial.print(r.error == FEED_STAGE_OK ? "true" : "false");
  Serial.print(F(",\"error\":\""));
  Serial.print(feedStageErrorName(r.error));
  Serial.print(F("\",\"R\":{\"laser\":"));
  Serial.print(r.side[MOTION_SIDE_R].laserOn ? "true" : "false");
  Serial.print(F(",\"enc_ok\":"));
  Serial.print(r.side[MOTION_SIDE_R].encOk ? "true" : "false");
  Serial.print(F(",\"counts\":"));
  Serial.print(r.side[MOTION_SIDE_R].countsMeasured);
  Serial.print(F(",\"comp_mm\":"));
  Serial.print(gTune.feeder[MOTION_SIDE_R].compensationMm, 2);
  Serial.print(F("},\"L\":{\"laser\":"));
  Serial.print(r.side[MOTION_SIDE_L].laserOn ? "true" : "false");
  Serial.print(F(",\"enc_ok\":"));
  Serial.print(r.side[MOTION_SIDE_L].encOk ? "true" : "false");
  Serial.print(F(",\"counts\":"));
  Serial.print(r.side[MOTION_SIDE_L].countsMeasured);
  Serial.print(F(",\"comp_mm\":"));
  Serial.print(gTune.feeder[MOTION_SIDE_L].compensationMm, 2);
  Serial.println(F("}}"));
}

// ── LM Guide (ciclo: model/named; HW en ServoLMGuide.h) ───────────────────

static bool lmguideMoveModel() {
  if (!gRt.lmguideHomed) return false;
  if (gData.targetLengthMm <= 0.0f) return false;
  float mm = gData.targetLengthMm + gData.measureCompMm;
  int32_t puu = (int32_t)lroundf(-mm * ASDA_FACTORY_STEPS_PER_MM);
  return lmguideMovePr1(puu);
}

static bool lmguideMoveNamed(const char* name) {
  if (!name) return false;
  if (strncmp(name, "model", 5) == 0) return lmguideMoveModel();
  if (strncmp(name, "home", 4) == 0) {
    // PUU 0 absoluto (P6.002 TYPE=2 OPT=ABS). No es homing mecánico.
    return lmguideMovePr1(0);
  }
  if (!gRt.lmguideHomed) return false;
  return lmguideMovePr1(gTune.lmguide.posPuu);
}
static void motionHwInitRun() {
  if (safetyIsActive()) {
    Serial.println(F("{\"op\":\"motion_hw_init\",\"ok\":false,\"error\":\"safety\"}"));
    return;
  }
  bool okFeed = feederInitBus();
  if (!okFeed) {
    Serial.println(F("{\"op\":\"motion_hw_init\",\"ok\":false,\"error\":\"feeder_init\"}"));
    return;
  }
  bool okHome = lmguideHome();
  if (!okHome) {
    Serial.println(F("{\"op\":\"motion_hw_init\",\"ok\":false,\"error\":\"lmguide_home\"}"));
    return;
  }
  Serial.print(F("{\"op\":\"motion_hw_init\",\"ok\":true,\"feeder_init\":true,\"lmguide_home\":true"));
  Serial.print(F(",\"feeder_r_ready\":"));
  Serial.print(gRt.feederReady[MOTION_SIDE_R] ? "true" : "false");
  Serial.print(F(",\"feeder_l_ready\":"));
  Serial.print(gRt.feederReady[MOTION_SIDE_L] ? "true" : "false");
  Serial.print(F(",\"lm_homed\":"));
  Serial.print(gRt.lmguideHomed ? "true" : "false");
  Serial.println(F("}"));
}

// ── Serial / telemetría ─────────────────────────────────────────────────────

static void printStatusJson() {
  gEncoders.update();
  Serial.print(F("{\"op\":\"status\""));
  Serial.print(F(",\"data\":{\"target_mm\":"));
  Serial.print(gData.targetLengthMm, 2);
  Serial.print(F(",\"tol_mm\":"));
  Serial.print(gData.toleranceMm, 2);
  Serial.print(F(",\"comp_mm\":"));
  Serial.print(gData.measureCompMm, 2);
  Serial.print(F(",\"modelo\":\""));
  Serial.print(gData.modeloId);
  Serial.print(F("\"}"));

  const char* sideTag[] = {"R", "L"};
  for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
    MotionSide side = (MotionSide)i;
    EncoderPCNT::Reading er = encoderReading(side);
    Serial.print(F(",\"enc_"));
    Serial.print(sideTag[i]);
    Serial.print(F("\":{\"counts\":"));
    Serial.print(encoderCountsOfficial(side));
    Serial.print(F(",\"raw\":"));
    Serial.print((long)er.rawCount);
    Serial.print(F(",\"cps\":"));
    Serial.print(er.speedCps, 1);
    Serial.print(F(",\"dir\":"));
    Serial.print((int)er.direction);
    Serial.print(F(",\"ready\":"));
    Serial.print(er.ready ? "true" : "false");
    Serial.print(F(",\"laser\":"));
    Serial.print(laserIsOn(side) ? "true" : "false");
    Serial.print(F(",\"joint\":"));
    Serial.print(jointIsOn(side) ? "true" : "false");
    Serial.print(F("}"));
  }

  Serial.print(F(",\"feeder_r_ready\":"));
  Serial.print(gRt.feederReady[MOTION_SIDE_R] ? "true" : "false");
  Serial.print(F(",\"feeder_l_ready\":"));
  Serial.print(gRt.feederReady[MOTION_SIDE_L] ? "true" : "false");
  Serial.print(F(",\"feeder_r_servo_on\":"));
  Serial.print(gRt.feederServoOn[MOTION_SIDE_R] ? "true" : "false");
  Serial.print(F(",\"feeder_l_servo_on\":"));
  Serial.print(gRt.feederServoOn[MOTION_SIDE_L] ? "true" : "false");
  Serial.print(F(",\"lm_servo_on\":"));
  Serial.print(gRt.lmguideServoOn ? "true" : "false");
  Serial.print(F(",\"lm_homed\":"));
  Serial.print(gRt.lmguideHomed ? "true" : "false");
  Serial.print(F(",\"safety\":"));
  Serial.print(safetyIsActive() ? "true" : "false");
  Serial.println(F(",\"mcu\":\"motion\"}"));
}

static void applyTuneFromJson(const char* json) {
  gTune.feeder[MOTION_SIDE_R].profileVelocityPps =
      (uint32_t)jsonReadLong(json, "feed_r_vel_pps", (long)gTune.feeder[MOTION_SIDE_R].profileVelocityPps);
  gTune.feeder[MOTION_SIDE_R].targetPositionCounts =
      (int32_t)jsonReadLong(json, "feed_r_target_counts", gTune.feeder[MOTION_SIDE_R].targetPositionCounts);

  gTune.feeder[MOTION_SIDE_L].profileVelocityPps =
      (uint32_t)jsonReadLong(json, "feed_l_vel_pps", (long)gTune.feeder[MOTION_SIDE_L].profileVelocityPps);
  gTune.feeder[MOTION_SIDE_L].targetPositionCounts =
      (int32_t)jsonReadLong(json, "feed_l_target_counts", gTune.feeder[MOTION_SIDE_L].targetPositionCounts);

  gTune.feeder[MOTION_SIDE_R].compensationMm =
      jsonReadFloat(json, "feed_r_comp_mm", gTune.feeder[MOTION_SIDE_R].compensationMm);
  gTune.feeder[MOTION_SIDE_L].compensationMm =
      jsonReadFloat(json, "feed_l_comp_mm", gTune.feeder[MOTION_SIDE_L].compensationMm);

  gTune.lmguide.prSpeedRpm =
      lmClampSpeedRpm(jsonReadFloat(json, "lm_pr_speed_rpm", gTune.lmguide.prSpeedRpm));
  gTune.lmguide.posPuu = (int32_t)jsonReadLong(json, "lm_pos_puu", gTune.lmguide.posPuu);
  gTune.validateEncoder = jsonReadBool(json, "feed_validate_encoder", gTune.validateEncoder);
}

static void applyDataFromJson(const char* json) {
  gData.targetLengthMm = jsonReadFloat(json, "target_length_mm", gData.targetLengthMm);
  gData.toleranceMm = jsonReadFloat(json, "tolerance_mm", gData.toleranceMm);
  gData.measureCompMm = jsonReadFloat(json, "measure_comp_mm", gData.measureCompMm);

  const char* p = nullptr;
  if (jsonFindKey(json, "modelo_id", &p)) {
    if (*p == '"') {
      p++;
      size_t i = 0;
      while (*p && *p != '"' && i + 1 < sizeof(gData.modeloId)) {
        gData.modeloId[i++] = *p++;
      }
      gData.modeloId[i] = '\0';
    }
  }
  dataSaveNvs();
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
  if (strstr(line, "\"op\":\"set_tune\"")) {
    applyTuneFromJson(line);
    Serial.println(F("{\"op\":\"set_tune\",\"ok\":true}"));
    return;
  }
  if (strstr(line, "\"op\":\"set_data\"")) {
    applyDataFromJson(line);
    Serial.println(F("{\"op\":\"set_data\",\"ok\":true}"));
    return;
  }
  if (strstr(line, "\"op\":\"encoder_set0\"")) {
    const char* sideTok = strstr(line, "\"side\"");
    MotionSide side = MOTION_SIDE_R;
    if (sideTok) {
      const char* q = strchr(sideTok, ':');
      if (q) {
        q++;
        while (*q == ' ' || *q == '"') q++;
        side = sideFromToken(q);
      }
    }
    gEncoders.update();
    if (strstr(line, "\"side\":\"both\"")) {
      gEncoders.setZeroAll();
    } else {
      encoderSetZero(side);
    }
    Serial.println(F("{\"op\":\"encoder_set0\",\"ok\":true}"));
    return;
  }
  if (strstr(line, "\"op\":\"motion_hw_init\"")) {
    motionHwInitRun();
    return;
  }
  if (strstr(line, "\"op\":\"encoder_reset\"")) {
    bool ok = false;
    if (strstr(line, "\"side\":\"L\"")) {
      ok = encoderResetHw(MOTION_SIDE_L);
    } else if (strstr(line, "\"side\":\"R\"")) {
      ok = encoderResetHw(MOTION_SIDE_R);
    } else {
      ok = gEncoders.resetAll();
    }
    Serial.print(F("{\"op\":\"encoder_reset\",\"ok\":"));
    Serial.print(ok ? "true" : "false");
    Serial.println(F("}"));
    return;
  }
  if (strstr(line, "\"op\":\"feeder_init\"")) {
    if (strstr(line, "\"side\":\"both\"") || !strstr(line, "\"side\"")) {
      feederInitBus();
    } else if (strstr(line, "\"side\":\"L\"")) {
      bool ok = feederBringUp(MOTION_SIDE_L, feederProbe(MOTION_SIDE_L));
      Serial.print(F("{\"op\":\"feeder_init\",\"ok\":"));
      Serial.print(ok ? "true" : "false");
      Serial.print(F(",\"ok_l\":"));
      Serial.print(ok ? "true" : "false");
      Serial.println(F("}"));
    } else {
      bool ok = feederBringUp(MOTION_SIDE_R, feederProbe(MOTION_SIDE_R));
      Serial.print(F("{\"op\":\"feeder_init\",\"ok\":"));
      Serial.print(ok ? "true" : "false");
      Serial.print(F(",\"ok_r\":"));
      Serial.print(ok ? "true" : "false");
      Serial.println(F("}"));
    }
    return;
  }
  if (strstr(line, "\"op\":\"feeder_jog\"")) {
    if (safetyIsActive()) {
      Serial.println(F("{\"op\":\"feeder_jog\",\"ok\":false,\"error\":\"safety\"}"));
      return;
    }
    MotionSide side = MOTION_SIDE_R;
    if (strstr(line, "\"side\":\"L\"")) side = MOTION_SIDE_L;
    long dir = jsonReadLong(line, "dir", 1);
    bool ok = feederJogStep(side, dir >= 0 ? 1 : -1);
    Serial.print(F("{\"op\":\"feeder_jog\",\"ok\":"));
    Serial.print(ok ? "true" : "false");
    Serial.println(F("}"));
    return;
  }
  if (strstr(line, "\"op\":\"feeder_stop\"")) {
    gFeedAbortRequested = true;
    gLmAbortRequested = true;
    bool ok = feederStopAll();
    Serial.print(F("{\"op\":\"feeder_stop\",\"ok\":"));
    Serial.print(ok ? "true" : "false");
    Serial.println(F("}"));
    return;
  }
  if (strstr(line, "\"op\":\"feeder_servo_on\"")) {
    bool ok = false;
    if (strstr(line, "\"side\":\"L\"")) ok = feederServoOn(MOTION_SIDE_L);
    else if (strstr(line, "\"side\":\"R\"")) ok = feederServoOn(MOTION_SIDE_R);
    else ok = feederServoOnAll();
    Serial.print(F("{\"op\":\"feeder_servo_on\",\"ok\":"));
    Serial.print(ok ? "true" : "false");
    Serial.print(F(",\"feeder_r_servo_on\":"));
    Serial.print(gRt.feederServoOn[MOTION_SIDE_R] ? "true" : "false");
    Serial.print(F(",\"feeder_l_servo_on\":"));
    Serial.print(gRt.feederServoOn[MOTION_SIDE_L] ? "true" : "false");
    Serial.println(F("}"));
    return;
  }
  if (strstr(line, "\"op\":\"feeder_servo_off\"")) {
    bool ok = false;
    if (strstr(line, "\"side\":\"L\"")) ok = feederServoOff(MOTION_SIDE_L);
    else if (strstr(line, "\"side\":\"R\"")) ok = feederServoOff(MOTION_SIDE_R);
    else ok = feederServoOffAll();
    Serial.print(F("{\"op\":\"feeder_servo_off\",\"ok\":"));
    Serial.print(ok ? "true" : "false");
    Serial.print(F(",\"feeder_r_servo_on\":"));
    Serial.print(gRt.feederServoOn[MOTION_SIDE_R] ? "true" : "false");
    Serial.print(F(",\"feeder_l_servo_on\":"));
    Serial.print(gRt.feederServoOn[MOTION_SIDE_L] ? "true" : "false");
    Serial.println(F("}"));
    return;
  }
  if (strstr(line, "\"op\":\"feed_stage\"")) {
    FeedStageRequest req = feedStageDefaultRequest();
    if (strstr(line, "\"feed_validate_encoder\"")) {
      req.checkEncoder = jsonReadBool(line, "feed_validate_encoder", req.checkEncoder);
      gTune.validateEncoder = req.checkEncoder;
    }
    if (strstr(line, "\"side\":\"L\"")) req.useR = false;
    else if (strstr(line, "\"side\":\"R\"")) req.useL = false;
    if (strstr(line, "\"mode\":\"clean\"")) {
      req.checkEncoder = false;
      req.ignoreLaser = true;
      req.laserTimeoutMs = 3000u;
    } else if (strstr(line, "\"mode\":\"laser\"")) {
      req.checkEncoder = false;
      req.laserTimeoutMs = 10000u;
    }
    long timeoutMs = jsonReadLong(line, "timeout_ms", (long)req.laserTimeoutMs);
    if (timeoutMs >= 500 && timeoutMs <= 60000) req.laserTimeoutMs = (uint32_t)timeoutMs;
    FeedStageResult r = FeedStage(req);
    printFeedStageResultJson(r);
    return;
  }
  if (strstr(line, "\"op\":\"lmguide_servo_on\"")) {
    bool ok = lmguideServoOn();
    Serial.print(F("{\"op\":\"lmguide_servo_on\",\"ok\":"));
    Serial.print(ok ? "true" : "false");
    Serial.println(F("}"));
    return;
  }
  if (strstr(line, "\"op\":\"lmguide_servo_off\"")) {
    bool ok = lmguideServoOff();
    Serial.print(F("{\"op\":\"lmguide_servo_off\",\"ok\":"));
    Serial.print(ok ? "true" : "false");
    Serial.println(F("}"));
    return;
  }
  if (strstr(line, "\"op\":\"lmguide_home\"")) {
    if (safetyIsActive()) {
      Serial.println(F("{\"op\":\"lmguide_home\",\"ok\":false,\"error\":\"safety\"}"));
      return;
    }
    bool ok = lmguideHome();
    Serial.print(F("{\"op\":\"lmguide_home\",\"ok\":"));
    Serial.print(ok ? "true" : "false");
    Serial.println(F("}"));
    return;
  }
  if (strstr(line, "\"op\":\"lmguide_move\"")) {
    if (safetyIsActive()) {
      Serial.println(F("{\"op\":\"lmguide_move\",\"ok\":false,\"error\":\"safety\"}"));
      return;
    }
    gTune.lmguide.prSpeedRpm =
        lmClampSpeedRpm(jsonReadFloat(line, "lm_pr_speed_rpm", gTune.lmguide.prSpeedRpm));
    const char* target = "home";
    const char* key = strstr(line, "\"target\"");
    if (key) {
      const char* q = strchr(key, ':');
      if (q) {
        q++;
        while (*q == ' ' || *q == '"') q++;
        target = q;
      }
    }
    long customPuu = jsonReadLong(line, "puu", LONG_MIN);
    bool ok = false;
    if (customPuu != LONG_MIN)
      ok = lmguideMovePr1((int32_t)customPuu);
    else
      ok = lmguideMoveNamed(target);
    Serial.print(F("{\"op\":\"lmguide_move\",\"ok\":"));
    Serial.print(ok ? "true" : "false");
    Serial.println(F("}"));
    return;
  }
  if (strstr(line, "\"op\":\"lmguide_jog\"")) {
    if (safetyIsActive()) {
      Serial.println(F("{\"op\":\"lmguide_jog\",\"ok\":false,\"error\":\"safety\"}"));
      return;
    }
    gTune.lmguide.prSpeedRpm =
        lmClampSpeedRpm(jsonReadFloat(line, "lm_pr_speed_rpm", gTune.lmguide.prSpeedRpm));
    bool ok = false;
    if (strstr(line, "\"delta_puu\"")) {
      ok = lmguideJogDelta((int32_t)jsonReadLong(line, "delta_puu", 0));
    } else {
      long dir = jsonReadLong(line, "dir", 1);
      ok = lmguideJog(dir >= 0 ? 1 : -1);
    }
    Serial.print(F("{\"op\":\"lmguide_jog\",\"ok\":"));
    Serial.print(ok ? "true" : "false");
    Serial.println(F("}"));
    return;
  }
  if (strstr(line, "\"op\":\"lmguide_stop\"")) {
    bool ok = lmguideStop();
    Serial.print(F("{\"op\":\"lmguide_stop\",\"ok\":"));
    Serial.print(ok ? "true" : "false");
    Serial.println(F("}"));
    return;
  }

  Serial.println(F("{\"op\":\"error\",\"msg\":\"unknown_command\"}"));
}

static char gSerialLine[512];
static size_t gSerialLineLen = 0;

static void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      char line[sizeof(gSerialLine)];
      memcpy(line, gSerialLine, gSerialLineLen);
      line[gSerialLineLen] = '\0';
      gSerialLineLen = 0;
      handleSerialLine(line);
      continue;
    }
    if (gSerialLineLen + 1 < sizeof(gSerialLine)) {
      gSerialLine[gSerialLineLen++] = c;
    }
  }
}
static void motionYieldMs(uint32_t ms) {
  uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < ms) {
    pollSerial();
    canServiceRxDrain();
    jointPoll(millis());
    delay(1);
  }
}

// ── Arduino ─────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(PARAM_SERIAL_BAUD);

  motionTuneSetDefaults(&gTune);
  dataLoadNvs();
  runtimeInit(&gRt);
  feederBind(&gTune, gRt.feederReady, gRt.feederServoOn, motionYieldMs, pollSerial);
  lmguideBind(&gTune, &gRt, &gLmAbortRequested, motionYieldMs, pollSerial, safetyIsActive);

  {
    EncoderPCNT::Config encCfg;
    encCfg.right = {PARAM_PIN_ENC_A_R, PARAM_PIN_ENC_B_R, PARAM_PIN_ENC_Z_R};
    encCfg.left = {PARAM_PIN_ENC_A_L, PARAM_PIN_ENC_B_L, PARAM_PIN_ENC_Z_L};
    encCfg.inputMode = INPUT_PULLUP;
    encCfg.glitchFilterNs = PARAM_ENC_GLITCH_FILTER_NS;
    if (!gEncoders.begin(encCfg)) {
      Serial.printf("{\"op\":\"boot\",\"mcu\":\"motion\",\"enc_ok\":false,\"err\":%d}\n",
                    (int)gEncoders.lastError());
    }
  }
  initLasersHw();
  modbusBeginHw();

  motionYieldMs(SERVO_CAN_POWERUP_MS);
  (void)feederInitBus();

  Serial.println(F("{\"op\":\"boot\",\"mcu\":\"motion\"}"));
  printStatusJson();
}

void loop() {
  pollSerial();
  canServiceRxDrain();

  uint32_t now = millis();
  jointPoll(now);
  if (now - gLastEncoderPollMs >= PARAM_ENC_POLL_MS) {
    gLastEncoderPollMs = now;
    gEncoders.update();
  }
}
