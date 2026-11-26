#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
static bool gFeedTickable = false;   // begin terminó; tick puede avanzar
static bool gFeedAsync = false;      // solape: reply al completar tick (no bloquea LM)
static bool gReleasePartBusy = false;
static bool gInFeedTick = false;     // evita reentrada feedStageTick vía feederPoll
static uint32_t gFeedBeginMs = 0;    // watchdog absoluto del feed (async/solape)
static uint8_t gSerialPollDepth = 0; // pollSerial anidado (SDO/CAN → feederPoll)
static char gDeferredSerialLine[512];
static bool gDeferredSerialPending = false;
static volatile bool gLmAbortRequested = false;
static volatile bool gFeedAbortRequested = false;

enum FeedStageError {
  FEED_STAGE_OK = 0,
  FEED_STAGE_ERR_BUSY,
  FEED_STAGE_ERR_NO_TARGET,
  FEED_STAGE_ERR_FEEDER,
  FEED_STAGE_ERR_LASER_R,
  FEED_STAGE_ERR_LASER_L,
  FEED_STAGE_ERR_SAFETY,
  FEED_STAGE_ERR_STOP,
  FEED_STAGE_ERR_JOINT_R,
  FEED_STAGE_ERR_JOINT_L,
  FEED_STAGE_ERR_JOINT_RL,
  FEED_STAGE_ERR_MATERIAL_R,
  FEED_STAGE_ERR_MATERIAL_L,
  FEED_STAGE_ERR_MATERIAL_RL,
};

enum FeedStageSideWarn : uint8_t {
  FEED_WARN_NONE = 0,
  FEED_WARN_JOINT = 1,
  FEED_WARN_MATERIAL = 2,
};

struct FeedStageSideResult {
  bool laserOn;
  FeedStageSideWarn warn;
};

struct FeedStageResult {
  FeedStageError error;
  FeedStageSideResult side[MOTION_SIDE_COUNT];
};
struct FeedStageRequest {
  bool useR;
  bool useL;
  bool ignoreLaser;
  bool ignoreJoint;     // purga / Probar: no abortar por Joint
  bool ignoreMaterial;  // purga: material puede seguir ausente
  bool waitNewEdge;     // solape tras corte: ON heredado no cuenta
  uint32_t laserTimeoutMs;
};

static FeedStageResult gLastFeedStage = {};

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

/* "cut" → PR1; cualquier otro / ausente → travel (PR2). */
static LmMoveProfile jsonReadLmProfile(const char* json, LmMoveProfile defVal) {
  const char* p = nullptr;
  if (!jsonFindKey(json, "profile", &p)) return defVal;
  while (*p == ' ' || *p == '"') p++;
  if (strncmp(p, "cut", 3) == 0) return LM_PROFILE_CUT;
  if (strncmp(p, "travel", 6) == 0) return LM_PROFILE_TRAVEL;
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

static void pollSerial();
static void motionYieldMs(uint32_t ms);
static void canRxService();
static bool safetyIsActive();

// ── FeedStage (Bloque 1 — alimentación; encoder solo dual-speed T0) ──────────

static void initLasersHw() {
  pinMode(LASER_PIN_R, INPUT_PULLUP);
  pinMode(LASER_PIN_L, INPUT);  // GPIO34: solo entrada, sin pull-up interno
  pinMode(JOINT_PIN_R, INPUT_PULLUP);
  pinMode(JOINT_PIN_L, INPUT_PULLUP);
  pinMode(MATERIAL_PIN_R, INPUT_PULLUP);
  pinMode(MATERIAL_PIN_L, INPUT_PULLUP);
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

static volatile bool gJointOn[MOTION_SIDE_COUNT] = {false, false};
static volatile bool gJointEvent[MOTION_SIDE_COUNT] = {false, false};  // latch OFF→ON
static uint32_t gJointEdgeMs[MOTION_SIDE_COUNT] = {0, 0};

static volatile bool gMaterialOn[MOTION_SIDE_COUNT] = {false, false};  // ON = presente
static volatile bool gMaterialGoneEvent[MOTION_SIDE_COUNT] = {false, false};  // latch ON→OFF
static uint32_t gMaterialEdgeMs[MOTION_SIDE_COUNT] = {0, 0};
static bool gMaterialInited = false;

// Avisos Serial diferidos (la tarea FreeRTOS no escribe Serial).
static volatile bool gNotifyJointOn[MOTION_SIDE_COUNT] = {false, false};
static volatile bool gNotifyMaterialOff[MOTION_SIDE_COUNT] = {false, false};
static TaskHandle_t gSensorTaskHandle = nullptr;

static bool jointRawOn(MotionSide side) {
  int pin = (side == MOTION_SIDE_R) ? JOINT_PIN_R : JOINT_PIN_L;
  int level = digitalRead(pin);
  if (JOINT_ACTIVE_HIGH) return level == HIGH;
  return level == LOW;
}

static bool materialRawOn(MotionSide side) {
  if (!PARAM_MATERIAL_SENSOR_ENABLE) return true;  // sin sensor: siempre presente
  int pin = (side == MOTION_SIDE_R) ? MATERIAL_PIN_R : MATERIAL_PIN_L;
  int level = digitalRead(pin);
  if (MATERIAL_ACTIVE_HIGH) return level == HIGH;
  return level == LOW;
}

static void printSensorEventJson(MotionSide side, const char* kind, bool active) {
  Serial.print(F("{\"op\":\"sensor_event\",\"side\":\""));
  Serial.print(side == MOTION_SIDE_R ? "R" : "L");
  Serial.print(F("\",\"kind\":\""));
  Serial.print(kind);
  Serial.print(F("\",\"active\":"));
  Serial.print(active ? "true" : "false");
  Serial.println(F("}"));
}
static void jointPoll(uint32_t nowMs) {
  for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
    bool raw = jointRawOn((MotionSide)i);
    if (raw == gJointOn[i]) {
      gJointEdgeMs[i] = 0;
      continue;
    }
    uint32_t needMs = raw ? PARAM_JOINT_DEBOUNCE_ON_MS : PARAM_JOINT_DEBOUNCE_OFF_MS;
    if (gJointEdgeMs[i] == 0) {
      gJointEdgeMs[i] = nowMs;
    } else if ((nowMs - gJointEdgeMs[i]) >= needMs) {
      gJointOn[i] = raw;
      gJointEdgeMs[i] = 0;
      if (raw) {
        gJointEvent[i] = true;
        gNotifyJointOn[i] = true;
      } else {
        // Sensor OFF: no conservar latch huérfano (falso joint_rl con LEDs apagados).
        gJointEvent[i] = false;
        gNotifyJointOn[i] = false;
      }
    }
  }

  if (!gMaterialInited) {
    for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
      gMaterialOn[i] = materialRawOn((MotionSide)i);
      gMaterialEdgeMs[i] = 0;
      gMaterialGoneEvent[i] = false;
    }
    gMaterialInited = true;
  }

  if (!PARAM_MATERIAL_SENSOR_ENABLE) {
    gMaterialOn[MOTION_SIDE_R] = true;
    gMaterialOn[MOTION_SIDE_L] = true;
    return;
  }

  for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
    bool raw = materialRawOn((MotionSide)i);
    if (raw == gMaterialOn[i]) {
      gMaterialEdgeMs[i] = 0;
      continue;
    }
    uint32_t needMs =
        raw ? PARAM_MATERIAL_DEBOUNCE_ON_MS : PARAM_MATERIAL_DEBOUNCE_OFF_MS;
    if (gMaterialEdgeMs[i] == 0) {
      gMaterialEdgeMs[i] = nowMs;
    } else if ((nowMs - gMaterialEdgeMs[i]) >= needMs) {
      bool wasOn = gMaterialOn[i];
      gMaterialOn[i] = raw;
      gMaterialEdgeMs[i] = 0;
      if (wasOn && !raw) {
        gMaterialGoneEvent[i] = true;
        gNotifyMaterialOff[i] = true;
      }
    }
  }
}
static void sensorEventsService() {
  for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
    if (gNotifyJointOn[i]) {
      gNotifyJointOn[i] = false;
      printSensorEventJson((MotionSide)i, "joint", true);
    }
    if (gNotifyMaterialOff[i]) {
      gNotifyMaterialOff[i] = false;
      printSensorEventJson((MotionSide)i, "material", false);
    }
  }
}

static void sensorTask(void* /*arg*/) {
  const TickType_t period = pdMS_TO_TICKS(PARAM_SENSOR_TASK_PERIOD_MS);
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    jointPoll(millis());
    vTaskDelayUntil(&last, period);
  }
}

static void sensorTaskStart() {
  if (gSensorTaskHandle) return;
  xTaskCreatePinnedToCore(
      sensorTask,
      "sensors",
      PARAM_SENSOR_TASK_STACK,
      nullptr,
      PARAM_SENSOR_TASK_PRIO,
      &gSensorTaskHandle,
      0);
}

static bool jointIsOn(MotionSide side) {
  return gJointOn[side];
}

static bool materialIsOn(MotionSide side) {
  return gMaterialOn[side];
}

/** True si material está ausente o hay evento de material terminado pendiente. */
static bool materialGoneDetected(MotionSide side) {
  return (!gMaterialOn[side]) || gMaterialGoneEvent[side];
}

static void sensorEventsClear(bool clearR, bool clearL) {
  if (clearR) {
    gJointEvent[MOTION_SIDE_R] = false;
    gMaterialGoneEvent[MOTION_SIDE_R] = false;
    gNotifyJointOn[MOTION_SIDE_R] = false;
    gNotifyMaterialOff[MOTION_SIDE_R] = false;
  }
  if (clearL) {
    gJointEvent[MOTION_SIDE_L] = false;
    gMaterialGoneEvent[MOTION_SIDE_L] = false;
    gNotifyJointOn[MOTION_SIDE_L] = false;
    gNotifyMaterialOff[MOTION_SIDE_L] = false;
  }
}

static void jointClearPulses() {
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
    case FEED_STAGE_ERR_SAFETY:
      return "safety";
    case FEED_STAGE_ERR_STOP:
      return "stopped";
    case FEED_STAGE_ERR_JOINT_R:
      return "joint_r";
    case FEED_STAGE_ERR_JOINT_L:
      return "joint_l";
    case FEED_STAGE_ERR_JOINT_RL:
      return "joint_rl";
    case FEED_STAGE_ERR_MATERIAL_R:
      return "material_r";
    case FEED_STAGE_ERR_MATERIAL_L:
      return "material_l";
    case FEED_STAGE_ERR_MATERIAL_RL:
      return "material_rl";
    default:
      return "unknown";
  }
}

static FeedStageError feedStageWarnToError(FeedStageSideWarn wR, FeedStageSideWarn wL,
                                          bool useR, bool useL) {
  bool r = useR && wR != FEED_WARN_NONE;
  bool l = useL && wL != FEED_WARN_NONE;
  if (r && l) {
    if (wR == FEED_WARN_JOINT || wL == FEED_WARN_JOINT) return FEED_STAGE_ERR_JOINT_RL;
    return FEED_STAGE_ERR_MATERIAL_RL;
  }
  if (r) {
    return (wR == FEED_WARN_JOINT) ? FEED_STAGE_ERR_JOINT_R : FEED_STAGE_ERR_MATERIAL_R;
  }
  if (l) {
    return (wL == FEED_WARN_JOINT) ? FEED_STAGE_ERR_JOINT_L : FEED_STAGE_ERR_MATERIAL_L;
  }
  return FEED_STAGE_OK;
}

/* Estado del FeedStage en curso (bloqueante o solape async con release_part). */
static FeedStageRequest gFeedReq;
static FeedStageResult gFeedOut;
static bool gFeedSideStopped[MOTION_SIDE_COUNT];
static bool gFeedSideValidated[MOTION_SIDE_COUNT];
static bool gFeedRunR = false;
static bool gFeedRunL = false;
/* Alimentación normal: ON al arrancar = ya en posición (no mueve ese lado).
 * waitNewEdge (solape): ON heredado no cuenta; V1 arranca igual; stop solo tras OFF→ON.
 * rawCount: referencia de movimiento V1→V2 (no afectada por setZero de medición). */
static bool gFeedLaserEdgeArmed[MOTION_SIDE_COUNT] = {true, true};
static FeedStageError gFeedFail = FEED_STAGE_OK;
static uint32_t gFeedT0 = 0;  // primer TX V1 (timeout láser / skew)
static uint32_t gFeedSideTxMs[MOTION_SIDE_COUNT] = {0, 0};  // TX V1 por lado
static uint32_t gFeedSideT0[MOTION_SIDE_COUNT] = {0, 0};  // inicio de movimiento real (V1→V2)
static bool gFeedSideStarted[MOTION_SIDE_COUNT] = {false, false};
static bool gFeedV1Confirmed[MOTION_SIDE_COUNT] = {false, false};
static int64_t gFeedMotionRefRaw[MOTION_SIDE_COUNT] = {0, 0};
static bool gFeedStartLPending = false;  // L espera skew tras V1 de R
static uint32_t gFeedStartLAt = 0;
static FeedStageSideWarn gFeedSideWarn[MOTION_SIDE_COUNT] = {
    FEED_WARN_NONE, FEED_WARN_NONE};
/* Latch del perfil PV del feed en curso (set_tune no altera el movimiento actual). */
static bool gFeedDualSpeed = false;
static uint32_t gFeedSpeedSwitchMs[MOTION_SIDE_COUNT] = {0, 0};  // independiente R/L
static uint32_t gFeedVel1Pps[MOTION_SIDE_COUNT] = {0, 0};
static uint32_t gFeedVel2Pps[MOTION_SIDE_COUNT] = {0, 0};
static bool gFeedSpeed2Sent[MOTION_SIDE_COUNT] = {false, false};
/* Diagnóstico compacto por lado (una línea al cerrar). */
struct FeedSideDiag {
  uint32_t tRecv;
  uint32_t tPrepDone;
  bool servoWasOn;
  bool didEnable;
  uint32_t tV1Tx;
  uint32_t tV1Ack;
  int64_t encRef;
  uint32_t tMotion;
  uint32_t tV2Tx;
  uint32_t tLaserOff;
  uint32_t tLaserOn;
  uint32_t tickGapMax;
};
static FeedSideDiag gFeedDiag[MOTION_SIDE_COUNT] = {};
static uint32_t gFeedLastTickMs = 0;
/* Confirmaciones SDO de 0x60FF en curso (no bloquean arranque L ni V2). */
static CanSdoPending gFeedVelSdo[MOTION_SIDE_COUNT] = {};
static uint32_t gFeedVelSdoDeadline[MOTION_SIDE_COUNT] = {0, 0};
static uint8_t gFeedVelSdoAttempts[MOTION_SIDE_COUNT] = {0, 0};
static uint8_t gFeedVelSdoPhase[MOTION_SIDE_COUNT] = {0, 0};  // 1=V1, 2=V2
static int32_t gFeedVelSdoCmd[MOTION_SIDE_COUNT] = {0, 0};
static uint32_t gFeedVelSdoTTx[MOTION_SIDE_COUNT] = {0, 0};

/* Durante feed async no tirar ACK SDO: canServiceRxDrain los descarta y el
 * solape se queda sin V2/timeout (release_part / loop). */
static void canRxService() {
  if (gFeedStageBusy && gFeedTickable) {
    feederVelPendService(gFeedVelSdo, MOTION_SIDE_COUNT);
  } else {
    canServiceRxDrain();
  }
}

static bool feedStageHealthyDone(MotionSide side) {
  if (side == MOTION_SIDE_R && !gFeedReq.useR) return false;
  if (side == MOTION_SIDE_L && !gFeedReq.useL) return false;
  if (gFeedSideWarn[side] != FEED_WARN_NONE) return false;
  return gFeedSideValidated[side] && gFeedOut.side[side].laserOn;
}

static void feedVelSdoClear(MotionSide side) {
  int i = (int)side;
  gFeedVelSdo[i] = {};
  gFeedVelSdoDeadline[i] = 0;
  gFeedVelSdoAttempts[i] = 0;
  gFeedVelSdoPhase[i] = 0;
  gFeedVelSdoCmd[i] = 0;
  gFeedVelSdoTTx[i] = 0;
}

static int64_t feedEncoderRaw(MotionSide side) {
  return (side == MOTION_SIDE_R) ? gEncoders.right().rawCount : gEncoders.left().rawCount;
}

static void feedDiagPrintSide(MotionSide side) {
  int i = (int)side;
  const FeedSideDiag& d = gFeedDiag[i];
  Serial.print(F("{\"op\":\"feed_diag\",\"side\":\""));
  Serial.print(side == MOTION_SIDE_R ? 'R' : 'L');
  Serial.print(F("\",\"servo_was_on\":"));
  Serial.print(d.servoWasOn ? "true" : "false");
  Serial.print(F(",\"did_enable\":"));
  Serial.print(d.didEnable ? "true" : "false");
  Serial.print(F(",\"prep_ms\":"));
  Serial.print(d.tPrepDone && d.tRecv ? (d.tPrepDone - d.tRecv) : 0u);
  Serial.print(F(",\"v1_tx\":"));
  Serial.print(d.tV1Tx);
  Serial.print(F(",\"v1_ack\":"));
  Serial.print(d.tV1Ack);
  Serial.print(F(",\"enc_ref\":"));
  Serial.print((long)d.encRef);
  Serial.print(F(",\"t_motion\":"));
  Serial.print(d.tMotion);
  Serial.print(F(",\"v1_hold_ms\":"));
  Serial.print(d.tMotion && d.tV2Tx ? (d.tV2Tx - d.tMotion) : 0u);
  Serial.print(F(",\"v2_tx\":"));
  Serial.print(d.tV2Tx);
  Serial.print(F(",\"laser_off\":"));
  Serial.print(d.tLaserOff);
  Serial.print(F(",\"laser_on\":"));
  Serial.print(d.tLaserOn);
  Serial.print(F(",\"tick_gap_max\":"));
  Serial.print(d.tickGapMax);
  Serial.println(F("}"));
}

static void feedVelSdoArm(MotionSide side, int32_t cmd, uint8_t phase, uint32_t tTx) {
  int i = (int)side;
  gFeedVelSdo[i] = {feederNode(side), SERVO_OD_TARGET_VELOCITY, 0, true, false, false};
  gFeedVelSdoDeadline[i] = tTx + FEEDER_SDO_CONFIRM_MS;
  gFeedVelSdoAttempts[i] = 1;
  gFeedVelSdoPhase[i] = phase;
  gFeedVelSdoCmd[i] = cmd;
  gFeedVelSdoTTx[i] = tTx;
}

/* Emite 0x60FF y arma confirmación asíncrona. No espera SDO.
 * phase 1 (V1): permite arrancar un lado aún marcado stopped (skew L).
 * phase 2 (V2): no escribe si ese lado ya paró. */
static bool feedVelFire(MotionSide side, int32_t cmd, uint8_t phase, uint32_t* tTxOut) {
  if (phase == 2 && gFeedSideStopped[(int)side]) return false;
  uint32_t tTx = 0;
  if (!feederSetPvVelocityTx(side, cmd, &tTx)) return false;
  feedVelSdoArm(side, cmd, phase, tTx);
  if (tTxOut) *tTxOut = tTx;
  return true;
}

static void feedVelSdoService() {
  feederVelPendService(gFeedVelSdo, MOTION_SIDE_COUNT);
  uint32_t nowMs = millis();
  for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
    if (!gFeedVelSdo[i].active) continue;
    MotionSide side = (MotionSide)i;
    if (gFeedSideStopped[i]) {
      feedVelSdoClear(side);  // no rearmar velocidad tras paro
      continue;
    }
    if (gFeedVelSdo[i].done) {
      bool ok = gFeedVelSdo[i].ok;
      if (ok) {
        if (gFeedVelSdoPhase[i] == 1) {
          gFeedV1Confirmed[i] = true;
          if (!gFeedDiag[i].tV1Ack) gFeedDiag[i].tV1Ack = nowMs;
        }
        gFeedVelSdo[i].active = false;
        continue;
      }
      if (gFeedVelSdoAttempts[i] < 2) {
        uint32_t tTx = 0;
        if (feederSetPvVelocityTx(side, gFeedVelSdoCmd[i], &tTx)) {
          uint8_t ph = gFeedVelSdoPhase[i];
          int32_t cmd = gFeedVelSdoCmd[i];
          feedVelSdoArm(side, cmd, ph, tTx);
          gFeedVelSdoAttempts[i] = 2;
          continue;
        }
      }
      gFeedVelSdo[i].active = false;
      gFeedFail = FEED_STAGE_ERR_FEEDER;
      feederStopAll();
      continue;
    }
    if (nowMs < gFeedVelSdoDeadline[i]) continue;
    if (gFeedVelSdoAttempts[i] < 2) {
      uint32_t tTx = 0;
      if (feederSetPvVelocityTx(side, gFeedVelSdoCmd[i], &tTx)) {
        gFeedVelSdo[i].done = false;
        gFeedVelSdo[i].ok = false;
        gFeedVelSdoDeadline[i] = tTx + FEEDER_SDO_CONFIRM_MS;
        gFeedVelSdoAttempts[i] = 2;
        gFeedVelSdoTTx[i] = tTx;  // reintento: no reinicia gFeedSideT0
        continue;
      }
    }
    gFeedVelSdo[i].active = false;
    gFeedFail = FEED_STAGE_ERR_FEEDER;
    feederStopAll();
  }
}

static FeedStageRequest feedStageDefaultRequest() {
  FeedStageRequest req;
  req.useR = true;
  req.useL = true;
  req.ignoreLaser = false;
  req.ignoreJoint = false;
  req.ignoreMaterial = false;
  req.waitNewEdge = false;
  req.laserTimeoutMs = PARAM_FEED_STAGE_LASER_TIMEOUT_MS;
  return req;
}

static void printFeedStageResultJson(const FeedStageResult& r);

/* Paro por láser (begin o tick). false = fallo de paro (gFeedFail seteado). */
static bool feedStageStopForLaser(MotionSide side) {
  int i = (int)side;
  if (gFeedSideStopped[i]) return true;
  // Recoger confirms pendientes antes del flush de feederStop.
  feederVelPendService(gFeedVelSdo, MOTION_SIDE_COUNT);
  feedVelSdoClear(side);
  gFeedSideStopped[i] = true;
  bool stopOk = feederStop(side);
  stopOk = canSdoDownloadU32(feederNode(side), SERVO_OD_TARGET_VELOCITY, 0, 0) && stopOk;
  if (!stopOk) {
    gFeedFail = FEED_STAGE_ERR_FEEDER;
    return false;
  }
  gFeedOut.side[i].laserOn = true;
  gFeedSideValidated[i] = true;
  if (side == MOTION_SIDE_R) gFeedRunR = false;
  else gFeedRunL = false;
  return true;
}

/** Detiene un lado por Warning y lo marca como cerrado (sin timeout de láser). */
static void feedStageWarnSide(MotionSide side, FeedStageSideWarn warn) {
  int i = (int)side;
  if (gFeedSideWarn[i] != FEED_WARN_NONE) return;
  gFeedSideWarn[i] = warn;
  gFeedOut.side[i].warn = warn;
  feederVelPendService(gFeedVelSdo, MOTION_SIDE_COUNT);
  feedVelSdoClear(side);
  if (side == MOTION_SIDE_L) gFeedStartLPending = false;
  if (!gFeedSideStopped[i]) {
    gFeedSideStopped[i] = true;
    feederStop(side);
    canSdoDownloadU32(feederNode(side), SERVO_OD_TARGET_VELOCITY, 0, 0);
  } else {
    gFeedSideStopped[i] = true;
  }
  gFeedSideValidated[i] = true;
  if (side == MOTION_SIDE_R) gFeedRunR = false;
  else gFeedRunL = false;
}

static void feedStageFinish() {
  gFeedTickable = false;
  gFeedStartLPending = false;
  feedVelSdoClear(MOTION_SIDE_R);
  feedVelSdoClear(MOTION_SIDE_L);
  if (!feederStopAll() && gFeedFail == FEED_STAGE_OK) gFeedFail = FEED_STAGE_ERR_FEEDER;
  if (gFeedFail != FEED_STAGE_OK) gFeedOut.error = gFeedFail;
  gLastFeedStage = gFeedOut;
  gFeedStageBusy = false;
  if (gFeedReq.useR) feedDiagPrintSide(MOTION_SIDE_R);
  if (gFeedReq.useL) feedDiagPrintSide(MOTION_SIDE_L);
}

static bool feedStageStartSideNow(MotionSide side);

/* true = arrancó (tickable). false = error inmediato en gFeedOut. */
static bool feedStageBegin(FeedStageRequest req) {
  gFeedOut = {};
  gFeedAbortRequested = false;
  gLmAbortRequested = false;
  if (gFeedStageBusy) {
    gFeedOut.error = FEED_STAGE_ERR_BUSY;
    return false;
  }
  if ((req.useR && !gRt.feederReady[MOTION_SIDE_R]) ||
      (req.useL && !gRt.feederReady[MOTION_SIDE_L])) {
    gFeedOut.error = FEED_STAGE_ERR_FEEDER;
    return false;
  }
  if (safetyIsActive()) {
    gFeedOut.error = FEED_STAGE_ERR_SAFETY;
    return false;
  }

  gFeedStageBusy = true;
  gFeedTickable = false;
  gFeedReq = req;
  gFeedFail = FEED_STAGE_OK;
  gFeedSideWarn[MOTION_SIDE_R] = FEED_WARN_NONE;
  gFeedSideWarn[MOTION_SIDE_L] = FEED_WARN_NONE;
  gFeedBeginMs = millis();
  gFeedT0 = 0;
  gFeedSideTxMs[MOTION_SIDE_R] = 0;
  gFeedSideTxMs[MOTION_SIDE_L] = 0;
  gFeedSideT0[MOTION_SIDE_R] = 0;
  gFeedSideT0[MOTION_SIDE_L] = 0;
  gFeedSideStarted[MOTION_SIDE_R] = false;
  gFeedSideStarted[MOTION_SIDE_L] = false;
  gFeedV1Confirmed[MOTION_SIDE_R] = false;
  gFeedV1Confirmed[MOTION_SIDE_L] = false;
  gFeedMotionRefRaw[MOTION_SIDE_R] = 0;
  gFeedMotionRefRaw[MOTION_SIDE_L] = 0;
  gFeedStartLPending = false;
  gFeedStartLAt = 0;
  gFeedLastTickMs = 0;
  memset(gFeedDiag, 0, sizeof(gFeedDiag));
  gFeedDiag[MOTION_SIDE_R].tRecv = gFeedBeginMs;
  gFeedDiag[MOTION_SIDE_L].tRecv = gFeedBeginMs;
  feedVelSdoClear(MOTION_SIDE_R);
  feedVelSdoClear(MOTION_SIDE_L);
  gFeedDualSpeed = gTune.dualSpeedEnable;
  for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
    gFeedSpeedSwitchMs[i] = gTune.feeder[i].speedSwitchMs;
    gFeedVel1Pps[i] = gTune.feeder[i].profileVelocityPps;
    gFeedVel2Pps[i] = gTune.feeder[i].secondaryVelocityPps;
    if (gFeedVel2Pps[i] > gFeedVel1Pps[i]) gFeedVel2Pps[i] = gFeedVel1Pps[i];
    // dual OFF: sin cambio V1→V2 (evita arrastre a 5 rpm con latch viejo).
    if (!gFeedDualSpeed) {
      gFeedVel2Pps[i] = gFeedVel1Pps[i];
      gFeedSpeedSwitchMs[i] = 0;
    }
    gFeedSpeed2Sent[i] = false;
  }

  // Alimentación normal: láser ya ON = material en posición. Ese lado no se mueve.
  // Solape (waitNewEdge): ON heredado no detiene; V1 arranca igual; stop tras OFF→ON.
  bool positioned[MOTION_SIDE_COUNT] = {false, false};
  if (!req.ignoreLaser && !req.waitNewEdge) {
    if (req.useR) positioned[MOTION_SIDE_R] = laserIsOn(MOTION_SIDE_R);
    if (req.useL) positioned[MOTION_SIDE_L] = laserIsOn(MOTION_SIDE_L);
  }
  // waitNewEdge + láser ON: no armado (falta OFF). No retrasa V1.
  gFeedLaserEdgeArmed[MOTION_SIDE_R] =
      !req.useR || req.ignoreLaser || !req.waitNewEdge || !laserIsOn(MOTION_SIDE_R);
  gFeedLaserEdgeArmed[MOTION_SIDE_L] =
      !req.useL || req.ignoreLaser || !req.waitNewEdge || !laserIsOn(MOTION_SIDE_L);

  gFeedRunR = req.useR && !positioned[MOTION_SIDE_R];
  gFeedRunL = req.useL && !positioned[MOTION_SIDE_L];
  gFeedSideStopped[MOTION_SIDE_R] = !gFeedRunR;
  gFeedSideStopped[MOTION_SIDE_L] = !gFeedRunL;
  gFeedSideValidated[MOTION_SIDE_R] = !req.useR;
  gFeedSideValidated[MOTION_SIDE_L] = !req.useL;

  // Joint: solo estado físico. Un joint_event viejo con sensor OFF no debe
  // abortar la alimentación (material presente y Joint apagado = operable).
  if (!req.ignoreJoint && req.useR && jointIsOn(MOTION_SIDE_R)) {
    feedStageWarnSide(MOTION_SIDE_R, FEED_WARN_JOINT);
  } else if (!req.ignoreMaterial && req.useR && !materialIsOn(MOTION_SIDE_R)) {
    gMaterialGoneEvent[MOTION_SIDE_R] = true;
    feedStageWarnSide(MOTION_SIDE_R, FEED_WARN_MATERIAL);
  }
  if (!req.ignoreJoint && req.useL && jointIsOn(MOTION_SIDE_L)) {
    feedStageWarnSide(MOTION_SIDE_L, FEED_WARN_JOINT);
  } else if (!req.ignoreMaterial && req.useL && !materialIsOn(MOTION_SIDE_L)) {
    gMaterialGoneEvent[MOTION_SIDE_L] = true;
    feedStageWarnSide(MOTION_SIDE_L, FEED_WARN_MATERIAL);
  }
  for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
    if (!positioned[i] || gFeedSideWarn[i] != FEED_WARN_NONE) continue;
    gFeedOut.side[i].laserOn = true;
    gFeedSideValidated[i] = true;
  }
  if (gFeedFail == FEED_STAGE_OK && gFeedRunR) {
    gFeedDiag[MOTION_SIDE_R].servoWasOn = feederServoIsOn(MOTION_SIDE_R);
    gFeedDiag[MOTION_SIDE_R].didEnable = !gFeedDiag[MOTION_SIDE_R].servoWasOn;
    if (!feederEnsureServoOn(MOTION_SIDE_R)) gFeedFail = FEED_STAGE_ERR_FEEDER;
  }
  if (gFeedFail == FEED_STAGE_OK && gFeedRunL) {
    gFeedDiag[MOTION_SIDE_L].servoWasOn = feederServoIsOn(MOTION_SIDE_L);
    gFeedDiag[MOTION_SIDE_L].didEnable = !gFeedDiag[MOTION_SIDE_L].servoWasOn;
    if (!feederEnsureServoOn(MOTION_SIDE_L)) gFeedFail = FEED_STAGE_ERR_FEEDER;
  }
  if (gFeedFail == FEED_STAGE_OK && gFeedRunR) {
    if (!feederPreparePv(MOTION_SIDE_R)) gFeedFail = FEED_STAGE_ERR_FEEDER;
  }
  if (gFeedFail == FEED_STAGE_OK && gFeedRunL) {
    if (!feederPreparePv(MOTION_SIDE_L)) gFeedFail = FEED_STAGE_ERR_FEEDER;
  }
  gFeedDiag[MOTION_SIDE_R].tPrepDone = millis();
  gFeedDiag[MOTION_SIDE_L].tPrepDone = gFeedDiag[MOTION_SIDE_R].tPrepDone;
  // Liberar Halt; consigna 0 ya escrita en prepare. Luego V1.
  if (gFeedFail == FEED_STAGE_OK && (gFeedRunR || gFeedRunL)) {
    motionYieldMs(SERVO_CAN_CIA402_SHORT_MS);
    if (gLmAbortRequested || gFeedAbortRequested) {
      gFeedFail = FEED_STAGE_ERR_STOP;
    } else {
      if (gFeedRunR && !feederReleasePv(MOTION_SIDE_R)) gFeedFail = FEED_STAGE_ERR_FEEDER;
      if (gFeedFail == FEED_STAGE_OK && gFeedRunL && !feederReleasePv(MOTION_SIDE_L))
        gFeedFail = FEED_STAGE_ERR_FEEDER;
    }
  }
  // Misma lógica normal/paralelo: V1 al arrancar (no creep/V2).
  const bool fireR = gFeedRunR && gFeedSideWarn[MOTION_SIDE_R] == FEED_WARN_NONE;
  const bool fireL = gFeedRunL && gFeedSideWarn[MOTION_SIDE_L] == FEED_WARN_NONE;
  if (gFeedFail == FEED_STAGE_OK && (fireR || fireL)) {
    if (gLmAbortRequested || gFeedAbortRequested) {
      gFeedFail = FEED_STAGE_ERR_STOP;
    } else if (fireR && fireL) {
      if (!feedStageStartSideNow(MOTION_SIDE_R)) {
        gFeedFail = FEED_STAGE_ERR_FEEDER;
        feederStopAll();
      } else {
        gFeedSideStopped[MOTION_SIDE_L] = true;  // L aún no alimenta
        gFeedSideStarted[MOTION_SIDE_L] = false;
        gFeedStartLPending = true;
        gFeedStartLAt = gFeedSideTxMs[MOTION_SIDE_R] + PARAM_FEED_DUAL_START_SKEW_MS;
      }
    } else if (fireR) {
      if (!feedStageStartSideNow(MOTION_SIDE_R)) {
        gFeedFail = FEED_STAGE_ERR_FEEDER;
        feederStopAll();
      }
    } else {
      if (!feedStageStartSideNow(MOTION_SIDE_L)) {
        gFeedFail = FEED_STAGE_ERR_FEEDER;
        feederStopAll();
      }
    }
  }

  if (gFeedFail != FEED_STAGE_OK) {
    feedStageFinish();
    return false;
  }

  // Ambos lados ya cerrados (láser al inicio/probe y/o Warning): cerrar ya.
  if (gFeedSideValidated[MOTION_SIDE_R] && gFeedSideValidated[MOTION_SIDE_L]) {
    bool anyHealthy = feedStageHealthyDone(MOTION_SIDE_R) || feedStageHealthyDone(MOTION_SIDE_L);
    if (!anyHealthy) {
      gFeedFail = feedStageWarnToError(
          gFeedSideWarn[MOTION_SIDE_R], gFeedSideWarn[MOTION_SIDE_L], req.useR, req.useL);
      if (gFeedFail == FEED_STAGE_OK) gFeedFail = FEED_STAGE_ERR_FEEDER;
    } else {
      gFeedOut.error = FEED_STAGE_OK;
    }
    feedStageFinish();
    return false;  // terminó en begin; caller usa gLastFeedStage / gFeedOut
  }

  gFeedTickable = true;
  return true;
}

static bool feedStageStartSideNow(MotionSide side) {
  int i = (int)side;
  if (gFeedSideWarn[i] != FEED_WARN_NONE) return true;
  // Referencia de movimiento justo antes de V1 (rawCount; setZero de medición no la pisa).
  gEncoders.update();
  gFeedMotionRefRaw[i] = feedEncoderRaw(side);
  gFeedDiag[i].encRef = gFeedMotionRefRaw[i];
  gFeedV1Confirmed[i] = false;
  gFeedSideT0[i] = 0;
  gFeedSpeed2Sent[i] = false;

  int32_t cmd = feederSignedVelocityPps(side, gFeedVel1Pps[i]);
  uint32_t tTx = 0;
  gFeedSideStopped[i] = false;
  if (!feedVelFire(side, cmd, 1, &tTx)) {
    gFeedSideStopped[i] = true;
    return false;
  }
  gFeedSideStarted[i] = true;
  gFeedSideTxMs[i] = tTx;
  gFeedDiag[i].tV1Tx = tTx;
  if (!gFeedT0) gFeedT0 = tTx;
  if (side == MOTION_SIDE_L) gFeedStartLPending = false;
  return true;
}

static bool feedStageStartLNow() {
  return feedStageStartSideNow(MOTION_SIDE_L);
}

/* T0 V1→V2: tras V1 confirmado, ≥8 counts vs referencia previa al TX. */
static void feedStageArmMotionT0(uint32_t nowMs) {
  for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
    if (!gFeedSideStarted[i] || gFeedSideStopped[i] || gFeedSideT0[i]) continue;
    if (!gFeedV1Confirmed[i]) continue;
    MotionSide side = (MotionSide)i;
    int64_t raw = feedEncoderRaw(side);
    int64_t d = raw - gFeedMotionRefRaw[i];
    if (d < 0) d = -d;
    if (d < (int64_t)PARAM_FEED_MOTION_START_COUNTS) continue;
    gFeedSideT0[i] = nowMs;
    gFeedDiag[i].tMotion = nowMs;
  }
}

/* Un paso del lazo láser. true = sigue; false = terminó (gLastFeedStage listo). */
static bool feedStageTick() {
  if (!gFeedStageBusy || !gFeedTickable) return false;
  if (gInFeedTick) return true;
  gInFeedTick = true;

  uint32_t nowMs = millis();
  if (gFeedLastTickMs) {
    uint32_t gap = nowMs - gFeedLastTickMs;
    if (gap > gFeedDiag[MOTION_SIDE_R].tickGapMax)
      gFeedDiag[MOTION_SIDE_R].tickGapMax = gap;
    if (gap > gFeedDiag[MOTION_SIDE_L].tickGapMax)
      gFeedDiag[MOTION_SIDE_L].tickGapMax = gap;
  }
  gFeedLastTickMs = nowMs;
  motionPollEncoders();
  feedVelSdoService();

  if (gFeedFail != FEED_STAGE_OK) {
    feedStageFinish();
    gInFeedTick = false;
    return false;
  }

  if (gLmAbortRequested || gFeedAbortRequested) {
    gFeedFail = FEED_STAGE_ERR_STOP;
  } else if (safetyIsActive()) {
    gFeedFail = FEED_STAGE_ERR_SAFETY;
  } else if (!gFeedReq.ignoreJoint && gFeedReq.useR &&
             gFeedSideWarn[MOTION_SIDE_R] == FEED_WARN_NONE &&
             jointIsOn(MOTION_SIDE_R)) {
    feedStageWarnSide(MOTION_SIDE_R, FEED_WARN_JOINT);
  } else if (!gFeedReq.ignoreJoint && gFeedReq.useL &&
             gFeedSideWarn[MOTION_SIDE_L] == FEED_WARN_NONE &&
             jointIsOn(MOTION_SIDE_L)) {
    feedStageWarnSide(MOTION_SIDE_L, FEED_WARN_JOINT);
  } else if (!gFeedReq.ignoreMaterial && gFeedReq.useR &&
             gFeedSideWarn[MOTION_SIDE_R] == FEED_WARN_NONE &&
             materialGoneDetected(MOTION_SIDE_R) && gMaterialGoneEvent[MOTION_SIDE_R]) {
    feedStageWarnSide(MOTION_SIDE_R, FEED_WARN_MATERIAL);
  } else if (!gFeedReq.ignoreMaterial && gFeedReq.useL &&
             gFeedSideWarn[MOTION_SIDE_L] == FEED_WARN_NONE &&
             materialGoneDetected(MOTION_SIDE_L) && gMaterialGoneEvent[MOTION_SIDE_L]) {
    feedStageWarnSide(MOTION_SIDE_L, FEED_WARN_MATERIAL);
  } else if (gFeedFail == FEED_STAGE_OK) {
    // Arranque L programado (skew desde TX de R); no espera confirm SDO de R.
    if (gFeedStartLPending && gFeedRunL && !gFeedSideStarted[MOTION_SIDE_L] &&
        (int32_t)(nowMs - gFeedStartLAt) >= 0) {
      if (!feedStageStartLNow()) {
        gFeedFail = FEED_STAGE_ERR_FEEDER;
        feederStopAll();
      }
    }

    // T0 V1→V2 independiente por lado (ref raw previa a V1 + confirm SDO).
    feedStageArmMotionT0(nowMs);

    // Prioridad: láser/parada → cambio V2 → continuar.
    // clean/expulsar: sin láser; cierra por timeout y marca lados usados.
    if (gFeedFail == FEED_STAGE_OK && gFeedReq.ignoreLaser) {
      uint32_t feedRef = gFeedT0 ? gFeedT0 : gFeedBeginMs;
      if (feedRef && (nowMs - feedRef >= gFeedReq.laserTimeoutMs)) {
        for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
          if ((i == MOTION_SIDE_R && !gFeedReq.useR) ||
              (i == MOTION_SIDE_L && !gFeedReq.useL)) {
            continue;
          }
          gFeedSideValidated[i] = true;
          gFeedSideStopped[i] = true;
        }
        gFeedOut.error = FEED_STAGE_OK;
        feedStageFinish();
        gInFeedTick = false;
        return false;
      }
    } else if (gFeedFail == FEED_STAGE_OK) {
      for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
        if (gFeedSideValidated[i]) continue;
        MotionSide side = (MotionSide)i;

        bool laserOn = laserIsOn(side);
        // Solape: ON heredado no cuenta. OFF arma el flanco. V1 ya corre.
        if (gFeedReq.waitNewEdge && !gFeedLaserEdgeArmed[i]) {
          if (!laserOn) {
            gFeedLaserEdgeArmed[i] = true;
            if (!gFeedDiag[i].tLaserOff) gFeedDiag[i].tLaserOff = nowMs;
          }
          // Timeout también mientras se espera OFF (material atascado).
          uint32_t sideTx = gFeedSideTxMs[i] ? gFeedSideTxMs[i] : gFeedT0;
          if (gFeedSideStarted[i] && sideTx &&
              (nowMs - sideTx >= gFeedReq.laserTimeoutMs)) {
            gFeedFail =
                (side == MOTION_SIDE_R) ? FEED_STAGE_ERR_LASER_R : FEED_STAGE_ERR_LASER_L;
            break;
          }
          continue;
        }
        if (!gFeedSideStopped[i] && gFeedLaserEdgeArmed[i] && laserOn) {
          if (!gFeedDiag[i].tLaserOn) gFeedDiag[i].tLaserOn = nowMs;
          if (!feedStageStopForLaser(side)) break;
        }

        if (!gFeedSideStarted[i]) continue;
        // Timeout láser desde TX de ese lado (no desde T0 de movimiento).
        uint32_t sideTx = gFeedSideTxMs[i] ? gFeedSideTxMs[i] : gFeedT0;
        if (!gFeedSideStopped[i] && sideTx && (nowMs - sideTx >= gFeedReq.laserTimeoutMs)) {
          gFeedFail =
              (side == MOTION_SIDE_R) ? FEED_STAGE_ERR_LASER_R : FEED_STAGE_ERR_LASER_L;
          break;
        }
      }
    }

    // V2 por lado: T0 de movimiento + switch_ms de ese lado (igual normal/paralelo).
    if (gFeedFail == FEED_STAGE_OK && gFeedDualSpeed) {
      for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
        if (gFeedSideStopped[i] || !gFeedSideStarted[i] || gFeedSpeed2Sent[i] || !gFeedSideT0[i])
          continue;
        if (gFeedSpeedSwitchMs[i] == 0 || (nowMs - gFeedSideT0[i] < gFeedSpeedSwitchMs[i]))
          continue;
        MotionSide side = (MotionSide)i;
        if (gFeedVel2Pps[i] == gFeedVel1Pps[i]) {
          gFeedSpeed2Sent[i] = true;
          continue;
        }
        int32_t cmd = feederSignedVelocityPps(side, gFeedVel2Pps[i]);
        uint32_t tTx = 0;
        bool ok = feedVelFire(side, cmd, 2, &tTx);
        gFeedSpeed2Sent[i] = ok;
        if (ok) gFeedDiag[i].tV2Tx = tTx;
        if (!ok) {
          gFeedFail = FEED_STAGE_ERR_FEEDER;
          feederStopAll();
          break;
        }
      }
    }
  }

  if (gFeedFail != FEED_STAGE_OK) {
    feedStageFinish();
    gInFeedTick = false;
    return false;
  }

  if (gFeedSideValidated[MOTION_SIDE_R] && gFeedSideValidated[MOTION_SIDE_L]) {
    bool anyHealthy = feedStageHealthyDone(MOTION_SIDE_R) || feedStageHealthyDone(MOTION_SIDE_L);
    if (!anyHealthy) {
      gFeedFail = feedStageWarnToError(
          gFeedSideWarn[MOTION_SIDE_R], gFeedSideWarn[MOTION_SIDE_L], gFeedReq.useR,
          gFeedReq.useL);
      if (gFeedFail == FEED_STAGE_OK) gFeedFail = FEED_STAGE_ERR_FEEDER;
    } else {
      gFeedOut.error = FEED_STAGE_OK;
    }
    feedStageFinish();
    gInFeedTick = false;
    return false;
  }

  // Watchdog absoluto: cubre lado stopped sin validar / tick irregular en solape.
  {
    uint32_t feedRef = gFeedT0 ? gFeedT0 : gFeedBeginMs;
    if (feedRef && (nowMs - feedRef >= gFeedReq.laserTimeoutMs)) {
      if (gFeedReq.useR && !gFeedSideValidated[MOTION_SIDE_R])
        gFeedFail = FEED_STAGE_ERR_LASER_R;
      else if (gFeedReq.useL && !gFeedSideValidated[MOTION_SIDE_L])
        gFeedFail = FEED_STAGE_ERR_LASER_L;
      else
        gFeedFail = FEED_STAGE_ERR_FEEDER;
      feedStageFinish();
      gInFeedTick = false;
      return false;
    }
  }

  gInFeedTick = false;
  return true;
}

static FeedStageResult FeedStage(FeedStageRequest req) {
  gFeedAsync = false;
  if (!feedStageBegin(req)) return gFeedOut;
  while (feedStageTick()) {
    pollSerial();
    delay(1);
  }
  return gLastFeedStage;
}

/* Avanza feed async del solape sin bloquear release_part / LM. */
static void feedStageServiceAsync() {
  if (!gFeedAsync || !gFeedTickable || !gFeedStageBusy) return;
  if (feedStageTick()) return;
  printFeedStageResultJson(gLastFeedStage);
  gFeedAsync = false;
}

static const char* feedWarnName(FeedStageSideWarn w) {
  if (w == FEED_WARN_JOINT) return "joint";
  if (w == FEED_WARN_MATERIAL) return "material";
  return "";
}

static void printFeedStageResultJson(const FeedStageResult& r) {
  FeedStageSideWarn wR = r.side[MOTION_SIDE_R].warn;
  FeedStageSideWarn wL = r.side[MOTION_SIDE_L].warn;
  Serial.print(F("{\"op\":\"feed_stage\",\"ok\":"));
  Serial.print(r.error == FEED_STAGE_OK ? "true" : "false");
  Serial.print(F(",\"error\":\""));
  Serial.print(feedStageErrorName(r.error));
  Serial.print(F("\",\"warn\":\""));
  if (wR != FEED_WARN_NONE && wL != FEED_WARN_NONE) {
    Serial.print((wR == FEED_WARN_JOINT || wL == FEED_WARN_JOINT) ? "joint_rl" : "material_rl");
  } else if (wR != FEED_WARN_NONE) {
    Serial.print(wR == FEED_WARN_JOINT ? "joint_r" : "material_r");
  } else if (wL != FEED_WARN_NONE) {
    Serial.print(wL == FEED_WARN_JOINT ? "joint_l" : "material_l");
  }
  Serial.print(F("\",\"R\":{\"laser\":"));
  Serial.print(r.side[MOTION_SIDE_R].laserOn ? "true" : "false");
  Serial.print(F(",\"warn\":\""));
  Serial.print(feedWarnName(wR));
  Serial.print(F("\"},\"L\":{\"laser\":"));
  Serial.print(r.side[MOTION_SIDE_L].laserOn ? "true" : "false");
  Serial.print(F(",\"warn\":\""));
  Serial.print(feedWarnName(wL));
  Serial.println(F("\"}}"));
}

// Ack de movimiento con estado útil (evita status aparte tras lmguide_move).
static void printLmguideMoveResult(bool ok, bool setDataApplied) {
  Serial.print(F("{\"op\":\"lmguide_move\",\"ok\":"));
  Serial.print(ok ? "true" : "false");
  Serial.print(F(",\"lm_homed\":"));
  Serial.print(gRt.lmguideHomed ? "true" : "false");
  Serial.print(F(",\"modbus_writes\":"));
  Serial.print((unsigned)lmguideLastModbusWrites());
  Serial.print(F(",\"profiles_applied\":"));
  Serial.print(lmguideProfilesApplied() ? "true" : "false");
  if (setDataApplied) {
    Serial.print(F(",\"set_data\":true"));
  }
  Serial.println(F("}"));
}

// ── Blower Dual (GPIO Motion) ───────────────────────────────────────────────

static bool gBlowerOn = false;
static uint32_t gBlowerT0 = 0;
static uint32_t gBlowerMs = 0;

static void blowerOff() {
  digitalWrite(PARAM_PIN_BLOWER, LOW);
  gBlowerOn = false;
}

static uint32_t blowerOn(uint32_t ms) {
  if (ms < PARAM_BLOWER_MIN_MS) ms = PARAM_BLOWER_MIN_MS;
  if (ms > PARAM_BLOWER_MAX_MS) ms = PARAM_BLOWER_MAX_MS;
  digitalWrite(PARAM_PIN_BLOWER, HIGH);
  gBlowerOn = true;
  gBlowerT0 = millis();
  gBlowerMs = ms;
  return ms;
}

static void serviceBlower(uint32_t now) {
  if (!gBlowerOn) return;
  if ((uint32_t)(now - gBlowerT0) >= gBlowerMs) blowerOff();
}

static void initBlowerHw() {
  pinMode(PARAM_PIN_BLOWER, OUTPUT);
  digitalWrite(PARAM_PIN_BLOWER, LOW);
  gBlowerOn = false;
}

// Home a PUU 0 con blower: delay → ON → OFF solo al llegar a home (o abort).
// blowerMs > 0 = usar blower (compat; la duración por timer no aplica aquí).
// delay 0 = ON inmediato. Si ya está en home, omite blower.
static void blowerHoldOn() {
  digitalWrite(PARAM_PIN_BLOWER, HIGH);
  gBlowerOn = true;
  gBlowerT0 = millis();
  gBlowerMs = 0xFFFFFFFFu;  // sin apagado por tiempo; solo blowerOff()
}

static bool releasePartHomeWithBlower(uint32_t blowerDelayMs, uint32_t blowerMs) {
  gLmAbortRequested = false;
  gRt.lmguideHomed = false;
  const bool useBlower = (blowerMs > 0);

  int32_t startPuu = 0;
  if (modbusReadReg32(ASDA_REG_P5_016, &startPuu) &&
      lmPositionWithinTol(startPuu, 0, PARAM_LM_MOVE_TOLERANCE_MM)) {
    gRt.lmguideHomed = true;
    gRt.lmguideServoOn = true;
    return true;
  }

  gLmModbusWrites = 0;
  if (!lmguideIssueMove(0, LM_PROFILE_TRAVEL)) return false;

  uint32_t t0 = millis();
  bool blowerStarted = false;

  // delay 0: GPIO ON al arrancar el regreso, sin esperar pollSerial/Modbus.
  if (useBlower && blowerDelayMs == 0) {
    blowerHoldOn();
    blowerStarted = true;
  }

  while ((uint32_t)(millis() - t0) < PARAM_LM_MOVE_DONE_TIMEOUT_MS) {
    uint32_t elapsed = (uint32_t)(millis() - t0);

    // delay > 0: ON en cuanto vence; permanece hasta home.
    if (!blowerStarted && useBlower && elapsed >= blowerDelayMs) {
      blowerHoldOn();
      blowerStarted = true;
    }

    pollSerial();
    canRxService();
    sensorEventsService();

    if (gLmAbortRequested || safetyIsActive()) {
      blowerOff();
      (void)lmguideStop();
      return false;
    }

    uint16_t dwo = 0;
    if (!modbusReadReg16(ASDA_REG_P0_046, &dwo)) {
      delay(5);
      continue;
    }

    bool atTarget = false;
    if ((dwo & ASDA_DO_BIT_TPOS) != 0) {
      int32_t pos = 0;
      if (modbusReadReg32(ASDA_REG_P5_016, &pos) &&
          lmPositionWithinTol(pos, 0, PARAM_LM_MOVE_TOLERANCE_MM)) {
        atTarget = true;
      }
    }

    // Llegó a home antes de vencer el delay → omitir blower (rules §4.8).
    if (atTarget && !blowerStarted) {
      gRt.lmguideHomed = true;
      gRt.lmguideServoOn = true;
      return true;
    }

    if (atTarget) {
      gRt.lmguideHomed = true;
      gRt.lmguideServoOn = true;
      blowerOff();
      return true;
    }
    delay(5);
  }
  blowerOff();
  return false;
}

/* PV inverso por tiempo: misma ruta que feed, signo contrario, paro por ms. */
static int32_t feederReverseVelocityFromRpm(MotionSide side, float rpm) {
  if (rpm < 1.0f) rpm = 1.0f;
  if (rpm > (float)PARAM_FEED_MAX_RPM) rpm = (float)PARAM_FEED_MAX_RPM;
  uint32_t pps =
      (uint32_t)((rpm * (float)PARAM_FEED_SHAFT_INC_PER_REV / 60.0f) + 0.5f);
  if (pps > PARAM_FEED_MAX_PPS) pps = PARAM_FEED_MAX_PPS;
  if (pps < 1u) pps = 1u;
  return -((int32_t)pps) * (int32_t)feederCmdSign(side);
}

static bool feederReverseStopSide(MotionSide side) {
  bool ok = feederStop(side);
  ok = canSdoDownloadU32(feederNode(side), SERVO_OD_TARGET_VELOCITY, 0, 0) && ok;
  return ok;
}

static void feederReverseStopRun(bool runR, bool runL) {
  if (runR) feederReverseStopSide(MOTION_SIDE_R);
  if (runL) feederReverseStopSide(MOTION_SIDE_L);
}

static bool feederReverseRun(bool useR, bool useL, float rpm, uint32_t ms) {
  gFeedAbortRequested = false;
  const bool runR = useR && gRt.feederReady[MOTION_SIDE_R];
  const bool runL = useL && gRt.feederReady[MOTION_SIDE_L];
  if (!runR && !runL) return false;

  if (runR && !feederPreparePv(MOTION_SIDE_R)) {
    feederReverseStopRun(runR, runL);
    return false;
  }
  if (runL && !feederPreparePv(MOTION_SIDE_L)) {
    feederReverseStopRun(runR, runL);
    return false;
  }
  motionYieldMs(SERVO_CAN_CIA402_SHORT_MS);
  if (gLmAbortRequested || gFeedAbortRequested || safetyIsActive()) {
    feederReverseStopRun(runR, runL);
    return false;
  }

  if (runR &&
      !feederSetPvVelocity(MOTION_SIDE_R,
                           feederReverseVelocityFromRpm(MOTION_SIDE_R, rpm))) {
    feederReverseStopRun(runR, runL);
    return false;
  }
  if (runL &&
      !feederSetPvVelocity(MOTION_SIDE_L,
                           feederReverseVelocityFromRpm(MOTION_SIDE_L, rpm))) {
    feederReverseStopRun(runR, runL);
    return false;
  }
  if (runR && !feederReleasePv(MOTION_SIDE_R)) {
    feederReverseStopRun(runR, runL);
    return false;
  }
  if (runL && !feederReleasePv(MOTION_SIDE_L)) {
    feederReverseStopRun(runR, runL);
    return false;
  }

  const uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < ms) {
    pollSerial();
    if (gLmAbortRequested || gFeedAbortRequested || safetyIsActive()) {
      feederReverseStopRun(runR, runL);
      return false;
    }
    delay(1);
  }

  bool ok = true;
  if (runR) ok = feederReverseStopSide(MOTION_SIDE_R) && ok;
  if (runL) ok = feederReverseStopSide(MOTION_SIDE_L) && ok;
  return ok;
}

static void printFeederReverseResult(bool ok, bool useR, bool useL, float rpm,
                                     uint32_t ms) {
  Serial.print(F("{\"op\":\"feeder_reverse\",\"ok\":"));
  Serial.print(ok ? "true" : "false");
  Serial.print(F(",\"side\":\""));
  if (useR && useL) Serial.print(F("both"));
  else if (useR) Serial.print(F("R"));
  else Serial.print(F("L"));
  Serial.print(F("\",\"rpm\":"));
  Serial.print(rpm, 1);
  Serial.print(F(",\"ms\":"));
  Serial.print(ms);
  Serial.println(F("}"));
}

/* Hold manual: PV adelante a RPM fijo hasta feeder_run off / feeder_stop. */
static int32_t feederHoldVelocityFromRpm(MotionSide side, float rpm) {
  if (rpm < 1.0f) rpm = 1.0f;
  if (rpm > (float)PARAM_FEED_MAX_RPM) rpm = (float)PARAM_FEED_MAX_RPM;
  uint32_t pps =
      (uint32_t)((rpm * (float)PARAM_FEED_SHAFT_INC_PER_REV / 60.0f) + 0.5f);
  if (pps > PARAM_FEED_MAX_PPS) pps = PARAM_FEED_MAX_PPS;
  if (pps < 1u) pps = 1u;
  return feederSignedVelocityPps(side, pps);
}

static bool feederRunStart(MotionSide side, float rpm) {
  if (!gRt.feederReady[side]) return false;
  gFeedAbortRequested = false;
  if (!feederPreparePv(side)) return false;
  motionYieldMs(SERVO_CAN_CIA402_SHORT_MS);
  if (gLmAbortRequested || gFeedAbortRequested || safetyIsActive()) {
    feederStop(side);
    return false;
  }
  if (!feederStartPv(side, feederHoldVelocityFromRpm(side, rpm))) {
    feederStop(side);
    return false;
  }
  return true;
}

static bool feederRunStop(MotionSide side) {
  return feederStop(side);
}

static void printFeederRunResult(bool ok, MotionSide side, bool on, float rpm) {
  Serial.print(F("{\"op\":\"feeder_run\",\"ok\":"));
  Serial.print(ok ? "true" : "false");
  Serial.print(F(",\"side\":\""));
  Serial.print(side == MOTION_SIDE_L ? F("L") : F("R"));
  Serial.print(F("\",\"on\":"));
  Serial.print(on ? "true" : "false");
  Serial.print(F(",\"rpm\":"));
  Serial.print(rpm, 1);
  Serial.println(F("}"));
}

static void printReleasePartResult(bool ok) {
  Serial.print(F("{\"op\":\"release_part\",\"ok\":"));
  Serial.print(ok ? "true" : "false");
  Serial.print(F(",\"lm_homed\":"));
  Serial.print(gRt.lmguideHomed ? "true" : "false");
  Serial.print(F(",\"blower\":"));
  Serial.print(gBlowerOn ? "true" : "false");
  Serial.println(F("}"));
}

static void applyLmRampFromJson(const char* json) {
  gTune.lmguide.prSpeedRpm =
      lmClampSpeedRpm(jsonReadFloat(json, "lm_pr_speed_rpm", gTune.lmguide.prSpeedRpm));
  gTune.lmguide.accelMs =
      lmClampRampMs(jsonReadLong(json, "lm_accel_ms", (long)gTune.lmguide.accelMs));
  gTune.lmguide.decelMs =
      lmClampRampMs(jsonReadLong(json, "lm_decel_ms", (long)gTune.lmguide.decelMs));
  gTune.lmguide.travelPrSpeedRpm = lmClampSpeedRpm(
      jsonReadFloat(json, "lm_travel_pr_speed_rpm", gTune.lmguide.travelPrSpeedRpm));
  gTune.lmguide.travelAccelMs = lmClampRampMs(
      jsonReadLong(json, "lm_travel_accel_ms", (long)gTune.lmguide.travelAccelMs));
  gTune.lmguide.travelDecelMs = lmClampRampMs(
      jsonReadLong(json, "lm_travel_decel_ms", (long)gTune.lmguide.travelDecelMs));
  // Compat App antigua: lm_pr_* sin lm_travel_* → copiar a travel.
  if (!strstr(json, "lm_travel_pr_speed_rpm") && strstr(json, "lm_pr_speed_rpm")) {
    gTune.lmguide.travelPrSpeedRpm = gTune.lmguide.prSpeedRpm;
    gTune.lmguide.travelAccelMs = gTune.lmguide.accelMs;
    gTune.lmguide.travelDecelMs = gTune.lmguide.decelMs;
  }
}

static void releasePartRun(const char* line) {
  // Viaje (PR2). Con perfiles aplicados no reescribe rampas; el feed del solape
  // ya no pisa P5.020/060 porque cut/travel son paths distintos.
  gReleasePartBusy = true;
  if (!lmguideProfilesApplied()) applyLmRampFromJson(line);

  uint32_t blowerDelayMs =
      (uint32_t)jsonReadLong(line, "blower_delay_ms", (long)PARAM_MS_BLOWER_DELAY);
  uint32_t blowerMs =
      (uint32_t)jsonReadLong(line, "blower_ms", (long)PARAM_BLOWER_DEFAULT_MS);
  uint32_t homeSettleMs =
      (uint32_t)jsonReadLong(line, "home_settle_ms", (long)PARAM_MS_AT_HOME_END);

  long clearPuu = jsonReadLong(line, "clear_puu", LONG_MIN);
  bool doClear = (clearPuu != LONG_MIN);

  if (doClear) {
    if (!lmguideMove((int32_t)clearPuu, LM_PROFILE_TRAVEL)) {
      gReleasePartBusy = false;
      printReleasePartResult(false);
      return;
    }
  }

  if (!releasePartHomeWithBlower(blowerDelayMs, blowerMs)) {
    gReleasePartBusy = false;
    printReleasePartResult(false);
    return;
  }

  if (homeSettleMs > 0) {
    uint32_t tSettle = millis();
    while ((uint32_t)(millis() - tSettle) < homeSettleMs) {
      pollSerial();
      canRxService();
      sensorEventsService();
      serviceBlower(millis());
      delay(1);
    }
  }
  blowerOff();
  gReleasePartBusy = false;
  printReleasePartResult(true);
}

// ── LM Guide (ciclo: model/named; HW en ServoLMGuide.h) ───────────────────

static bool lmguideMoveModel(LmMoveProfile profile) {
  if (!gRt.lmguideHomed) return false;
  if (gData.targetLengthMm <= 0.0f) return false;
  float mm = gData.targetLengthMm + gData.measureCompMm;
  int32_t puu = (int32_t)lroundf(-mm * ASDA_FACTORY_STEPS_PER_MM);
  return lmguideMove(puu, profile);
}

static bool lmguideMoveNamed(const char* name, LmMoveProfile profile) {
  if (!name) return false;
  if (strncmp(name, "model", 5) == 0) return lmguideMoveModel(profile);
  if (strncmp(name, "home", 4) == 0) {
    return lmguideMove(0, LM_PROFILE_TRAVEL);
  }
  if (!gRt.lmguideHomed) return false;
  return lmguideMove(gTune.lmguide.posPuu, profile);
}
static void motionHwInitRun() {
  if (safetyIsActive()) {
    Serial.println(F("{\"op\":\"motion_hw_init\",\"ok\":false,\"error\":\"safety\"}"));
    return;
  }
  Serial.println(F("{\"op\":\"hw_init_stage\",\"stage\":\"feeder_init\"}"));
  bool okFeed = feederInitBus();
  if (!okFeed) {
    Serial.println(F("{\"op\":\"motion_hw_init\",\"ok\":false,\"error\":\"feeder_init\"}"));
    return;
  }
  Serial.println(F("{\"op\":\"hw_init_stage\",\"stage\":\"lmguide_init\"}"));
  if (!lmguideServoOn()) {
    Serial.println(F("{\"op\":\"motion_hw_init\",\"ok\":false,\"error\":\"lmguide_init\"}"));
    return;
  }
  Serial.println(F("{\"op\":\"hw_init_stage\",\"stage\":\"lmguide_home\"}"));
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

  // Clave legada enc_*: sensores por lado (sin telemetría de encoder OM).
  const char* sideTag[] = {"R", "L"};
  for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
    MotionSide side = (MotionSide)i;
    Serial.print(F(",\"enc_"));
    Serial.print(sideTag[i]);
    Serial.print(F("\":{\"laser\":"));
    Serial.print(laserIsOn(side) ? "true" : "false");
    Serial.print(F(",\"joint\":"));
    Serial.print(jointIsOn(side) ? "true" : "false");
    Serial.print(F(",\"joint_event\":"));
    Serial.print(gJointEvent[side] ? "true" : "false");
    Serial.print(F(",\"material\":"));
    Serial.print(materialIsOn(side) ? "true" : "false");
    Serial.print(F(",\"material_event\":"));
    Serial.print(gMaterialGoneEvent[side] ? "true" : "false");
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
  Serial.print(F(",\"blower\":"));
  Serial.print(gBlowerOn ? "true" : "false");
  Serial.print(F(",\"safety\":"));
  Serial.print(safetyIsActive() ? "true" : "false");
  Serial.println(F(",\"mcu\":\"motion\"}"));
}

static void applyTuneFromJson(const char* json) {
  uint32_t prevVelR = gTune.feeder[MOTION_SIDE_R].profileVelocityPps;
  uint32_t prevVelL = gTune.feeder[MOTION_SIDE_L].profileVelocityPps;

  gTune.feeder[MOTION_SIDE_R].profileVelocityPps =
      (uint32_t)jsonReadLong(json, "feed_r_vel_pps", (long)gTune.feeder[MOTION_SIDE_R].profileVelocityPps);
  gTune.feeder[MOTION_SIDE_R].secondaryVelocityPps =
      (uint32_t)jsonReadLong(json, "feed_r_vel2_pps",
                             (long)gTune.feeder[MOTION_SIDE_R].secondaryVelocityPps);
  gTune.feeder[MOTION_SIDE_R].targetPositionCounts =
      (int32_t)jsonReadLong(json, "feed_r_target_counts", gTune.feeder[MOTION_SIDE_R].targetPositionCounts);

  gTune.feeder[MOTION_SIDE_L].profileVelocityPps =
      (uint32_t)jsonReadLong(json, "feed_l_vel_pps", (long)gTune.feeder[MOTION_SIDE_L].profileVelocityPps);
  gTune.feeder[MOTION_SIDE_L].secondaryVelocityPps =
      (uint32_t)jsonReadLong(json, "feed_l_vel2_pps",
                             (long)gTune.feeder[MOTION_SIDE_L].secondaryVelocityPps);
  gTune.feeder[MOTION_SIDE_L].targetPositionCounts =
      (int32_t)jsonReadLong(json, "feed_l_target_counts", gTune.feeder[MOTION_SIDE_L].targetPositionCounts);

  if (gTune.feeder[MOTION_SIDE_R].secondaryVelocityPps > gTune.feeder[MOTION_SIDE_R].profileVelocityPps)
    gTune.feeder[MOTION_SIDE_R].secondaryVelocityPps = gTune.feeder[MOTION_SIDE_R].profileVelocityPps;
  if (gTune.feeder[MOTION_SIDE_L].secondaryVelocityPps > gTune.feeder[MOTION_SIDE_L].profileVelocityPps)
    gTune.feeder[MOTION_SIDE_L].secondaryVelocityPps = gTune.feeder[MOTION_SIDE_L].profileVelocityPps;

  if (gTune.feeder[MOTION_SIDE_R].profileVelocityPps != prevVelR)
    feederInvalidatePvConfig(MOTION_SIDE_R);
  if (gTune.feeder[MOTION_SIDE_L].profileVelocityPps != prevVelL)
    feederInvalidatePvConfig(MOTION_SIDE_L);

  bool touchLm = strstr(json, "lm_pr_speed_rpm") || strstr(json, "lm_accel_ms") ||
                 strstr(json, "lm_decel_ms") || strstr(json, "lm_travel_pr_speed_rpm") ||
                 strstr(json, "lm_travel_accel_ms") || strstr(json, "lm_travel_decel_ms");
  if (touchLm) {
    applyLmRampFromJson(json);
    lmguideInvalidateProfiles();
  }
  gTune.lmguide.posPuu = (int32_t)jsonReadLong(json, "lm_pos_puu", gTune.lmguide.posPuu);
  gTune.dualSpeedEnable = jsonReadBool(json, "feed_dual_speed_enable", gTune.dualSpeedEnable);
  // Preferente por lado; feed_speed_switch_ms legado aplica a ambos si no vienen R/L.
  long swLegacy = jsonReadLong(json, "feed_speed_switch_ms", -1);
  long swR = jsonReadLong(json, "feed_r_speed_switch_ms", -1);
  long swL = jsonReadLong(json, "feed_l_speed_switch_ms", -1);
  if (swR < 0) swR = (swLegacy >= 0) ? swLegacy : (long)gTune.feeder[MOTION_SIDE_R].speedSwitchMs;
  if (swL < 0) swL = (swLegacy >= 0) ? swLegacy : (long)gTune.feeder[MOTION_SIDE_L].speedSwitchMs;
  if (swR < 0) swR = 0;
  if (swL < 0) swL = 0;
  if ((uint32_t)swR > PARAM_FEED_SPEED_SWITCH_MS_MAX) swR = (long)PARAM_FEED_SPEED_SWITCH_MS_MAX;
  if ((uint32_t)swL > PARAM_FEED_SPEED_SWITCH_MS_MAX) swL = (long)PARAM_FEED_SPEED_SWITCH_MS_MAX;
  gTune.feeder[MOTION_SIDE_R].speedSwitchMs = (uint32_t)swR;
  gTune.feeder[MOTION_SIDE_L].speedSwitchMs = (uint32_t)swL;
}

static void applyDataFromJson(const char* json) {
  gData.targetLengthMm = jsonReadFloat(json, "target_length_mm", gData.targetLengthMm);
  // Legado: tolerancia solo referencia visual HMI (±3 mm); no valida encoder.
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
  if (strstr(line, "\"op\":\"sensor_events_clear\"")) {
    bool clearR = true;
    bool clearL = true;
    if (strstr(line, "\"side\":\"R\"")) clearL = false;
    else if (strstr(line, "\"side\":\"L\"")) clearR = false;
    sensorEventsClear(clearR, clearL);
    Serial.print(F("{\"op\":\"sensor_events_clear\",\"ok\":true"));
    Serial.print(F(",\"R\":{\"joint_event\":"));
    Serial.print(gJointEvent[MOTION_SIDE_R] ? "true" : "false");
    Serial.print(F(",\"material_event\":"));
    Serial.print(gMaterialGoneEvent[MOTION_SIDE_R] ? "true" : "false");
    Serial.print(F("},\"L\":{\"joint_event\":"));
    Serial.print(gJointEvent[MOTION_SIDE_L] ? "true" : "false");
    Serial.print(F(",\"material_event\":"));
    Serial.print(gMaterialGoneEvent[MOTION_SIDE_L] ? "true" : "false");
    Serial.println(F("}}"));
    return;
  }
  if (strstr(line, "\"op\":\"set_tune\"")) {
    applyTuneFromJson(line);
    bool lmOk = lmguideApplyProfiles();
    bool feedOk = true;
    if (gRt.feederReady[MOTION_SIDE_R]) {
      if (!feederApplyPvConfig(MOTION_SIDE_R)) feedOk = false;
    } else {
      feederInvalidatePvConfig(MOTION_SIDE_R);
    }
    if (gRt.feederReady[MOTION_SIDE_L]) {
      if (!feederApplyPvConfig(MOTION_SIDE_L)) feedOk = false;
    } else {
      feederInvalidatePvConfig(MOTION_SIDE_L);
    }
    bool applied = lmOk && feedOk;
    Serial.print(F("{\"op\":\"set_tune\",\"ok\":"));
    Serial.print(applied ? "true" : "false");
    Serial.print(F(",\"applied\":"));
    Serial.print(applied ? "true" : "false");
    Serial.print(F(",\"lm_profiles\":"));
    Serial.print(lmOk ? "true" : "false");
    Serial.print(F(",\"feeder_pv\":"));
    Serial.print(feedOk ? "true" : "false");
    Serial.println(F("}"));
    return;
  }
  if (strstr(line, "\"op\":\"set_data\"")) {
    applyDataFromJson(line);
    Serial.println(F("{\"op\":\"set_data\",\"ok\":true}"));
    return;
  }
  if (strstr(line, "\"op\":\"motion_hw_init\"")) {
    motionHwInitRun();
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
  if (strstr(line, "\"op\":\"feeder_run\"")) {
    if (safetyIsActive()) {
      Serial.println(F("{\"op\":\"feeder_run\",\"ok\":false,\"error\":\"safety\"}"));
      return;
    }
    MotionSide side = MOTION_SIDE_R;
    if (strstr(line, "\"side\":\"L\"")) side = MOTION_SIDE_L;
    bool on = jsonReadBool(line, "on", true);
    float rpm = jsonReadFloat(line, "rpm", PARAM_FEED_HOLD_RPM_DEFAULT);
    if (rpm < 1.0f) rpm = 1.0f;
    if (rpm > (float)PARAM_FEED_MAX_RPM) rpm = (float)PARAM_FEED_MAX_RPM;
    bool ok = on ? feederRunStart(side, rpm) : feederRunStop(side);
    printFeederRunResult(ok, side, on, rpm);
    return;
  }
  if (strstr(line, "\"op\":\"feeder_stop\"")) {
    gFeedAbortRequested = true;
    gLmAbortRequested = true;
    blowerOff();
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
    if (strstr(line, "feed_r_vel_pps") || strstr(line, "feed_l_vel_pps") ||
        strstr(line, "feed_r_vel2_pps") || strstr(line, "feed_l_vel2_pps") ||
        strstr(line, "feed_dual_speed_enable") ||
        strstr(line, "feed_speed_switch_ms") || strstr(line, "feed_r_speed_switch_ms") ||
        strstr(line, "feed_l_speed_switch_ms") ||
        strstr(line, "feed_r_target_counts") || strstr(line, "feed_l_target_counts") ||
        strstr(line, "lm_pos_puu")) {
      applyTuneFromJson(line);
    }
    FeedStageRequest req = feedStageDefaultRequest();
    if (strstr(line, "\"side\":\"L\"")) req.useR = false;
    else if (strstr(line, "\"side\":\"R\"")) req.useL = false;
    if (strstr(line, "\"mode\":\"clean\"")) {
      req.ignoreLaser = true;
      req.ignoreJoint = true;      // purga: Joint puede seguir ON
      req.ignoreMaterial = true;   // purga: material puede seguir ausente
      req.laserTimeoutMs = 3000u;
    } else if (strstr(line, "\"mode\":\"laser\"")) {
      req.ignoreJoint = true;
      req.ignoreMaterial = true;
      req.laserTimeoutMs = 10000u;
    }
    if (strstr(line, "\"ignore_joint\":true") || strstr(line, "\"ignore_joint\": true")) {
      req.ignoreJoint = true;
    }
    if (strstr(line, "\"ignore_material\":true") || strstr(line, "\"ignore_material\": true")) {
      req.ignoreMaterial = true;
    }
    // Solape tras cortadores: solo "edge" pide OFF→ON. async no implica edge
    // (hold/clean async no debe heredar waitNewEdge ni pasar el láser).
    if (strstr(line, "\"edge\":true") || strstr(line, "\"edge\": true")) {
      req.waitNewEdge = true;
    }
    long timeoutMs = jsonReadLong(line, "timeout_ms", (long)req.laserTimeoutMs);
    if (timeoutMs >= 500 && timeoutMs <= 60000) req.laserTimeoutMs = (uint32_t)timeoutMs;

    // Solape: arrancar y volver. El reply sale al cerrar el flanco.
    // REVERTIR arranque en descarga: dejar solo gReleasePartBusy (el feed
    // async existía únicamente dentro de release_part, sin "async":true).
    if (gReleasePartBusy || strstr(line, "\"async\":true") || strstr(line, "\"async\": true")) {
      gFeedAsync = true;
      if (!feedStageBegin(req)) {
        printFeedStageResultJson(
            gFeedOut.error != FEED_STAGE_OK ? gFeedOut : gLastFeedStage);
        gFeedAsync = false;
      }
      return;
    }

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
    bool setDataApplied = false;
    if (strstr(line, "target_length_mm") || strstr(line, "tolerance_mm") ||
        strstr(line, "modelo_id") || strstr(line, "measure_comp_mm")) {
      applyDataFromJson(line);
      setDataApplied = true;
    }
    LmMoveProfile profile = jsonReadLmProfile(line, LM_PROFILE_TRAVEL);
    // Compat: rampas embebidas sin perfiles aplicados (App antigua).
    if (!lmguideProfilesApplied() &&
        (strstr(line, "lm_pr_speed_rpm") || strstr(line, "lm_accel_ms") ||
         strstr(line, "lm_travel_pr_speed_rpm"))) {
      applyLmRampFromJson(line);
    }
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
      ok = lmguideMove((int32_t)customPuu, profile);
    else
      ok = lmguideMoveNamed(target, profile);
    printLmguideMoveResult(ok, setDataApplied);
    return;
  }
  if (strstr(line, "\"op\":\"feeder_reverse\"")) {
    if (safetyIsActive()) {
      Serial.println(F("{\"op\":\"feeder_reverse\",\"ok\":false,\"error\":\"safety\"}"));
      return;
    }
    bool useR = true;
    bool useL = true;
    if (strstr(line, "\"side\":\"L\"")) useR = false;
    else if (strstr(line, "\"side\":\"R\"")) useL = false;
    float rpm = jsonReadFloat(line, "rpm", PARAM_FEED_REVERSE_RPM_DEFAULT);
    if (rpm < 1.0f) rpm = 1.0f;
    if (rpm > (float)PARAM_FEED_MAX_RPM) rpm = (float)PARAM_FEED_MAX_RPM;
    long msRaw = jsonReadLong(line, "ms", (long)PARAM_FEED_REVERSE_MS_DEFAULT);
    uint32_t ms = (uint32_t)msRaw;
    if (ms < PARAM_FEED_REVERSE_MS_MIN) ms = PARAM_FEED_REVERSE_MS_MIN;
    if (ms > PARAM_FEED_REVERSE_MS_MAX) ms = PARAM_FEED_REVERSE_MS_MAX;
    bool ok = feederReverseRun(useR, useL, rpm, ms);
    printFeederReverseResult(ok, useR, useL, rpm, ms);
    return;
  }
  if (strstr(line, "\"op\":\"release_part\"")) {
    if (safetyIsActive()) {
      Serial.println(F("{\"op\":\"release_part\",\"ok\":false,\"error\":\"safety\"}"));
      return;
    }
    releasePartRun(line);
    return;
  }
  if (strstr(line, "\"op\":\"blower_on\"")) {
    uint32_t ms = blowerOn((uint32_t)jsonReadLong(line, "ms", (long)PARAM_BLOWER_DEFAULT_MS));
    Serial.print(F("{\"op\":\"blower_on\",\"ok\":true,\"ms\":"));
    Serial.print(ms);
    Serial.print(F(",\"blower\":true"));
    Serial.println(F("}"));
    return;
  }
  if (strstr(line, "\"op\":\"blower_off\"")) {
    blowerOff();
    Serial.println(F("{\"op\":\"blower_off\",\"ok\":true,\"blower\":false}"));
    return;
  }
  if (strstr(line, "\"op\":\"lmguide_jog\"")) {
    if (safetyIsActive()) {
      Serial.println(F("{\"op\":\"lmguide_jog\",\"ok\":false,\"error\":\"safety\"}"));
      return;
    }
    if (!lmguideProfilesApplied() &&
        (strstr(line, "lm_pr_speed_rpm") || strstr(line, "lm_accel_ms"))) {
      applyLmRampFromJson(line);
    }
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
    blowerOff();
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

static void pollSerialDispatchLine(char* line) {
  // REVERTIR: aceptar feed_stage anidado solo con
  // gSerialPollDepth == 2 && gReleasePartBusy (no durante lmguide_move).
  if (gSerialPollDepth > 1 && strstr(line, "\"op\":\"feed_stage\"")) {
    if (!gFeedStageBusy) handleSerialLine(line);
    return;
  }
  if (gSerialPollDepth > 1) {
    if (strstr(line, "\"op\":\"feeder_stop\"") || strstr(line, "\"op\":\"lmguide_stop\"")) {
      gFeedAbortRequested = true;
      gLmAbortRequested = true;
      return;
    }
    // Hold manual: soltar durante prepare no debe dejar el eje en marcha.
    if (strstr(line, "\"op\":\"feeder_run\"") &&
        (strstr(line, "\"on\":false") || strstr(line, "\"on\": false"))) {
      gFeedAbortRequested = true;
      if (!gDeferredSerialPending) {
        strncpy(gDeferredSerialLine, line, sizeof(gDeferredSerialLine) - 1);
        gDeferredSerialLine[sizeof(gDeferredSerialLine) - 1] = '\0';
        gDeferredSerialPending = true;
      }
      return;
    }
    if (gFeedStageBusy && (strstr(line, "\"op\":\"feeder_servo_on\"") ||
                           strstr(line, "\"op\":\"feeder_servo_off\""))) {
      return;
    }
    if (!gDeferredSerialPending) {
      strncpy(gDeferredSerialLine, line, sizeof(gDeferredSerialLine) - 1);
      gDeferredSerialLine[sizeof(gDeferredSerialLine) - 1] = '\0';
      gDeferredSerialPending = true;
    }
    return;
  }
  handleSerialLine(line);
}

static void pollSerial() {
  gSerialPollDepth++;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      char line[sizeof(gSerialLine)];
      memcpy(line, gSerialLine, gSerialLineLen);
      line[gSerialLineLen] = '\0';
      gSerialLineLen = 0;
      pollSerialDispatchLine(line);
      continue;
    }
    if (gSerialLineLen + 1 < sizeof(gSerialLine)) {
      gSerialLine[gSerialLineLen++] = c;
    }
  }

  if (gSerialPollDepth == 1) {
    if (gDeferredSerialPending) {
      if (gFeedStageBusy &&
          (strstr(gDeferredSerialLine, "\"op\":\"feeder_servo_on\"") ||
           strstr(gDeferredSerialLine, "\"op\":\"feeder_servo_off\""))) {
        gDeferredSerialPending = false;
      } else {
        gDeferredSerialPending = false;
        handleSerialLine(gDeferredSerialLine);
      }
    }
  }
  // REVERTIR: en profundidad 2, tick del feed solo si gReleasePartBusy.
  if (!gInFeedTick && (gSerialPollDepth == 1 || gSerialPollDepth == 2)) {
    feedStageServiceAsync();
  }

  gSerialPollDepth--;
}
static void motionYieldMs(uint32_t ms) {
  uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < ms) {
    pollSerial();
    canRxService();
    sensorEventsService();
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
  initBlowerHw();
  modbusBeginHw();
  sensorTaskStart();

  motionYieldMs(SERVO_CAN_POWERUP_MS);
  (void)feederInitBus();

  Serial.println(F("{\"op\":\"boot\",\"mcu\":\"motion\"}"));
  printStatusJson();
}

void loop() {
  pollSerial();
  // Solape: el feed async debe avanzar aunque no haya tráfico serial nuevo.
  if (gFeedAsync && gFeedStageBusy && gFeedTickable && !gInFeedTick) {
    feedStageServiceAsync();
  }
  canRxService();
  sensorEventsService();

  uint32_t now = millis();
  serviceBlower(now);
  if (now - gLastEncoderPollMs >= PARAM_ENC_POLL_MS) {
    gLastEncoderPollMs = now;
    gEncoders.update();
  }
}
