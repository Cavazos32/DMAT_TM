#pragma once

/*
 * Parameters.h — fuente única de parámetros MCU Motion
 *
 * [A] Fijos en firmware (rules.md / banco mecánico) — no Motion.json
 * [B] Motion.json + set_tune (RAM en ESP32)
 * [C] Datos de ciclo + set_data / modelo (NVS en ESP32)
 *
 * Claves JSON (data/Motion.json) ↔ tune:
 *   feeder.R/L.profile_velocity_pps, target_position_counts
 *   lmguide.pr_speed_rpm, pos_puu
 *   set_data: target_length_mm, tolerance_mm, modelo_id, measure_comp_mm
 *
 * FeedStage: timeout láser fijo PARAM_FEED_STAGE_LASER_TIMEOUT_MS;
 * velocidad de alimentación = feeder.R/L.profile_velocity_pps (mismo bloque CAN).
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// ── [A] Ciclo automático (rules.md §6.7) ────────────────────────────────────

static const uint32_t PARAM_MS_HOLDER_ENC_STABLE   = 100;
static const uint32_t PARAM_MS_LM_AT_CUT          = 100;
static const uint32_t PARAM_MS_LM_AT_DISCHARGE    = 100;
static const uint32_t PARAM_MS_CUTTER_ON          = 50;
static const uint32_t PARAM_MS_AFTER_CUTTER       = 100;
static const uint32_t PARAM_MS_AFTER_GRIP_RELEASE = 100;
static const uint32_t PARAM_MS_BLOWER_DELAY       = 200;
static const uint32_t PARAM_MS_AT_HOME_END        = 50;

static const float PARAM_DATA_TOLERANCE_MM_DEFAULT = 2.0f;

// ── [A] Blower Dual (GPIO en Motion; pin provisional, cableable) ───────────
static const int      PARAM_PIN_BLOWER       = 27;
static const uint32_t PARAM_BLOWER_MIN_MS    = 200;
static const uint32_t PARAM_BLOWER_MAX_MS    = 300000;
static const uint32_t PARAM_BLOWER_DEFAULT_MS = 2000;

// ── [A] Encoder OM (pines; lógica en Encoder.h / EncoderPCNT) + láser + joint ─

static const uint32_t PARAM_ENC_POLL_MS = 80;
// Filtro de glitch PCNT (ns). 5 µs; tope típico ESP32 ~12.5 µs @ APB 80 MHz.
static const uint32_t PARAM_ENC_GLITCH_FILTER_NS = 5000;

static const int PARAM_PIN_ENC_A_R = 18;
static const int PARAM_PIN_ENC_B_R = 19;
static const int PARAM_PIN_ENC_Z_R = 21;
static const int PARAM_PIN_ENC_A_L = 22;
static const int PARAM_PIN_ENC_B_L = 23;
static const int PARAM_PIN_ENC_Z_L = 25;

static const int   PARAM_PIN_LASER_R        = 32;
static const int   PARAM_PIN_LASER_L        = 34;
static const bool  PARAM_LASER_ACTIVE_HIGH  = false;
static const int   PARAM_PIN_JOINT_R        = 12;
static const int   PARAM_PIN_JOINT_L        = 13;
static const bool  PARAM_JOINT_ACTIVE_HIGH  = false;
// Evita falsos ON cruzados por ruido/acoplamiento entre GPIO 12/13.
static const uint32_t PARAM_JOINT_DEBOUNCE_MS = 25;
static const int   PARAM_PIN_SAFETY         = 26;

// ── [A] FeedStage (solo firmware) ─────────────────────────────────────────

static const uint32_t PARAM_FEED_STAGE_LASER_TIMEOUT_MS = 4000u;
static const bool     PARAM_FEED_VALIDATE_ENCODER       = true;
/* Diagnóstico de parada por láser: asiento de encoder + lectura corta de status. */
static const uint32_t PARAM_LASER_STOP_SETTLE_MS   = 80u;
static const uint32_t PARAM_LASER_STOP_STABLE_MS   = 10u;
static const uint32_t PARAM_LASER_STOP_STATUS_MS   = 50u;

// ── [A] Feeder CAN (CiA402) ────────────────────────────────────────────────

static const int      PARAM_CAN_TX_GPIO           = 4;
static const int      PARAM_CAN_RX_GPIO           = 5;
static const uint32_t PARAM_CAN_BITRATE_KBPS      = 500;
static const uint8_t  PARAM_CAN_NODE_R            = 1;
static const uint8_t  PARAM_CAN_NODE_L            = 2;
static const uint8_t  PARAM_CAN_RX_QUEUE_LEN      = 32;
static const uint8_t  PARAM_CAN_TX_QUEUE_LEN      = 16;
static const uint8_t  PARAM_CAN_INIT_MAX_RETRIES  = 3;
static const uint32_t PARAM_CAN_INIT_RETRY_MS     = 400;
static const uint32_t PARAM_CAN_SERVO_POWERUP_MS  = 1000;
static const uint32_t PARAM_CAN_FAULT_RESET_MS    = 300;
static const uint32_t PARAM_CAN_NMT_PREOP_MS      = 500;
static const uint32_t PARAM_CAN_NMT_OPERATIONAL_MS = 500;
static const uint32_t PARAM_CAN_CIA402_STEP_MS    = 200;
static const uint32_t PARAM_CAN_CIA402_SHORT_MS   = 50;

/* DSY 6091:01=1, 6091:02=10000 → 0x60FF = RPM * 10000 / 60 */
static const uint32_t PARAM_FEED_SHAFT_INC_PER_REV = 10000u;
static const int8_t   PARAM_FEED_CMD_SIGN_R       = 1;
static const int8_t   PARAM_FEED_CMD_SIGN_L       = -1;

static const uint32_t PARAM_FEED_TARGET_COUNTS_DEFAULT = 42500u;
static const uint32_t PARAM_FEED_PROFILE_VEL_DEFAULT   = 42500u;
static const uint32_t PARAM_FEED_PROFILE_ACCEL_PPS2      = 800000u;
static const uint32_t PARAM_FEED_PROFILE_DECEL_PPS2      = 800000u;
static const int32_t  PARAM_FEED_JOG_STEP_COUNTS         = 5000;
static const uint32_t PARAM_FEED_MAX_RPM                 = 3000u;
static const uint8_t  PARAM_FEED_HALT_OPTION_CODE        = 2;
static const uint32_t PARAM_FEED_HALT_DECEL_PP           = 8000000u;

static const uint32_t PARAM_FEED_MAX_PPS =
    (uint32_t)((uint64_t)PARAM_FEED_MAX_RPM * PARAM_FEED_SHAFT_INC_PER_REV / 60u);

// ── [A] LM Guide Modbus (ASDA-B3) ─────────────────────────────────────────

static const uint8_t  PARAM_MODBUS_SLAVE_ID       = 0x01;
static const uint32_t PARAM_MODBUS_BAUD           = 9600;
static const int      PARAM_PIN_MODBUS_RX         = 16;
static const int      PARAM_PIN_MODBUS_TX         = 17;

static const float    PARAM_LM_PR_SPEED_RPM       = 1200.0f;
static const float    PARAM_LM_PR_SPEED_RPM_MIN   = 400.0f;
static const float    PARAM_LM_PR_SPEED_RPM_MAX   = 3000.0f;
static const int32_t  PARAM_LM_JOG_DELTA_PUU      = 2000;
static const uint32_t PARAM_LM_SERVO_ON_SETTLE_MS = 2000;

// Validación LM Guide (ASDA PUU vs objetivo; fábrica ServoLMGuide.h [7])
static const float    PARAM_LM_MOVE_TOLERANCE_MM  = 0.5f;
static const float    PARAM_LM_HOME_TOLERANCE_MM  = 1.0f;
static const float    PARAM_LM_HOME_MAX_TRAVEL_MM = 120.0f;
static const uint32_t PARAM_LM_MOVE_DONE_TIMEOUT_MS = 45000;
// Un solo intento: si Modbus/destino fallan, el error debe salir sin esperar 3×45 s.
static const uint8_t  PARAM_LM_MOVE_MAX_ATTEMPTS  = 1;

static const uint16_t PARAM_LM_HOME_TORQUE_PCT    = 10;
static const uint16_t PARAM_LM_HOME_TIME_MS       = 150;
static const float    PARAM_LM_HOME_SPEED_RPM     = 10.0f;
static const uint16_t PARAM_LM_HOME_METHOD_FWD    = 0x0029;
static const uint16_t PARAM_LM_HOME_METHOD_REV    = 0x002A;
static const uint32_t PARAM_LM_HOME_DONE_TIMEOUT_MS = 30000;

static const uint32_t PARAM_SERIAL_BAUD         = 115200;

// ── [B] Motion.json / set_tune (RAM) ───────────────────────────────────────

enum MotionSide : uint8_t { MOTION_SIDE_R = 0, MOTION_SIDE_L = 1, MOTION_SIDE_COUNT = 2 };

struct MotionFeederSideTune {
  uint32_t profileVelocityPps;
  int32_t targetPositionCounts;
};

struct MotionLMGuideTune {
  float prSpeedRpm;
  int32_t posPuu;
};

struct MotionTune {
  MotionFeederSideTune feeder[MOTION_SIDE_COUNT];
  MotionLMGuideTune lmguide;
  bool validateEncoder;
};

// ── [C] set_data / modelo (NVS) ────────────────────────────────────────────

struct MotionData {
  float targetLengthMm;
  float toleranceMm;
  float measureCompMm;
  char modeloId[24];
};

static inline void motionTuneSetDefaults(MotionTune* tune) {
  if (!tune) return;
  memset(tune, 0, sizeof(MotionTune));
  for (int i = 0; i < MOTION_SIDE_COUNT; i++) {
    tune->feeder[i].profileVelocityPps = PARAM_FEED_PROFILE_VEL_DEFAULT;
    tune->feeder[i].targetPositionCounts = (int32_t)PARAM_FEED_TARGET_COUNTS_DEFAULT;
  }
  tune->lmguide.prSpeedRpm = PARAM_LM_PR_SPEED_RPM;
  tune->lmguide.posPuu = 0;
  tune->validateEncoder = PARAM_FEED_VALIDATE_ENCODER;
}

static inline void motionDataSetDefaults(MotionData* data) {
  if (!data) return;
  memset(data, 0, sizeof(MotionData));
  data->toleranceMm = PARAM_DATA_TOLERANCE_MM_DEFAULT;
}

// ── Alias compatibilidad (ServoFeeder.h / ServoLMGuide.h / GPIO) ────────────

#define LASER_PIN_R       PARAM_PIN_LASER_R
#define LASER_PIN_L       PARAM_PIN_LASER_L
#define LASER_ACTIVE_HIGH PARAM_LASER_ACTIVE_HIGH
#define JOINT_PIN_R       PARAM_PIN_JOINT_R
#define JOINT_PIN_L       PARAM_PIN_JOINT_L
#define JOINT_ACTIVE_HIGH PARAM_JOINT_ACTIVE_HIGH
#define SAFETY_PIN        PARAM_PIN_SAFETY

#define SERVO_CAN_TX_GPIO           PARAM_CAN_TX_GPIO
#define SERVO_CAN_RX_GPIO           PARAM_CAN_RX_GPIO
#define SERVO_CAN_BITRATE_KBPS      PARAM_CAN_BITRATE_KBPS
#define SERVO_CAN_NODE_R            PARAM_CAN_NODE_R
#define SERVO_CAN_NODE_L            PARAM_CAN_NODE_L
#define SERVO_CAN_RX_QUEUE_LEN      PARAM_CAN_RX_QUEUE_LEN
#define SERVO_CAN_TX_QUEUE_LEN      PARAM_CAN_TX_QUEUE_LEN
#define SERVO_CAN_INIT_MAX_RETRIES  PARAM_CAN_INIT_MAX_RETRIES
#define SERVO_CAN_INIT_RETRY_MS     PARAM_CAN_INIT_RETRY_MS
#define SERVO_CAN_POWERUP_MS        PARAM_CAN_SERVO_POWERUP_MS
#define SERVO_CAN_FAULT_RESET_MS    PARAM_CAN_FAULT_RESET_MS
#define SERVO_CAN_NMT_PREOP_MS      PARAM_CAN_NMT_PREOP_MS
#define SERVO_CAN_NMT_OPERATIONAL_MS PARAM_CAN_NMT_OPERATIONAL_MS
#define SERVO_CAN_CIA402_STEP_MS    PARAM_CAN_CIA402_STEP_MS
#define SERVO_CAN_CIA402_SHORT_MS   PARAM_CAN_CIA402_SHORT_MS
#define SERVO_CAN_ENC_INC_PER_REV   PARAM_FEED_SHAFT_INC_PER_REV
#define SERVO_CAN_CMD_SIGN_R        PARAM_FEED_CMD_SIGN_R
#define SERVO_CAN_CMD_SIGN_L        PARAM_FEED_CMD_SIGN_L
#define SERVO_CAN_HOME_PP           PARAM_FEED_TARGET_COUNTS_DEFAULT
#define SERVO_CAN_PROFILE_ACCEL_PPS2 PARAM_FEED_PROFILE_ACCEL_PPS2
#define SERVO_CAN_PROFILE_DECEL_PPS2 PARAM_FEED_PROFILE_DECEL_PPS2
#define SERVO_CAN_JOG_STEP_COUNTS   PARAM_FEED_JOG_STEP_COUNTS
#define SERVO_CAN_MAX_RPM           PARAM_FEED_MAX_RPM
#define SERVO_CAN_MAX_PPS           PARAM_FEED_MAX_PPS
#define SERVO_CAN_HALT_OPTION_CODE  PARAM_FEED_HALT_OPTION_CODE
#define SERVO_CAN_HALT_DECEL_PP     PARAM_FEED_HALT_DECEL_PP

#define MODBUS_ASDA_SLAVE_ID      PARAM_MODBUS_SLAVE_ID
#define MODBUS_ASDA_BAUD          PARAM_MODBUS_BAUD
#define ASDA_PR_SPEED_RPM         PARAM_LM_PR_SPEED_RPM
#define ASDA_JOG_DELTA_PUU        PARAM_LM_JOG_DELTA_PUU
#define ASDA_SERVO_ON_SETTLE_MS   PARAM_LM_SERVO_ON_SETTLE_MS
#define ASDA_HOME_TORQUE_PCT_DEFAULT PARAM_LM_HOME_TORQUE_PCT
#define ASDA_HOME_TIME_MS_DEFAULT    PARAM_LM_HOME_TIME_MS
#define ASDA_HOME_SPEED_RPM_DEFAULT  PARAM_LM_HOME_SPEED_RPM
#define ASDA_HOME_METHOD_FORWARD     PARAM_LM_HOME_METHOD_FWD
#define ASDA_HOME_METHOD_REVERSE     PARAM_LM_HOME_METHOD_REV
