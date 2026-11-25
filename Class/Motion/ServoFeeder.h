#pragma once

/*
 * ServoFeeder.h — feeders L/R por CAN (CiA402 / SDO / TWAI).
 * Requiere Parameters.h (MotionSide + alias SERVO_CAN_*).
 * Llamar feederBind() antes de usar.
 */

#include "Parameters.h"

#include <Arduino.h>
#include <driver/twai.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#ifndef MOTION_CALLBACK_TYPES
#define MOTION_CALLBACK_TYPES
typedef void (*MotionPollFn)();
typedef void (*MotionYieldFn)(uint32_t ms);
typedef bool (*MotionSafetyFn)();
#endif

#ifndef SERVO_CAN_TX_GPIO
static const int      SERVO_CAN_TX_GPIO      = 4;
static const int      SERVO_CAN_RX_GPIO      = 5;
static const uint32_t SERVO_CAN_BITRATE_KBPS = 500;
#endif

#ifndef SERVO_CAN_NODE_R
static const uint8_t SERVO_CAN_NODE_R = 1;
static const uint8_t SERVO_CAN_NODE_L = 2;
#endif

#ifndef SERVO_CAN_ENC_INC_PER_REV
static const uint32_t SERVO_CAN_ENC_INC_PER_REV = 10000u;  /* 6091:02 shaft */
static const int8_t   SERVO_CAN_CMD_SIGN_R      = 1;
static const int8_t   SERVO_CAN_CMD_SIGN_L      = -1;
static const uint32_t SERVO_CAN_HOME_PP           = 42500u;
static const uint32_t SERVO_CAN_MAX_RPM           = 3000u;
static const uint32_t SERVO_CAN_MAX_PPS           =
    (uint32_t)((uint64_t)SERVO_CAN_MAX_RPM * SERVO_CAN_ENC_INC_PER_REV / 60u);
static const uint8_t  SERVO_CAN_HALT_OPTION_CODE  = 2;
static const uint32_t SERVO_CAN_HALT_DECEL_PP     = 8000000u;
#endif

#ifndef SERVO_CAN_RX_QUEUE_LEN
static const uint8_t  SERVO_CAN_RX_QUEUE_LEN     = 32;
static const uint8_t  SERVO_CAN_TX_QUEUE_LEN     = 16;
static const uint8_t  SERVO_CAN_INIT_MAX_RETRIES = 3;
static const uint32_t SERVO_CAN_INIT_RETRY_MS    = 200;
static const uint32_t SERVO_CAN_FAULT_RESET_MS   = 200;
static const uint32_t SERVO_CAN_NMT_PREOP_MS     = 200;
static const uint32_t SERVO_CAN_NMT_OPERATIONAL_MS = 200;
static const uint32_t SERVO_CAN_CIA402_STEP_MS   = 200;
static const uint32_t SERVO_CAN_CIA402_SHORT_MS  = 50;
static const uint32_t SERVO_CAN_PROFILE_ACCEL_PPS2 = 800000u;
static const uint32_t SERVO_CAN_PROFILE_DECEL_PPS2 = 800000u;
static const int32_t  SERVO_CAN_JOG_STEP_COUNTS  = 5000;
#endif

static const uint32_t SERVO_CAN_NMT_ID      = 0x000u;
static const uint32_t SERVO_CAN_SDO_TX_BASE = 0x600u;
static const uint32_t SERVO_CAN_SDO_RX_BASE = 0x580u;

#define SERVO_CAN_COB_TX(node) (SERVO_CAN_SDO_TX_BASE + (uint32_t)(node))
#define SERVO_CAN_COB_RX(node) (SERVO_CAN_SDO_RX_BASE + (uint32_t)(node))

static const uint16_t SERVO_OD_CONTROL_WORD      = 0x6040u;
static const uint16_t SERVO_OD_STATUS_WORD       = 0x6041u;
static const uint16_t SERVO_OD_MODE_OF_OPERATION = 0x6060u;
static const uint16_t SERVO_OD_POSITION_ACTUAL   = 0x6064u;
static const uint16_t SERVO_OD_TARGET_POSITION   = 0x607Au;
static const uint16_t SERVO_OD_MAX_PROFILE_VEL   = 0x607Fu;
static const uint16_t SERVO_OD_PROFILE_VELOCITY  = 0x6081u;
static const uint16_t SERVO_OD_PROFILE_ACCEL     = 0x6083u;
static const uint16_t SERVO_OD_PROFILE_DECEL     = 0x6084u;
static const uint16_t SERVO_OD_QUICK_STOP_DECEL  = 0x6085u;
static const uint16_t SERVO_OD_HALT_OPTION       = 0x605Du;
static const uint16_t SERVO_OD_TARGET_VELOCITY   = 0x60FFu;
static const uint16_t SERVO_OD_VELOCITY_ACTUAL   = 0x606Cu;

static const int8_t SERVO_MODE_PP = 1;
static const int8_t SERVO_MODE_PV = 3;

static const float    SERVO_FEED_NOMINAL_MM     = 55.0f;
static const uint32_t SERVO_FEED_NOMINAL_STEPS  = 3600u;
static const float    SERVO_FEED_STEPS_PER_MM_S =
    (float)SERVO_FEED_NOMINAL_STEPS / SERVO_FEED_NOMINAL_MM;

static const uint32_t FEEDER_SDO_CONFIRM_MS = 600;

static bool gCanBusReady = false;
static MotionTune* gFeederTune = nullptr;
static bool* gFeederReady = nullptr;
static bool* gFeederServoOn = nullptr;
static MotionYieldFn gFeederYield = nullptr;
static MotionPollFn gFeederPoll = nullptr;

static void feederYield(uint32_t ms) {
  if (gFeederYield) gFeederYield(ms);
  else delay(ms);
}

static void feederPoll() {
  if (gFeederPoll) gFeederPoll();
}

static void feederBind(MotionTune* tune, bool ready[MOTION_SIDE_COUNT],
                       bool servoOn[MOTION_SIDE_COUNT], MotionYieldFn yieldFn,
                       MotionPollFn pollFn) {
  gFeederTune = tune;
  gFeederReady = ready;
  gFeederServoOn = servoOn;
  gFeederYield = yieldFn;
  gFeederPoll = pollFn;
}

static void feederMarkServoOn(MotionSide side, bool on) {
  if (gFeederServoOn) gFeederServoOn[side] = on;
}

static uint8_t feederNode(MotionSide side) {
  return (side == MOTION_SIDE_R) ? SERVO_CAN_NODE_R : SERVO_CAN_NODE_L;
}

static int8_t feederCmdSign(MotionSide side) {
  return (side == MOTION_SIDE_R) ? SERVO_CAN_CMD_SIGN_R : SERVO_CAN_CMD_SIGN_L;
}

static void feederTrace(const char* phase, MotionSide side, const char* err, uint32_t rx_sdo_frames,
                        int last_cmd_byte) {
  char s = (side == MOTION_SIDE_R) ? 'R' : 'L';
  uint8_t node = feederNode(side);
  twai_status_info_t st = {};
  (void)twai_get_status_info(&st);
  if (last_cmd_byte >= 0) {
    Serial.printf(
        "{\"op\":\"feeder_trace\",\"phase\":\"%s\",\"side\":\"%c\",\"node\":%u,\"err\":\"%s\","
        "\"rx_sdo\":%lu,\"last_cmd\":%d,"
        "\"twai\":{\"tx_err\":%lu,\"rx_missed\":%lu,\"arb_lost\":%lu,\"bus_err\":%lu}}\n",
        phase, s, (unsigned)node, err, (unsigned long)rx_sdo_frames, last_cmd_byte,
        (unsigned long)st.tx_error_counter, (unsigned long)st.rx_missed_count,
        (unsigned long)st.arb_lost_count, (unsigned long)st.bus_error_count);
  } else {
    Serial.printf(
        "{\"op\":\"feeder_trace\",\"phase\":\"%s\",\"side\":\"%c\",\"node\":%u,\"err\":\"%s\","
        "\"rx_sdo\":%lu,"
        "\"twai\":{\"tx_err\":%lu,\"rx_missed\":%lu,\"arb_lost\":%lu,\"bus_err\":%lu}}\n",
        phase, s, (unsigned)node, err, (unsigned long)rx_sdo_frames, (unsigned long)st.tx_error_counter,
        (unsigned long)st.rx_missed_count, (unsigned long)st.arb_lost_count,
        (unsigned long)st.bus_error_count);
  }
}

static bool canBusInit() {
  if (gCanBusReady) return true;
  for (uint8_t attempt = 0; attempt < SERVO_CAN_INIT_MAX_RETRIES; attempt++) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)SERVO_CAN_TX_GPIO, (gpio_num_t)SERVO_CAN_RX_GPIO, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = SERVO_CAN_RX_QUEUE_LEN;
    g_config.tx_queue_len = SERVO_CAN_TX_QUEUE_LEN;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
      if (twai_start() == ESP_OK) return true;
      (void)twai_driver_uninstall();
    }
    feederYield(SERVO_CAN_INIT_RETRY_MS);
  }
  return false;
}

static bool ensureCanBus() {
  if (gCanBusReady) return true;
  if (!canBusInit()) return false;
  gCanBusReady = true;
  return true;
}

static bool canRestartBus() {
  (void)twai_stop();
  (void)twai_driver_uninstall();
  gCanBusReady = false;
  return ensureCanBus();
}

static bool canEnsureRunning() {
  if (!ensureCanBus()) return false;

  twai_status_info_t status = {};
  if (twai_get_status_info(&status) != ESP_OK) return canRestartBus();

  if (status.state == TWAI_STATE_BUS_OFF) {
    if (twai_initiate_recovery() != ESP_OK) return canRestartBus();
    if (twai_get_status_info(&status) != ESP_OK) return canRestartBus();
  }

  uint32_t t0 = millis();
  while (status.state == TWAI_STATE_RECOVERING && millis() - t0 < 1000u) {
    delay(10);
    if (twai_get_status_info(&status) != ESP_OK) return canRestartBus();
  }

  if (status.state == TWAI_STATE_STOPPED) {
    if (twai_start() != ESP_OK) return canRestartBus();
  } else if (status.state != TWAI_STATE_RUNNING) {
    return canRestartBus();
  }
  return true;
}

static bool canPrepareForInit() {
  if (!canEnsureRunning()) return false;
  (void)twai_clear_transmit_queue();
  (void)twai_clear_receive_queue();
  return true;
}

static void canServiceRxDrain() {
  twai_message_t rx;
  for (uint8_t n = 0; n < SERVO_CAN_RX_QUEUE_LEN; n++) {
    if (twai_receive(&rx, 0) != ESP_OK) return;
  }
}

static bool canSendRaw(uint32_t cobId, const uint8_t* data, uint8_t len) {
  twai_message_t msg = {};
  msg.identifier = cobId;
  msg.data_length_code = len;
  msg.extd = 0;
  msg.rtr = 0;
  memcpy(msg.data, data, len);
  esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(80));
  if (err == ESP_OK) return true;
  if (!canEnsureRunning()) return false;
  return twai_transmit(&msg, pdMS_TO_TICKS(80)) == ESP_OK;
}

static void canFlushRxNow() {
  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK) {
  }
}
static void canFlushRxQuiet() {
  canFlushRxNow();
  delay(2);
  canFlushRxNow();
}

static bool canSdoDownload(uint8_t node, uint16_t index, uint8_t sub, const uint8_t data[8]) {
  const uint32_t expectedRx = SERVO_CAN_COB_RX(node);
  const TickType_t rxTicks = pdMS_TO_TICKS(5);

  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    canFlushRxQuiet();
    if (!canSendRaw(SERVO_CAN_COB_TX(node), data, 8)) continue;

    uint32_t t0 = millis();
    while (millis() - t0 < FEEDER_SDO_CONFIRM_MS) {
      twai_message_t rx;
      if (twai_receive(&rx, rxTicks) != ESP_OK) continue;
      if ((rx.identifier & 0x7FFu) != expectedRx || rx.data_length_code < 4) continue;
      if (rx.data[1] != (uint8_t)(index & 0xFF) ||
          rx.data[2] != (uint8_t)(index >> 8) || rx.data[3] != sub)
        continue;
      if (rx.data[0] == 0x60) return true;
      if (rx.data[0] == 0x80) break;
    }
  }
  return false;
}

static void canSdoPackU32(uint8_t d[8], uint16_t index, uint8_t sub, uint32_t value) {
  d[0] = 0x23;
  d[1] = (uint8_t)(index & 0xFF);
  d[2] = (uint8_t)(index >> 8);
  d[3] = sub;
  d[4] = (uint8_t)(value & 0xFF);
  d[5] = (uint8_t)((value >> 8) & 0xFF);
  d[6] = (uint8_t)((value >> 16) & 0xFF);
  d[7] = (uint8_t)((value >> 24) & 0xFF);
}

static bool canSdoDownloadU32(uint8_t node, uint16_t index, uint8_t sub, uint32_t value) {
  uint8_t d[8];
  canSdoPackU32(d, index, sub, value);
  return canSdoDownload(node, index, sub, d);
}

/* Slot de confirmación SDO pendiente (distinto por NodeID + índice + subíndice). */
struct CanSdoPending {
  uint8_t node;
  uint16_t index;
  uint8_t sub;
  bool active;
  bool done;
  bool ok;
};

static void canSdoApplyRxToPending(CanSdoPending* slots, uint8_t n, const twai_message_t& rx) {
  if (!slots || n == 0 || rx.data_length_code < 4) return;
  const uint32_t id = rx.identifier & 0x7FFu;
  for (uint8_t i = 0; i < n; i++) {
    if (!slots[i].active || slots[i].done) continue;
    if (id != SERVO_CAN_COB_RX(slots[i].node)) continue;
    if (rx.data[1] != (uint8_t)(slots[i].index & 0xFF) ||
        rx.data[2] != (uint8_t)(slots[i].index >> 8) || rx.data[3] != slots[i].sub)
      continue;
    if (rx.data[0] == 0x60) {
      slots[i].done = true;
      slots[i].ok = true;
    } else if (rx.data[0] == 0x80) {
      slots[i].done = true;
      slots[i].ok = false;
    }
  }
}

/* Espera confirmaciones de varios nodos en un solo lazo RX (sin flush intermedio). */
static bool canSdoAwaitPending(CanSdoPending* slots, uint8_t n, uint32_t timeoutMs) {
  if (!slots || n == 0) return true;
  const TickType_t rxTicks = pdMS_TO_TICKS(5);
  uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < timeoutMs) {
    bool allDone = true;
    for (uint8_t i = 0; i < n; i++) {
      if (slots[i].active && !slots[i].done) {
        allDone = false;
        break;
      }
    }
    if (allDone) break;
    twai_message_t rx;
    if (twai_receive(&rx, rxTicks) != ESP_OK) continue;
    canSdoApplyRxToPending(slots, n, rx);
  }
  bool ok = true;
  for (uint8_t i = 0; i < n; i++) {
    if (slots[i].active && !slots[i].ok) ok = false;
  }
  return ok;
}

/*
 * Descarga U32 a uno o dos nodos del mismo índice/sub.
 * Con ambos activos: TX consecutivos sin esperar entre ellos; luego confirma ambos.
 * No reenvía un nodo ya confirmado en el reintento.
 */
static bool canSdoDownloadU32Pair(uint8_t nodeA, uint32_t valueA, bool useA, uint8_t nodeB,
                                  uint32_t valueB, bool useB, uint16_t index, uint8_t sub,
                                  uint32_t* tTxA, uint32_t* tTxB, uint32_t* tAckA,
                                  uint32_t* tAckB) {
  if (!useA && !useB) return true;
  if (useA && !useB) {
    if (tTxA) *tTxA = millis();
    bool ok = canSdoDownloadU32(nodeA, index, sub, valueA);
    if (tAckA) *tAckA = millis();
    return ok;
  }
  if (!useA && useB) {
    if (tTxB) *tTxB = millis();
    bool ok = canSdoDownloadU32(nodeB, index, sub, valueB);
    if (tAckB) *tAckB = millis();
    return ok;
  }

  CanSdoPending slots[2] = {};
  slots[0] = {nodeA, index, sub, true, false, false};
  slots[1] = {nodeB, index, sub, true, false, false};
  uint8_t dA[8], dB[8];
  canSdoPackU32(dA, index, sub, valueA);
  canSdoPackU32(dB, index, sub, valueB);

  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    canFlushRxQuiet();
    bool anyPending = false;
    if (!slots[0].ok) {
      slots[0].done = false;
      if (canSendRaw(SERVO_CAN_COB_TX(nodeA), dA, 8)) {
        if (tTxA) *tTxA = millis();
        anyPending = true;
      } else {
        slots[0].done = true;
        slots[0].ok = false;
      }
    }
    if (!slots[1].ok) {
      slots[1].done = false;
      if (canSendRaw(SERVO_CAN_COB_TX(nodeB), dB, 8)) {
        if (tTxB) *tTxB = millis();
        anyPending = true;
      } else {
        slots[1].done = true;
        slots[1].ok = false;
      }
    }
    if (anyPending) (void)canSdoAwaitPending(slots, 2, FEEDER_SDO_CONFIRM_MS);
    if (slots[0].ok && tAckA) *tAckA = millis();
    if (slots[1].ok && tAckB) *tAckB = millis();
    if (slots[0].ok && slots[1].ok) return true;
  }
  return false;
}

static bool canSdoDownloadU16(uint8_t node, uint16_t index, uint8_t sub, uint16_t value) {
  uint8_t d[8] = {0x2B,
                  (uint8_t)(index & 0xFF),
                  (uint8_t)(index >> 8),
                  sub,
                  (uint8_t)(value & 0xFF),
                  (uint8_t)((value >> 8) & 0xFF),
                  0,
                  0};
  return canSdoDownload(node, index, sub, d);
}

static bool canSdoDownloadU8(uint8_t node, uint16_t index, uint8_t sub, uint8_t value) {
  uint8_t d[8] = {0x2F, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), sub, value, 0, 0, 0};
  return canSdoDownload(node, index, sub, d);
}

static bool canNmt(uint8_t cmd) {
  uint8_t d[2] = {cmd, 0};
  return canSendRaw(SERVO_CAN_NMT_ID, d, 2);
}

static bool canReadSdoI32(uint8_t node, uint16_t index, uint8_t sub, int32_t* out, uint32_t timeoutMs) {
  uint8_t req[8] = {0x40, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), sub, 0, 0, 0, 0};
  canFlushRxQuiet();
  if (!canSendRaw(SERVO_CAN_COB_TX(node), req, 8)) return false;

  const TickType_t rxTicks = pdMS_TO_TICKS(5);
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    twai_message_t rx;
    if (twai_receive(&rx, rxTicks) != ESP_OK) continue;
    if ((rx.identifier & 0x7FFu) != SERVO_CAN_COB_RX(node) || rx.data_length_code < 8) continue;
    if (rx.data[0] != 0x43) continue;
    if (rx.data[1] != (index & 0xFF) || rx.data[2] != (index >> 8) || rx.data[3] != sub) continue;
    *out = (int32_t)((uint32_t)rx.data[4] | ((uint32_t)rx.data[5] << 8) | ((uint32_t)rx.data[6] << 16) |
                     ((uint32_t)rx.data[7] << 24));
    return true;
  }
  return false;
}

static bool canReadStatusWord(MotionSide side, uint16_t* out, uint32_t timeoutMs) {
  uint8_t node = feederNode(side);
  uint8_t req[8] = {0x40,
                    (uint8_t)(SERVO_OD_STATUS_WORD & 0xFF),
                    (uint8_t)(SERVO_OD_STATUS_WORD >> 8),
                    0,
                    0,
                    0,
                    0,
                    0};
  canFlushRxQuiet();
  if (!canSendRaw(SERVO_CAN_COB_TX(node), req, 8)) return false;

  const uint32_t expectedRx = SERVO_CAN_COB_RX(node);
  const TickType_t rxTicks = pdMS_TO_TICKS(5);
  uint32_t t0 = millis();
  uint32_t rx_sdo_frames = 0;
  int last_cmd = -1;
  while (millis() - t0 < timeoutMs) {
    twai_message_t rx;
    if (twai_receive(&rx, rxTicks) != ESP_OK) continue;
    uint32_t rxId = rx.identifier & 0x7FFu;
    if (rxId != expectedRx || rx.data_length_code < 4) continue;

    if (rx.data[1] != (uint8_t)(SERVO_OD_STATUS_WORD & 0xFF) ||
        rx.data[2] != (uint8_t)(SERVO_OD_STATUS_WORD >> 8) || rx.data[3] != 0) {
      continue;
    }

    rx_sdo_frames++;
    uint8_t cmd = rx.data[0];
    last_cmd = (int)cmd;

    if (cmd == 0x80) {
      feederTrace("status_word_abort", side, "sdo_abort", rx_sdo_frames, last_cmd);
      return false;
    }

    if (cmd == 0x43 && rx.data_length_code >= 8) {
      *out = (uint16_t)((uint16_t)rx.data[4] | ((uint16_t)rx.data[5] << 8));
      return true;
    }
    if (cmd == 0x4B && rx.data_length_code >= 6) {
      *out = (uint16_t)((uint16_t)rx.data[4] | ((uint16_t)rx.data[5] << 8));
      return true;
    }
  }
  feederTrace("status_word_timeout", side, "no_valid_upload", rx_sdo_frames, last_cmd);
  return false;
}

static bool feederWriteProfile(MotionSide side) {
  if (!gFeederTune) return false;
  uint8_t node = feederNode(side);
  return canSdoDownloadU32(node, SERVO_OD_PROFILE_VELOCITY, 0, gFeederTune->feeder[side].profileVelocityPps) &&
         canSdoDownloadU32(node, SERVO_OD_PROFILE_ACCEL, 0, SERVO_CAN_PROFILE_ACCEL_PPS2) &&
         canSdoDownloadU32(node, SERVO_OD_PROFILE_DECEL, 0, SERVO_CAN_PROFILE_DECEL_PPS2);
}

static bool feederInitNode(MotionSide side) {
  if (!ensureCanBus() || !gFeederTune || !gFeederReady) return false;
  gFeederReady[side] = false;
  feederMarkServoOn(side, false);

  uint8_t node = feederNode(side);
  if (!canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x0006)) return false;
  feederYield(SERVO_CAN_CIA402_STEP_MS);
  if (!canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x0007)) return false;
  feederYield(SERVO_CAN_CIA402_STEP_MS);
  if (!canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x000F)) return false;
  feederYield(SERVO_CAN_CIA402_STEP_MS);
  if (!canSdoDownloadU8(node, SERVO_OD_MODE_OF_OPERATION, 0, (uint8_t)SERVO_MODE_PP)) return false;
  feederYield(SERVO_CAN_CIA402_STEP_MS);
  if (!canSdoDownloadU32(node, SERVO_OD_MAX_PROFILE_VEL, 0, SERVO_CAN_MAX_PPS)) return false;
  feederYield(SERVO_CAN_CIA402_SHORT_MS);
  if (!canSdoDownloadU16(node, SERVO_OD_HALT_OPTION, 0, SERVO_CAN_HALT_OPTION_CODE)) return false;
  if (!canSdoDownloadU32(node, SERVO_OD_QUICK_STOP_DECEL, 0, SERVO_CAN_HALT_DECEL_PP)) return false;
  feederYield(100);
  if (!feederWriteProfile(side)) return false;
  feederYield(500);
  if (!canSdoDownloadU32(node, SERVO_OD_TARGET_POSITION, 0,
                         (uint32_t)gFeederTune->feeder[side].targetPositionCounts))
    return false;
  feederYield(SERVO_CAN_CIA402_STEP_MS);
  if (!canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x002F)) return false;
  feederYield(SERVO_CAN_CIA402_STEP_MS);

  uint16_t sw = 0;
  if (!canReadStatusWord(side, &sw, FEEDER_SDO_CONFIRM_MS)) return false;
  if ((sw & 0x0008u) != 0) return false;
  if ((sw & 0x006Fu) != 0x0027u) return false;

  gFeederReady[side] = true;
  feederMarkServoOn(side, true);
  return true;
}

static int feederProbe(MotionSide side) {
  uint16_t sw = 0;
  if (!canReadStatusWord(side, &sw, FEEDER_SDO_CONFIRM_MS)) return 0;
  if ((sw & 0x0008u) != 0) return 2;
  if ((sw & 0x006Fu) == 0x0027u) return 1;
  return 3;
}

/* Servo ya en Operation Enabled (p. ej. tras reinicio del ESP con ejes vivos). */
static bool feederAdoptReady(MotionSide side) {
  if (!gFeederReady) return false;
  gFeederReady[side] = true;
  feederMarkServoOn(side, true);
  return true;
}

static bool feederBringUp(MotionSide side, int probe) {
  if (probe == 1) return feederAdoptReady(side);
  if (probe == 2) {
    if (!canSdoDownloadU16(feederNode(side), SERVO_OD_CONTROL_WORD, 0, 0x0080)) return false;
    feederYield(SERVO_CAN_FAULT_RESET_MS);
  }
  return feederInitNode(side);
}

static void feederPrintInitResult(bool okR, bool okL) {
  Serial.print(F("{\"op\":\"feeder_init\",\"ok\":"));
  Serial.print(okR && okL ? "true" : "false");
  Serial.print(F(",\"ok_r\":"));
  Serial.print(okR ? "true" : "false");
  Serial.print(F(",\"ok_l\":"));
  Serial.print(okL ? "true" : "false");
  Serial.println(F("}"));
}

static bool feederInitBus() {
  if (!gFeederReady) return false;
  bool canOk = canPrepareForInit();
  if (!canOk) {
    Serial.println(F("{\"op\":\"feeder_init\",\"ok\":false,\"error\":\"can_bus\"}"));
    return false;
  }

  int probeR = feederProbe(MOTION_SIDE_R);
  int probeL = feederProbe(MOTION_SIDE_L);

  /* Ambos ya Operation Enabled: adoptar sin rehacer CiA402 ni NMT. */
  if (probeR == 1 && probeL == 1) {
    bool okR = feederAdoptReady(MOTION_SIDE_R);
    bool okL = feederAdoptReady(MOTION_SIDE_L);
    feederPrintInitResult(okR, okL);
    return okR && okL;
  }

  if (probeR != 0 || probeL != 0) {
    if (!canNmt(0x01)) {
      Serial.println(F("{\"op\":\"feeder_init\",\"ok\":false,\"error\":\"nmt_tx\"}"));
      return false;
    }
    feederYield(SERVO_CAN_NMT_OPERATIONAL_MS);
    bool okR = feederBringUp(MOTION_SIDE_R, probeR);
    bool okL = feederBringUp(MOTION_SIDE_L, probeL);
    feederPrintInitResult(okR, okL);
    return okR && okL;
  }

  gFeederReady[MOTION_SIDE_R] = false;
  gFeederReady[MOTION_SIDE_L] = false;
  feederMarkServoOn(MOTION_SIDE_R, false);
  feederMarkServoOn(MOTION_SIDE_L, false);

  bool resetR = canSdoDownloadU16(feederNode(MOTION_SIDE_R), SERVO_OD_CONTROL_WORD, 0, 0x0080);
  bool resetL = canSdoDownloadU16(feederNode(MOTION_SIDE_L), SERVO_OD_CONTROL_WORD, 0, 0x0080);
  feederYield(SERVO_CAN_FAULT_RESET_MS);
  bool nmtPre = canNmt(0x80);
  feederYield(SERVO_CAN_NMT_PREOP_MS);
  bool nmtOp = canNmt(0x01);
  feederYield(SERVO_CAN_NMT_OPERATIONAL_MS);

  if (!resetR || !resetL) {
    Serial.println(F("{\"op\":\"feeder_init\",\"ok\":false,\"error\":\"fault_reset_tx\"}"));
    return false;
  }
  if (!nmtPre || !nmtOp) {
    Serial.println(F("{\"op\":\"feeder_init\",\"ok\":false,\"error\":\"nmt_tx\"}"));
    return false;
  }
  bool okR = feederInitNode(MOTION_SIDE_R);
  bool okL = feederInitNode(MOTION_SIDE_L);
  feederPrintInitResult(okR, okL);
  return okR && okL;
}

static bool feederStop(MotionSide side) {
  uint8_t node = feederNode(side);
  bool ok = canSdoDownloadU32(node, SERVO_OD_TARGET_VELOCITY, 0, 0);
  if (gFeederServoOn && gFeederServoOn[side]) {
    ok = canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x010F) && ok;
    ok = canSdoDownloadU32(node, SERVO_OD_TARGET_VELOCITY, 0, 0) && ok;
  }
  return ok;
}

/* Duro/suelto tras init: solo Control Word, sin reconfigurar modo/perfil. */
static bool feederServoOn(MotionSide side) {
  if (!ensureCanBus() || !gFeederReady || !gFeederReady[side]) return false;
  uint8_t node = feederNode(side);
  if (!canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x0006)) return false;
  feederYield(SERVO_CAN_CIA402_SHORT_MS);
  if (!canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x0007)) return false;
  feederYield(SERVO_CAN_CIA402_SHORT_MS);
  if (!canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x000F)) return false;
  feederYield(SERVO_CAN_CIA402_SHORT_MS);
  feederMarkServoOn(side, true);
  return true;
}

static bool feederServoOff(MotionSide side) {
  if (!ensureCanBus() || !gFeederReady || !gFeederReady[side]) return false;
  uint8_t node = feederNode(side);
  (void)canSdoDownloadU32(node, SERVO_OD_TARGET_VELOCITY, 0, 0);
  if (!canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x0006)) return false;
  feederMarkServoOn(side, false);
  return true;
}

static bool feederServoOnAll() {
  bool okR = feederServoOn(MOTION_SIDE_R);
  bool okL = feederServoOn(MOTION_SIDE_L);
  return okR && okL;
}

static bool feederServoOffAll() {
  bool okR = feederServoOff(MOTION_SIDE_R);
  bool okL = feederServoOff(MOTION_SIDE_L);
  return okR && okL;
}

static bool feederJogStep(MotionSide side, int dirSign) {
  if (!gFeederReady || !gFeederReady[side]) return false;
  uint8_t node = feederNode(side);
  int32_t pos = 0;
  if (!canReadSdoI32(node, SERVO_OD_POSITION_ACTUAL, 0, &pos, 400)) return false;

  int32_t delta = SERVO_CAN_JOG_STEP_COUNTS * (int32_t)dirSign * (int32_t)feederCmdSign(side);
  int32_t target = pos + delta;

  return feederWriteProfile(side) && canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x010F) &&
         canSdoDownloadU32(node, SERVO_OD_TARGET_POSITION, 0, (uint32_t)target) &&
         canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x012F) &&
         canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x013F) &&
         canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x003F);
}

static bool feederPreparePv(MotionSide side) {
  uint8_t node = feederNode(side);
  if (!canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x010F)) return false;
  if (!canSdoDownloadU8(node, SERVO_OD_MODE_OF_OPERATION, 0, (uint8_t)SERVO_MODE_PV)) return false;
  if (!feederWriteProfile(side)) return false;
  if (!canSdoDownloadU32(node, SERVO_OD_TARGET_VELOCITY, 0, 0)) return false;
  return canSdoDownloadU16(node, SERVO_OD_CONTROL_WORD, 0, 0x010F);
}

static bool feederSetPvVelocity(MotionSide side, int32_t velocitySigned) {
  return canSdoDownloadU32(feederNode(side), SERVO_OD_TARGET_VELOCITY, 0, (uint32_t)velocitySigned);
}

/* Consigna 0x60FF en R y/o L. Dual: TX seguido sin esperar SDO entre nodos. */
static bool feederSetPvVelocityPair(bool useR, int32_t velR, bool useL, int32_t velL,
                                    uint32_t* tTxR, uint32_t* tTxL, uint32_t* tAckR,
                                    uint32_t* tAckL) {
  return canSdoDownloadU32Pair(feederNode(MOTION_SIDE_R), (uint32_t)velR, useR,
                               feederNode(MOTION_SIDE_L), (uint32_t)velL, useL,
                               SERVO_OD_TARGET_VELOCITY, 0, tTxR, tTxL, tAckR, tAckL);
}

static bool feederReleasePv(MotionSide side) {
  return canSdoDownloadU16(feederNode(side), SERVO_OD_CONTROL_WORD, 0, 0x000F);
}

/* Lee velocidad real (0x606C). Tras ReleasePv debe ser ≠0 si 60FFh tiene consigna. */
static bool feederReadVelActual(MotionSide side, int32_t* out, uint32_t timeoutMs) {
  return canReadSdoI32(feederNode(side), SERVO_OD_VELOCITY_ACTUAL, 0, out, timeoutMs);
}

/* SetVel + clear Halt. Usado al arrancar y al reintentar si el eje no se movió. */
static bool feederStartPv(MotionSide side, int32_t velocitySigned) {
  return feederSetPvVelocity(side, velocitySigned) && feederReleasePv(side);
}

static bool feederStopAll() {
  bool ok = feederStop(MOTION_SIDE_R);
  ok = feederStop(MOTION_SIDE_L) && ok;
  return ok;
}

/* Consigna firmada desde PPS absolutos (latch del FeedStage / tune). */
static int32_t feederSignedVelocityPps(MotionSide side, uint32_t ppsAbs) {
  int32_t v = (int32_t)ppsAbs;
  if (v < 0) v = -v;
  return v * (int32_t)feederCmdSign(side);
}

static int32_t feederFeedVelocity(MotionSide side) {
  if (!gFeederTune) return 0;
  return feederSignedVelocityPps(side, gFeederTune->feeder[side].profileVelocityPps);
}
