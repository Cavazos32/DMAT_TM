#pragma once

/*
 * Parameters.h — fuente única de parámetros MCU PLC
 *
 * [A] Fijos de I/O (PlcIo_Uso.h). El sketch incluye este archivo antes
 *     que PlcIo_Uso.h; los #define evitan una segunda definición.
 *
 * Sin JSON: pines, debounce y límites físicos no se editan en ciclo.
 * Las duraciones de cortador llegan en sus órdenes; aquí se fijan
 * únicamente los límites físicos y valores por defecto.
 * Blower Dual: MCU Motion (PARAM_PIN_BLOWER), no este MCU.
 */

#include <stdint.h>

static const uint32_t PARAM_SERIAL_BAUD = 115200;

// ── [A] Entradas (sensor cerrado / falla = LOW) ───────────────────────────

static const uint8_t PARAM_PLC_PIN_IN_CUTTER  = 26;
static const uint8_t PARAM_PLC_PIN_IN_GRIPPER = 27;
static const uint8_t PARAM_PLC_PIN_IN_HOLDER  = 32;
static const uint8_t PARAM_PLC_PIN_IN_ENCODER = 34;

static const uint8_t PARAM_PLC_BIT_GRIPPER = (1 << 0);
static const uint8_t PARAM_PLC_BIT_HOLDER  = (1 << 1);
static const uint8_t PARAM_PLC_BIT_CUTTER  = (1 << 2);
static const uint8_t PARAM_PLC_BIT_ENCODER = (1 << 5);

// ── [A] Salidas (válvulas por nivel, reset por pulso) ─────────────────────

static const uint8_t PARAM_PLC_PIN_OUT_CUTTER_R = 25;
static const uint8_t PARAM_PLC_PIN_OUT_CUTTER_L = 14;
static const uint8_t PARAM_PLC_PIN_OUT_GRIPPER  = 16;
static const uint8_t PARAM_PLC_PIN_OUT_HOLDER   = 17;
static const uint8_t PARAM_PLC_PIN_OUT_ENCODER  = 21;
static const uint8_t PARAM_PLC_PIN_OUT_RESET    = 33;

// ── [A] Temporizaciones internas y sensores ────────────────────────────────

static const uint32_t PARAM_PLC_RESET_PULSE_MS     = 100;
static const uint32_t PARAM_PLC_CUTTER_MAX_MS      = 5000;
static const uint32_t PARAM_PLC_SETTLE_MAX_MS      = 5000;
static const uint32_t PARAM_PLC_SENSOR_DEBOUNCE_MS = 150;
static const uint32_t PARAM_PLC_BOOT_GRACE_MS      = 15000;

// ── [A] Etiquetas lógicas (PlcStates — no son trama serial) ────────────────

static const uint8_t PARAM_PLC_CMD_CUTTER_R       = 0x19;
static const uint8_t PARAM_PLC_CMD_CUTTER_L       = 0x1A;
static const uint8_t PARAM_PLC_CMD_GRIPPER        = 0x1B;
static const uint8_t PARAM_PLC_CMD_HOLDER         = 0x1C;
static const uint8_t PARAM_PLC_CMD_ENCODER_VALVE  = 0x1D;
static const uint8_t PARAM_PLC_CMD_RESET          = 0x1E;
static const uint8_t PARAM_PLC_CMD_HOLDER_ENCODER = 0x2A;
static const uint8_t PARAM_PLC_CMD_CUTTERS        = 0x2B;

static const uint8_t PARAM_PLC_ERR_CUTTER  = 0x1F;
static const uint8_t PARAM_PLC_ERR_GRIPPER = 0x20;
static const uint8_t PARAM_PLC_ERR_HOLDER  = 0x21;
static const uint8_t PARAM_PLC_ERR_ENCODER = 0x22;

// ── Alias que PlcIo_Uso.h respeta con #ifndef ──────────────────────────────

#define PLC_PIN_IN_CUTTER  PARAM_PLC_PIN_IN_CUTTER
#define PLC_PIN_IN_GRIPPER PARAM_PLC_PIN_IN_GRIPPER
#define PLC_PIN_IN_HOLDER  PARAM_PLC_PIN_IN_HOLDER
#define PLC_PIN_IN_ENCODER PARAM_PLC_PIN_IN_ENCODER

#define PLC_PIN_OUT_CUTTER_R PARAM_PLC_PIN_OUT_CUTTER_R
#define PLC_PIN_OUT_CUTTER_L PARAM_PLC_PIN_OUT_CUTTER_L
#define PLC_PIN_OUT_GRIPPER  PARAM_PLC_PIN_OUT_GRIPPER
#define PLC_PIN_OUT_HOLDER   PARAM_PLC_PIN_OUT_HOLDER
#define PLC_PIN_OUT_ENCODER  PARAM_PLC_PIN_OUT_ENCODER
#define PLC_PIN_OUT_RESET    PARAM_PLC_PIN_OUT_RESET

#define PLC_BIT_GRIPPER PARAM_PLC_BIT_GRIPPER
#define PLC_BIT_HOLDER  PARAM_PLC_BIT_HOLDER
#define PLC_BIT_CUTTER  PARAM_PLC_BIT_CUTTER
#define PLC_BIT_ENCODER PARAM_PLC_BIT_ENCODER

#define PLC_RESET_PULSE_MS     PARAM_PLC_RESET_PULSE_MS
#define PLC_CUTTER_MAX_MS      PARAM_PLC_CUTTER_MAX_MS
#define PLC_SETTLE_MAX_MS      PARAM_PLC_SETTLE_MAX_MS
#define PLC_SENSOR_DEBOUNCE_MS PARAM_PLC_SENSOR_DEBOUNCE_MS
#define PLC_BOOT_GRACE_MS      PARAM_PLC_BOOT_GRACE_MS

#define PLC_CMD_CUTTER_R       PARAM_PLC_CMD_CUTTER_R
#define PLC_CMD_CUTTER_L       PARAM_PLC_CMD_CUTTER_L
#define PLC_CMD_GRIPPER        PARAM_PLC_CMD_GRIPPER
#define PLC_CMD_HOLDER         PARAM_PLC_CMD_HOLDER
#define PLC_CMD_ENCODER_VALVE  PARAM_PLC_CMD_ENCODER_VALVE
#define PLC_CMD_RESET          PARAM_PLC_CMD_RESET
#define PLC_CMD_HOLDER_ENCODER PARAM_PLC_CMD_HOLDER_ENCODER
#define PLC_CMD_CUTTERS        PARAM_PLC_CMD_CUTTERS

#define PLC_ERR_CUTTER  PARAM_PLC_ERR_CUTTER
#define PLC_ERR_GRIPPER PARAM_PLC_ERR_GRIPPER
#define PLC_ERR_HOLDER  PARAM_PLC_ERR_HOLDER
#define PLC_ERR_ENCODER PARAM_PLC_ERR_ENCODER
