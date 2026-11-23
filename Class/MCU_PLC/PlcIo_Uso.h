#pragma once

/*
 * PlcIo_Uso.h — MCU PLC (ESP32) ↔ sensores + válvulas neumáticas (GPIO)
 *
 * No hay Modbus ni CAN hacia el PLC: entradas y salidas digitales por nivel.
 * Órdenes por Serial. Sin WiFi.
 * Referencia: Parameters.h · PlcStates.h · MCU_PLC.ino
 *
 * Segmentos:
 *   [1] Enlace físico (pines entrada/salida)
 *   [2] Códigos lógicos (bytes PlcStates — solo etiqueta, no trama serial)
 *   [3] Salidas válvulas — nivel sostenido
 *   [4] Entradas sensores — lectura y debounce
 *   [5] Blower y reset
 *   [6] All off
 *   [7] Ejemplos numéricos
 */

#include <stdint.h>

// ═══════════════════════════════════════════════════════════════════════════
// [1] Enlace físico — MCU ↔ I/O
// ═══════════════════════════════════════════════════════════════════════════
//
// Salidas: GPIO HIGH = válvula activa; GPIO LOW = válvula desactivada.
//
// Entradas (sensor cerrado / falla → lectura LOW → bit en máscara):
//
// | Señal    | GPIO | Bit máscara | Byte error (solo reporte lógico) |
// |----------|------|-------------|----------------------------------|
// | Cutter   |  26  | BIT_CUTTER  | 0x1F                             |
// | Gripper  |  27  | BIT_GRIPPER | 0x20                             |
// | Holder   |  32  | BIT_HOLDER  | 0x21                             |
// | Encoder* |  34  | BIT_ENCODER | 0x22                             |
//
// * GPIO 26, 27 y 32: pull-up interno. Activo en LOW.
// * GPIO34: input-only, sin pull-up interno — pull-up externa en PCB.
//
// Salidas (válvula / relé por nivel sostenido):
//
// | Función     | GPIO | Byte cmd (referencia) |
// |-------------|------|------------------------|
// | Cutter R    |  25  | 0x19                   |
// | Cutter L    |  14  | 0x1A                   |
// | Gripper     |  16  | 0x1B                   |
// | Holder      |  17  | 0x1C                   |
// | Encoder tray|  21  | 0x1D                   |
// | Reset PLC   |  33  | 0x1E                   |
// | Blower      |   4  | 0x23 (nivel temporal)  |
//
// Combinados (actualización simultánea en varios GPIO):
//   0x2A — Holder + Encoder (pines 17 y 21)
//   0x2B — Cutter R + L (pines 25 y 14)

// Si Parameters.h ya definió estos nombres, se usan esos (una sola fuente).

#ifndef PLC_PIN_IN_CUTTER
static const uint8_t PLC_PIN_IN_CUTTER  = 26;
static const uint8_t PLC_PIN_IN_GRIPPER = 27;
static const uint8_t PLC_PIN_IN_HOLDER  = 32;
static const uint8_t PLC_PIN_IN_ENCODER = 34;

static const uint8_t PLC_PIN_OUT_CUTTER_R = 25;
static const uint8_t PLC_PIN_OUT_CUTTER_L = 14;
static const uint8_t PLC_PIN_OUT_GRIPPER  = 16;
static const uint8_t PLC_PIN_OUT_HOLDER   = 17;
static const uint8_t PLC_PIN_OUT_ENCODER  = 21;
static const uint8_t PLC_PIN_OUT_RESET    = 33;

static const uint8_t PLC_BIT_GRIPPER = (1 << 0);
static const uint8_t PLC_BIT_HOLDER  = (1 << 1);
static const uint8_t PLC_BIT_CUTTER  = (1 << 2);
static const uint8_t PLC_BIT_ENCODER = (1 << 5);
#endif

// ═══════════════════════════════════════════════════════════════════════════
// [2] Códigos lógicos (PlcStates.h — mapa nombre ↔ byte)
// ═══════════════════════════════════════════════════════════════════════════

#ifndef PLC_CMD_CUTTER_R
static const uint8_t PLC_CMD_CUTTER_R        = 0x19;
static const uint8_t PLC_CMD_CUTTER_L        = 0x1A;
static const uint8_t PLC_CMD_GRIPPER         = 0x1B;
static const uint8_t PLC_CMD_HOLDER          = 0x1C;
static const uint8_t PLC_CMD_ENCODER_VALVE   = 0x1D;
static const uint8_t PLC_CMD_RESET           = 0x1E;
static const uint8_t PLC_CMD_HOLDER_ENCODER  = 0x2A;
static const uint8_t PLC_CMD_CUTTERS         = 0x2B;

static const uint8_t PLC_ERR_CUTTER  = 0x1F;
static const uint8_t PLC_ERR_GRIPPER = 0x20;
static const uint8_t PLC_ERR_HOLDER  = 0x21;
static const uint8_t PLC_ERR_ENCODER = 0x22;
#endif

// ═══════════════════════════════════════════════════════════════════════════
// [3] Válvulas — nivel sostenido (MCU → PLC/driver)
// ═══════════════════════════════════════════════════════════════════════════
//
// VALVE_SET(name, true/false): GPIO HIGH/LOW sostenido.
// El ack de valve incluye el estado GPIO y fallas (mismo patrón que Motion:
// orden → trabajo → confirma al terminar).
//
// settle_ms (opcional): espera N ms tras cambiar el GPIO y luego responde.
// ms (solo cortadores on): HIGH N ms → LOW → responde.
//
// Boot: todas las salidas de válvula quedan en LOW.

// ═══════════════════════════════════════════════════════════════════════════
// [4] Sensores — lectura (drive → MCU)
// ═══════════════════════════════════════════════════════════════════════════

#ifndef PLC_SENSOR_DEBOUNCE_MS
static const unsigned long PLC_SENSOR_DEBOUNCE_MS = 150;
static const unsigned long PLC_BOOT_GRACE_MS      = 15000;
#endif

// READ_SENSORS_RAW():
//   mask = 0;
//   if (digitalRead(PIN_CUTTER)  == LOW) mask |= PLC_BIT_CUTTER;
//   if (digitalRead(PIN_GRIPPER) == LOW) mask |= PLC_BIT_GRIPPER;
//   if (digitalRead(PIN_HOLDER)  == LOW) mask |= PLC_BIT_HOLDER;
//   if (digitalRead(PIN_ENCODER)== LOW) mask |= PLC_BIT_ENCODER;
//   return mask;
//
// Debounce: el mask debe ser estable PLC_SENSOR_DEBOUNCE_MS antes de aceptar cambio.
// Tras boot, ignorar fallas encoder en 34 hasta haber visto HIGH al menos una vez
// (encoderFaultArmed en firmware).
// Durante PLC_BOOT_GRACE_MS no emitir errores de sensor por arranque lento.

// ═══════════════════════════════════════════════════════════════════════════
// [5] Reset hardware
// ═══════════════════════════════════════════════════════════════════════════
//
// Blower Dual: MCU Motion (PARAM_PIN_BLOWER), no este MCU.
//
// RESET (0x1E, GPIO 33):
//   Todas las válvulas → LOW; GPIO 33 HIGH durante 100 ms y luego LOW.

// ═══════════════════════════════════════════════════════════════════════════
// [6] All off
// ═══════════════════════════════════════════════════════════════════════════
//
// Todas las salidas de válvula pasan a LOW de inmediato.

// ═══════════════════════════════════════════════════════════════════════════
// [7] Ejemplos numéricos
// ═══════════════════════════════════════════════════════════════════════════
//
// ── Ejemplo A: Gripper ON ──
//   digitalWrite(16, HIGH); reply valve+estado de inmediato.
//
// ── Ejemplo A2: Gripper + holder/encoder en una sola orden ──
//   {"op":"valve","gripper":true,"holder":false,"encoder":false,"settle_ms":100}
//   Aplica los bools presentes y responde una sola vez tras settle_ms.
//
// ── Ejemplo B: Holder+Encoder con settle_ms=100 ──
//   GPIO 17 y 21 HIGH; espera 100 ms; reply valve con holder/encoder true.
//
// ── Ejemplo C: Cutters ms=50 ──
//   GPIO 25/14 HIGH → 50 ms → LOW → reply valve (cortadores false).
//
// ── Ejemplo F: Leer sensor Cutter ──
//   digitalRead(26) == LOW  →  falla cortador activa (bit Cutter en mask)
//   Tras debounce 150 ms estable → asociar a PLC_ERR_CUTTER (0x1F) en lógica HMI.
//
// ── Ejemplo G: Reset ──
//   Salidas 25/14/16/17/21 LOW; GPIO 33 HIGH 100 ms y después LOW.
//
// Los bytes 0x19… son solo IDs para alinear nombres con PlcStates.h.
