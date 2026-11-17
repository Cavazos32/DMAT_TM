/*
  MotorControl — solo lógica de movimiento (base PreFeeder_TCM)

  Arquitectura:
    MotorA → RMT se aplica desde loop() / tu código en el hilo de Arduino
    MotorB → RMT se aplica solo desde su tarea FreeRTOS (core 1 por defecto)
    Core 0 → WiFi / red (no tocar RMT desde ahí)

  Convención RPM:
    Las APIs usan RPM de SALIDA (después del reductor).
    gearRatio convierte a RPM de eje motor antes de setSpeed().
      Ejemplo: gearRatio=15, motorARun(&a, 130) → eje a 1950 RPM
    +rpm = CW (UI)  →  drive RPM negativo en StepperRMT
    -rpm = CCW
     0   = stop

  Integración mínima:
    1) Rellenar MotorCtrl (name, pinStep, pinDir=-1 si solo PUL, taskCore=1 en B)
       Opcional: gearRatio (1.0 sin reductor; 15.0 = 15:1)
    2) motorABegin(&motorA);
       motorBBegin(&motorB, "motorB_task");
    3) Desde loop / tu lógica:
       motorARun(&motorA, +60);          // A ya (RPM salida)
       motorBRequestRpm(&motorB, +60);   // B encola → tarea
       motorARun(&motorA, 0);
       motorBRequestStop(&motorB);

  Regla: no llamar setSpeed/Stop/Brake del driver de B fuera de su tarea.
*/

#pragma once

#include <Arduino.h>
#include "StepperRMT.h"   // mismo folder del sketch PreFeeder
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef MOTOR_CTRL_MICROSTEP
#define MOTOR_CTRL_MICROSTEP 4
#endif
#ifndef MOTOR_CTRL_ACCEL
#define MOTOR_CTRL_ACCEL 1000.0f
#endif
#ifndef MOTOR_CTRL_DECEL
#define MOTOR_CTRL_DECEL 1000.0f
#endif
#ifndef MOTOR_CTRL_RPM_MIN
#define MOTOR_CTRL_RPM_MIN 1.0f
#endif
#ifndef MOTOR_CTRL_RPM_MAX
#define MOTOR_CTRL_RPM_MAX 3000.0f  // límite del eje motor (tras reductor)
#endif
#ifndef MOTOR_CTRL_GEAR_MAX
#define MOTOR_CTRL_GEAR_MAX 100.0f
#endif
#ifndef MOTOR_CTRL_DIR_SETUP_MS
#define MOTOR_CTRL_DIR_SETUP_MS 15
#endif
#ifndef MOTOR_CTRL_TASK_STACK
#define MOTOR_CTRL_TASK_STACK 4096
#endif
#ifndef MOTOR_CTRL_TASK_PRIO
#define MOTOR_CTRL_TASK_PRIO 1
#endif

struct MotorCtrl {
  const char* name;
  uint8_t id;       // 0 = A (loop), 1 = B (tarea)
  int pinStep;
  int pinDir;       // -1 = sin DIR GPIO (solo PUL)
  int taskCore;     // solo B (recomendado: 1)
  float gearRatio;  // 1.0 = directo; 15.0 = reductor 15:1

  StepperRMT* driver;
  TaskHandle_t taskHandle;

  volatile float commandedRpm;  // RPM de salida pedida/aplicada
  volatile float requestRpm;    // pedido de salida (solo B)
  volatile bool requestPending;
  volatile bool abortRequested;
  volatile bool running;
};

// Utilidades
float motorUiToDriveRpm(float uiSignedShaftRpm);
float motorClampGearRatio(float ratio);
void motorSetGearRatio(MotorCtrl* mc, float ratio);
float motorGearRatio(const MotorCtrl* mc);
float motorOutputToShaftRpm(const MotorCtrl* mc, float outputSigned);
float motorShaftToOutputRpm(const MotorCtrl* mc, float shaftSigned);
float motorClampOutputRpm(const MotorCtrl* mc, float outputSigned);
float motorClampSignedRpm(float signedShaftRpm);  // clamp directo de eje
float motorCurrentOutputRpm(const MotorCtrl* mc);
const char* motorDirName(float commandedOutputRpm);

// MotorA — dueño: loop
bool motorABegin(MotorCtrl* mc);
bool motorARun(MotorCtrl* mc, float signedOutputRpm);  // 0 = stop

// MotorB — dueño: tarea FreeRTOS
bool motorBBegin(MotorCtrl* mc, const char* taskName = "motorB_task");
void motorBRequestRpm(MotorCtrl* mc, float signedOutputRpm);
void motorBRequestStop(MotorCtrl* mc);
