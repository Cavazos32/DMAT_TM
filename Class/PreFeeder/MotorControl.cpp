#include "MotorControl.h"
#include <math.h>

float motorUiToDriveRpm(float uiSignedShaftRpm)
{
  const float mag = fabsf(uiSignedShaftRpm);
  return (uiSignedShaftRpm > 0.0f) ? -mag : mag;
}

float motorClampGearRatio(float ratio)
{
  // !(ratio >= 1) también atrapa NaN
  if (!(ratio >= 1.0f)) return 1.0f;
  if (ratio > MOTOR_CTRL_GEAR_MAX) return MOTOR_CTRL_GEAR_MAX;
  return ratio;
}

void motorSetGearRatio(MotorCtrl* mc, float ratio)
{
  if (!mc) return;
  mc->gearRatio = motorClampGearRatio(ratio);
}

float motorGearRatio(const MotorCtrl* mc)
{
  if (!mc) return 1.0f;
  return motorClampGearRatio(mc->gearRatio);
}

float motorClampSignedRpm(float signedShaftRpm)
{
  float mag = fabsf(signedShaftRpm);
  if (mag < 0.01f) return 0.0f;
  if (mag < MOTOR_CTRL_RPM_MIN) mag = MOTOR_CTRL_RPM_MIN;
  if (mag > MOTOR_CTRL_RPM_MAX) mag = MOTOR_CTRL_RPM_MAX;
  return (signedShaftRpm > 0.0f) ? mag : -mag;
}

float motorOutputToShaftRpm(const MotorCtrl* mc, float outputSigned)
{
  const float g = motorGearRatio(mc);
  return motorClampSignedRpm(outputSigned * g);
}

float motorShaftToOutputRpm(const MotorCtrl* mc, float shaftSigned)
{
  const float g = motorGearRatio(mc);
  return shaftSigned / g;
}

float motorClampOutputRpm(const MotorCtrl* mc, float outputSigned)
{
  const float shaft = motorOutputToShaftRpm(mc, outputSigned);
  return motorShaftToOutputRpm(mc, shaft);
}

float motorCurrentOutputRpm(const MotorCtrl* mc)
{
  if (!mc || !mc->driver) return 0.0f;
  return motorShaftToOutputRpm(mc, mc->driver->currentRPM());
}

const char* motorDirName(float commandedOutputRpm)
{
  if (fabsf(commandedOutputRpm) < 0.01f) return "stop";
  return commandedOutputRpm > 0.0f ? "cw" : "ccw";
}

static bool waitStoppedLoop(StepperRMT* m, uint16_t timeoutMs = 200)
{
  const uint32_t t0 = millis();
  while (fabsf(m->currentRPM()) > 1.0f && (millis() - t0) < timeoutMs)
    delay(1);
  return fabsf(m->currentRPM()) <= 1.0f;
}

static bool waitStoppedTask(StepperRMT* m, uint16_t timeoutMs = 200)
{
  const uint32_t t0 = millis();
  while (fabsf(m->currentRPM()) > 1.0f && (millis() - t0) < timeoutMs)
    vTaskDelay(pdMS_TO_TICKS(1));
  return fabsf(m->currentRPM()) <= 1.0f;
}

static void hardStopLoop(MotorCtrl* mc)
{
  mc->driver->Stop();
  waitStoppedLoop(mc->driver);
  mc->commandedRpm = 0.0f;
  mc->running = false;
}

static void brakeLoop(MotorCtrl* mc)
{
  mc->driver->Brake();
  waitStoppedLoop(mc->driver);
  mc->commandedRpm = 0.0f;
  mc->running = false;
}

static void stopMotionTask(MotorCtrl* mc)
{
  mc->driver->Stop();
  waitStoppedTask(mc->driver);
  mc->commandedRpm = 0.0f;
  mc->running = false;
}

static void brakeTask(MotorCtrl* mc)
{
  mc->driver->Brake();
  waitStoppedTask(mc->driver);
  mc->commandedRpm = 0.0f;
  mc->running = false;
}

static bool motorAStartSigned(MotorCtrl* mc, float outputSignedRpm)
{
  const float shaftTarget = motorOutputToShaftRpm(mc, outputSignedRpm);
  const float outputTarget = motorShaftToOutputRpm(mc, shaftTarget);
  if (fabsf(shaftTarget) < 0.01f)
  {
    brakeLoop(mc);
    return true;
  }

  const float driveRpm = motorUiToDriveRpm(shaftTarget);

  if (fabsf(mc->commandedRpm) > 0.01f || fabsf(mc->driver->currentRPM()) > 1.0f)
  {
    hardStopLoop(mc);
    delay(MOTOR_CTRL_DIR_SETUP_MS);
  }

  if (!mc->driver->setSpeed(driveRpm, MOTOR_CTRL_ACCEL))
  {
    hardStopLoop(mc);
    delay(MOTOR_CTRL_DIR_SETUP_MS);
    if (!mc->driver->setSpeed(driveRpm, MOTOR_CTRL_ACCEL))
      return false;
  }

  mc->commandedRpm = outputTarget;
  mc->running = true;
  return true;
}

bool motorARun(MotorCtrl* mc, float signedOutputRpm)
{
  const float shaftTarget = motorOutputToShaftRpm(mc, signedOutputRpm);
  const float outputTarget = motorShaftToOutputRpm(mc, shaftTarget);
  if (fabsf(shaftTarget) < 0.01f)
  {
    if (fabsf(mc->commandedRpm) > 0.01f)
      hardStopLoop(mc);
    else
      mc->commandedRpm = 0.0f;
    mc->running = false;
    return true;
  }

  const float cur = mc->commandedRpm;
  if (fabsf(cur - outputTarget) <= 0.5f)
    return true;

  if (fabsf(cur) > 0.01f && ((cur > 0.0f) == (outputTarget > 0.0f)))
  {
    if (mc->driver->setSpeed(motorUiToDriveRpm(shaftTarget), MOTOR_CTRL_ACCEL))
    {
      mc->commandedRpm = outputTarget;
      mc->running = true;
      return true;
    }
  }

  return motorAStartSigned(mc, outputTarget);
}

static bool motorBStartSigned(MotorCtrl* mc, float outputSignedRpm)
{
  const float shaftTarget = motorOutputToShaftRpm(mc, outputSignedRpm);
  const float outputTarget = motorShaftToOutputRpm(mc, shaftTarget);
  if (fabsf(shaftTarget) < 0.01f)
  {
    brakeTask(mc);
    return true;
  }

  const float driveRpm = motorUiToDriveRpm(shaftTarget);

  stopMotionTask(mc);
  vTaskDelay(pdMS_TO_TICKS(MOTOR_CTRL_DIR_SETUP_MS));

  if (!mc->driver->setSpeed(driveRpm, MOTOR_CTRL_ACCEL))
  {
    stopMotionTask(mc);
    vTaskDelay(pdMS_TO_TICKS(MOTOR_CTRL_DIR_SETUP_MS));
    if (!mc->driver->setSpeed(driveRpm, MOTOR_CTRL_ACCEL))
      return false;
  }

  mc->commandedRpm = outputTarget;
  mc->running = true;
  return true;
}

static bool motorBApplySpeed(MotorCtrl* mc, float signedOutputRpm)
{
  const float shaftTarget = motorOutputToShaftRpm(mc, signedOutputRpm);
  const float outputTarget = motorShaftToOutputRpm(mc, shaftTarget);
  if (fabsf(shaftTarget) < 0.01f)
  {
    if (fabsf(mc->commandedRpm) > 0.01f)
      brakeTask(mc);
    else
      mc->commandedRpm = 0.0f;
    return true;
  }

  const float cur = mc->commandedRpm;
  if (fabsf(cur) > 0.01f && ((cur > 0.0f) == (outputTarget > 0.0f)))
  {
    if (fabsf(cur - outputTarget) <= 0.5f)
      return true;
    if (mc->driver->setSpeed(motorUiToDriveRpm(shaftTarget), MOTOR_CTRL_ACCEL))
    {
      mc->commandedRpm = outputTarget;
      mc->running = true;
      return true;
    }
  }

  return motorBStartSigned(mc, outputTarget);
}

static void motorBTask(void* param)
{
  MotorCtrl* mc = static_cast<MotorCtrl*>(param);

  brakeTask(mc);
  Serial.printf("[%s] FreeRTOS core %d PUL=%d gear=%.1f:1\n",
                mc->name, xPortGetCoreID(), mc->pinStep, motorGearRatio(mc));

  for (;;)
  {
    if (mc->abortRequested)
    {
      if (fabsf(mc->commandedRpm) > 0.01f
          || fabsf(mc->driver->currentRPM()) > 1.0f)
        brakeTask(mc);
      else
        mc->commandedRpm = 0.0f;
      mc->requestRpm = 0.0f;
      mc->requestPending = false;
      mc->abortRequested = false;
    }

    if (mc->requestPending)
    {
      const float want = mc->requestRpm;
      mc->requestPending = false;
      motorBApplySpeed(mc, want);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

static bool beginDriver(MotorCtrl* mc)
{
  mc->driver = nullptr;
  mc->taskHandle = nullptr;
  mc->commandedRpm = 0.0f;
  mc->requestRpm = 0.0f;
  mc->requestPending = false;
  mc->abortRequested = false;
  mc->running = false;
  mc->gearRatio = motorClampGearRatio(mc->gearRatio);

  StepperRMT::Pins pins = {
    mc->pinStep,
    mc->pinDir,
    -1, -1, -1, -1, -1
  };

  mc->driver = new StepperRMT(DRIVER_DM556T, pins);
  if (!mc->driver->begin())
  {
    Serial.printf("ERROR: %s StepperRMT begin() falló\n", mc->name);
    return false;
  }
  mc->driver->setDefault(MOTOR_CTRL_MICROSTEP, MOTOR_CTRL_ACCEL, MOTOR_CTRL_DECEL);
  mc->driver->Brake();
  return true;
}

bool motorABegin(MotorCtrl* mc)
{
  if (!beginDriver(mc))
    return false;
  Serial.printf("[%s] dueño=loop() PUL=%d DIR=%d gear=%.1f:1\n",
                mc->name, mc->pinStep, mc->pinDir, motorGearRatio(mc));
  return true;
}

bool motorBBegin(MotorCtrl* mc, const char* taskName)
{
  if (!beginDriver(mc))
    return false;

  if (mc->taskCore < 0 || mc->taskCore > 1)
    mc->taskCore = 1;

  BaseType_t ok = xTaskCreatePinnedToCore(
    motorBTask,
    taskName ? taskName : "motorB_task",
    MOTOR_CTRL_TASK_STACK,
    mc,
    MOTOR_CTRL_TASK_PRIO,
    &mc->taskHandle,
    mc->taskCore);

  if (ok != pdPASS)
  {
    Serial.printf("ERROR: %s no pudo crear tarea\n", mc->name);
    return false;
  }
  return true;
}

void motorBRequestRpm(MotorCtrl* mc, float signedOutputRpm)
{
  mc->requestRpm = motorClampOutputRpm(mc, signedOutputRpm);
  mc->abortRequested = false;
  mc->requestPending = true;
}

void motorBRequestStop(MotorCtrl* mc)
{
  mc->requestRpm = 0.0f;
  mc->abortRequested = true;
  mc->requestPending = true;
}
