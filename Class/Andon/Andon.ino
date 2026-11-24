/*
 * Andon.ino — MCU Andon (ESP32)
 * Estado general de la máquina. Ethernet no se usa.
 *
 * Órdenes por Serial (JSON, una línea).
 *
 * Indicadores:
 *   idle/simple     = todo off
 *   trabajando      = LED verde
 *   error / tecnico = LED rojo + buzzer
 *   falta_material / materialista / joint = LED amarillo
 *   jefe            = buzzer
 *   terminado       = LED verde parpadeando + buzzer
 *   presostato HIGH = LED rojo + buzzer (prioridad)
 */

#include <Arduino.h>
#include <string.h>

static const uint32_t PARAM_SERIAL_BAUD = 115200;
static const uint32_t PARAM_BLINK_MS = 500;

static const int PIN_LED_GREEN = 16;
static const int PIN_LED_YELLOW = 17;
static const int PIN_LED_RED = 21;
static const int PIN_BUZZER = 25;
static const int PIN_PRESSURE = 33;

static const char* kStateName[] = {
    "simple",
    "trabajando",
    "error",
    "falta_material",
    "terminado",
    "materialista",
    "jefe",
    "tecnico",
};
static const uint8_t ANDON_SIMPLE = 0;
static const uint8_t ANDON_TRABAJANDO = 1;
static const uint8_t ANDON_ERROR = 2;
static const uint8_t ANDON_FALTA_MATERIAL = 3;
static const uint8_t ANDON_TERMINADO = 4;
static const uint8_t ANDON_MATERIALISTA = 5;
static const uint8_t ANDON_JEFE = 6;
static const uint8_t ANDON_TECNICO = 7;
static const uint8_t kStateCount = sizeof(kStateName) / sizeof(kStateName[0]);

static uint8_t gState = ANDON_SIMPLE;
static char gSerialLine[96];
static size_t gSerialLineLen = 0;

static bool jsonFindKey(const char* json, const char* key, const char** valOut) {
  char needle[24];
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

static int stateFromLine(const char* line) {
  const char* p = nullptr;
  if (!jsonFindKey(line, "state", &p) || *p != '"') return -1;
  p++;
  for (uint8_t i = 0; i < kStateCount; i++) {
    size_t n = strlen(kStateName[i]);
    if (strncmp(p, kStateName[i], n) == 0 && p[n] == '"') return (int)i;
  }
  /* Alias idle → simple */
  if (strncmp(p, "idle", 4) == 0 && p[4] == '"') return (int)ANDON_SIMPLE;
  return -1;
}

static bool pressureOk() {
  return digitalRead(PIN_PRESSURE) == LOW;
}

static void applyIndicators() {
  bool green = false;
  bool yellow = false;
  bool red = false;
  bool buzz = false;
  bool blink_green = false;

  if (!pressureOk() || gState == ANDON_ERROR || gState == ANDON_TECNICO) {
    red = true;
    buzz = true;
  } else if (gState == ANDON_JEFE) {
    buzz = true;
  } else if (gState == ANDON_TERMINADO) {
    blink_green = true;
    buzz = true;
  } else if (
      gState == ANDON_FALTA_MATERIAL || gState == ANDON_MATERIALISTA) {
    yellow = true;
  } else if (gState == ANDON_TRABAJANDO) {
    green = true;
  }
  /* simple / idle: todo off */

  if (blink_green) {
    green = ((millis() / PARAM_BLINK_MS) & 1u) == 0u;
  }

  digitalWrite(PIN_LED_GREEN, green ? HIGH : LOW);
  digitalWrite(PIN_LED_YELLOW, yellow ? HIGH : LOW);
  digitalWrite(PIN_LED_RED, red ? HIGH : LOW);
  digitalWrite(PIN_BUZZER, buzz ? HIGH : LOW);
}

static void printStatus() {
  Serial.print(F("{\"op\":\"status\",\"mcu\":\"andon\",\"state\":\""));
  Serial.print(kStateName[gState]);
  Serial.print(F("\",\"pressure_ok\":"));
  Serial.print(pressureOk() ? "true" : "false");
  Serial.println(F("}"));
}

static void handleSerialLine(char* line) {
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  if (strstr(line, "\"op\":\"status\"")) {
    printStatus();
    return;
  }
  if (strstr(line, "\"op\":\"set_state\"")) {
    int next = stateFromLine(line);
    if (next < 0) {
      Serial.println(F("{\"op\":\"set_state\",\"ok\":false}"));
      return;
    }
    gState = (uint8_t)next;
    applyIndicators();
    Serial.print(F("{\"op\":\"set_state\",\"ok\":true,\"state\":\""));
    Serial.print(kStateName[gState]);
    Serial.println(F("\"}"));
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

void setup() {
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_PRESSURE, INPUT_PULLUP);
  applyIndicators();
  Serial.begin(PARAM_SERIAL_BAUD);
  Serial.println(F("{\"op\":\"boot\",\"mcu\":\"andon\",\"state\":\"simple\"}"));
}

void loop() {
  pollSerial();
  applyIndicators();
}
