// Última modificación: 2026-11-05 19:56 | America/Santiago (UTC-03).
// ESP32-S3, Arduino-ESP32 2.0.14. HTTP/UDP y control local de encoders a 100 Hz.
// La duración se cumple en un temporizador local; el encendido nunca mueve motores.
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <driver/pcnt.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <soc/gpio_struct.h>
#include <atomic>
#include "WheelVelocityControl.h"

const uint8_t PWM_PIN[] = {47, 8}, PH_PIN[] = {5, 9};
const uint8_t SLEEP_PIN[] = {6, 10}, FAULT_PIN[] = {7, 11};
const bool INVERTIR[] = {false, true};
const uint8_t ENC_A[] = {15, 17}, ENC_B[] = {16, 18};
const int8_t SIGNO_ENCODER[] = {-1, 1};
const uint8_t STOP_PIN = 48, TURBINA_PIN = 12;
const int PULSE_PWM = 100;
const int MAX_VELOCITY_MM_S = robotracers::WheelVelocityControl::MAX_VELOCITY_MM_S;
const uint32_t PULSE_MS = 1000;
const uint32_t NAV_LEASE_MS = 300, NAV_INITIAL_MS = 1000;
const uint16_t NAV_UDP_PORT = 4210;
const size_t MAX_LINE = 512, MAX_ID = 64;

WebServer server(80);
WiFiUDP navigationUDP;
Preferences preferences;
Adafruit_NeoPixel rgb(1, 45, NEO_GRB + NEO_KHZ800);
SemaphoreHandle_t motorMutex = nullptr;
SemaphoreHandle_t udpMutex = nullptr;
TaskHandle_t motorTask = nullptr;
esp_timer_handle_t pulseTimer = nullptr;
std::atomic<bool> statusDirty(true);
volatile bool emergencyRequested = false;
bool moving = false, driversAwake = false, driverFault = false;
bool powerInitOK = false, hardwareOK = false, storageOK = false;
bool serverStarted = false;
std::atomic<bool> udpStarted(false);
bool navigationActive = false, haveControlSeq = false;
bool resumeBlocked = false;
bool encoderReady = false, velocityMode = false;
int16_t previousPCNT[2] = {};
int64_t encoderCount[2] = {};
int appliedPwm[2] = {};
robotracers::WheelVelocityControl wheelControl[2];
uint32_t controlSeq = 0;
int8_t lastDirection[2] = {1, 1};
int64_t controlDeadline = 0;
char bootId[17] = {}, token[33] = {}, lastId[MAX_ID + 1] = {};
char controlSession[MAX_ID + 1] = {}, retiredSessions[8][MAX_ID + 1] = {};
char stopReason[40] = "boot", recentIds[8][MAX_ID + 1] = {};
uint8_t nextRecent = 0, nextRetired = 0;
String robotMac, savedSSID, savedPassword;
uint32_t lastPowerCheck = 0, lastWifiRetry = 0;
wl_status_t previousWifi = WL_NO_SHIELD;
String previousIP;
char usbLine[MAX_LINE + 1] = {}, usbOutput[2048] = {}, usbError[64] = {};
size_t usbLength = 0, outputLength = 0, outputOffset = 0;
bool usbOverflow = false, previousSerial = false;
bool previousStopHeld = false;

void markStatus() { statusDirty.store(true); }

void IRAM_ATTR emergencyISR() {
  // Los dos nSLEEP están en el banco GPIO0..31: parada inmediata sin WiFi/I2C.
  GPIO.out_w1tc = (1UL << 6) | (1UL << 10);
  emergencyRequested = true;
}

bool stopHeld() { return digitalRead(STOP_PIN) == LOW; }

// Solo llamar con motorMutex tomado (excepto la inicialización, sin tareas).
void outputsOffLocked(bool sleepDrivers) {
  velocityMode = false;
  wheelControl[0].setTarget(0); wheelControl[1].setTarget(0);
  ledcWrite(0, 0); ledcWrite(1, 0);
  appliedPwm[0] = appliedPwm[1] = 0;
  if (sleepDrivers) {
    digitalWrite(SLEEP_PIN[0], LOW); digitalWrite(SLEEP_PIN[1], LOW);
    driversAwake = false;
  }
  moving = false;
}

void invalidateNavigationLocked() {
  if (navigationActive) {
    strlcpy(retiredSessions[nextRetired], controlSession, sizeof(retiredSessions[nextRetired]));
    nextRetired = (nextRetired + 1) % 8;
  }
  navigationActive = false;
  controlSession[0] = 0;
  haveControlSeq = false;
}

void stopLocked(const char *reason, bool sleepDrivers) {
  const bool wasNavigation = navigationActive;
  // Una parada explícita permanece bloqueada aunque después cambie el motivo
  // visible por pérdida de WiFi o alimentación. Solo un nuevo inicio la libera.
  if (strcmp(reason, "button_stop") == 0 || strcmp(reason, "driver_fault") == 0 ||
      strcmp(reason, "remote") == 0 || strcmp(reason, "remote_stop") == 0 ||
      strcmp(reason, "manual_stop") == 0 || strcmp(reason, "encoder_stall") == 0 ||
      strcmp(reason, "encoder_fault") == 0) resumeBlocked = true;
  invalidateNavigationLocked();
  outputsOffLocked(sleepDrivers);
  if (strcmp(stopReason, reason) != 0) {
    strlcpy(stopReason, reason, sizeof(stopReason));
    markStatus();
  }
  if (wasNavigation) markStatus();
}

void stopMotors(const char *reason, bool sleepDrivers) {
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  const bool changed = moving || driversAwake;
  stopLocked(reason, sleepDrivers || !powerInitOK || driverFault || stopHeld());
  if (pulseTimer) esp_timer_stop(pulseTimer);
  if (changed) markStatus();
  xSemaphoreGive(motorMutex);
}

void timerExpired(void *) {
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  // Cada pulso o paquete válido actualiza controlDeadline bajo este mutex.
  // Un callback anterior ya encolado no cancela una concesión renovada.
  if ((moving || navigationActive) && esp_timer_get_time() >= controlDeadline) {
    stopLocked(navigationActive ? "control_timeout" : "completed", false);
    markStatus();
  }
  xSemaphoreGive(motorMutex);
}

bool readRegister(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(0x74); Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(uint8_t(0x74), uint8_t(1)) != 1) return false;
  value = Wire.read(); return true;
}

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(0x74); Wire.write(reg); Wire.write(value);
  if (Wire.endTransmission() != 0) return false;
  uint8_t readback;
  return readRegister(reg, readback) && readback == value;
}

bool configurePower() {
  // Ajustes U11 de 6 V ya comprobados en la PCB y en REV3.11.
  const uint8_t settings[][2] = {{6,0x22},{4,0x83},{0,0xF7},{1,0x07},
                               {2,0xE4},{3,0x01},{5,0xE0},{6,0xA2}};
  for (const auto &setting : settings) {
    if (!writeRegister(setting[0], setting[1])) { writeRegister(6, 0x22); return false; }
  }
  delay(30);
  uint8_t status;
  if (!readRegister(7, status) || (status & 0xE0)) { writeRegister(6, 0x22); return false; }
  return true;
}

bool readyLocked() {
  return hardwareOK && powerInitOK && !driverFault && !stopHeld() && !emergencyRequested;
}

bool seenId(const char *id) {
  for (const auto &entry : recentIds) if (entry[0] && strcmp(entry, id) == 0) return true;
  return false;
}

// Devuelve HTTP200/409/503; la comprobación y el arranque son una operación.
int beginPulse(const char *id, bool &replayed, bool &isMoving) {
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  if (navigationActive) {
    replayed = false; isMoving = moving; xSemaphoreGive(motorMutex); return 409;
  }
  replayed = seenId(id);
  if (replayed) { isMoving = moving; xSemaphoreGive(motorMutex); return 200; }
  if (moving || !readyLocked()) {
    isMoving = moving; xSemaphoreGive(motorMutex); return 409;
  }
  outputsOffLocked(true);
  for (int i = 0; i < 2; i++) {
    digitalWrite(PH_PIN[i], (true != INVERTIR[i]) ? HIGH : LOW);
    lastDirection[i] = 1;
    digitalWrite(SLEEP_PIN[i], HIGH);
  }
  driversAwake = true;
  delay(3); // Despertar los drivers antes de nFAULT y antes de iniciar el segundo.
  if (stopHeld() || emergencyRequested) {
    stopLocked("button_stop", true); isMoving = false;
    xSemaphoreGive(motorMutex); return 409;
  }
  if (digitalRead(FAULT_PIN[0]) == LOW || digitalRead(FAULT_PIN[1]) == LOW) {
    driverFault = true; stopLocked("driver_fault", true); isMoving = false;
    xSemaphoreGive(motorMutex); return 409;
  }
  esp_timer_stop(pulseTimer);
  controlDeadline = esp_timer_get_time() + int64_t(PULSE_MS) * 1000;
  ledcWrite(0, PULSE_PWM); ledcWrite(1, PULSE_PWM);
  appliedPwm[0] = appliedPwm[1] = PULSE_PWM;
  moving = true;
  const int64_t remaining = controlDeadline - esp_timer_get_time();
  if (remaining <= 0 || esp_timer_start_once(pulseTimer, uint64_t(remaining)) != ESP_OK) {
    stopLocked("timer_error", true); isMoving = false;
    xSemaphoreGive(motorMutex); return 503;
  }
  strlcpy(lastId, id, sizeof(lastId));
  strlcpy(recentIds[nextRecent], id, sizeof(recentIds[nextRecent]));
  nextRecent = (nextRecent + 1) % 8;
  strlcpy(stopReason, "running", sizeof(stopReason));
  isMoving = true; markStatus();
  xSemaphoreGive(motorMutex); return 200;
}

void buildStatus(JsonDocument &doc, bool includeToken) {
  doc["device"] = "RobotRacers"; doc["version"] = 1;
  doc["mac"] = robotMac; doc["boot_id"] = bootId;
  doc["ip"] = WiFi.localIP().toString();
  doc["connected"] = WiFi.status() == WL_CONNECTED;
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  doc["ready"] = readyLocked(); doc["moving"] = moving;
  doc["resume_blocked"] = resumeBlocked;
  doc["velocity_control"] = true; doc["encoder_ready"] = encoderReady;
  doc["max_velocity_mm_s"] = MAX_VELOCITY_MM_S;
  for (int i = 0; i < 2; i++) {
    doc["count"][i] = encoderCount[i];
    doc["speed_mm_s"][i] = wheelControl[i].speed();
    doc["applied_pwm"][i] = appliedPwm[i];
    doc["startup"][i] = velocityMode && wheelControl[i].startup();
  }
  doc["navigation_active"] = navigationActive; doc["control_seq"] = controlSeq;
  doc["control_mode"] = navigationActive ? "navigation" : moving ? "pulse" : "idle";
  doc["control_session"] = String(controlSession);
  // JSON conserva copias: callback y loop pueden modificar el estado después.
  doc["last_id"] = String(lastId); doc["stop_reason"] = String(stopReason);
  xSemaphoreGive(motorMutex);
  if (includeToken) doc["token"] = token;
}

void sendJson(int code, JsonDocument &doc) {
  String body; serializeJson(doc, body);
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}

void sendError(int code, const char *reason) {
  JsonDocument doc; doc["accepted"] = false; doc["error"] = reason;
  sendJson(code, doc);
}

bool tokenMatches(const char *supplied, size_t length) {
  uint8_t difference = length == 32 ? 0 : 1;
  for (size_t i = 0; i < 32; i++) difference |= token[i] ^ (i < length ? supplied[i] : 0);
  return difference == 0;
}

bool authorized() {
  const String supplied = server.header("X-Robot-Token");
  return tokenMatches(supplied.c_str(), supplied.length());
}

bool validSession(const char *value, size_t length) {
  if (length != 36 && length != 32) return false;
  for (size_t i = 0; i < length; i++) {
    const bool separator = length == 36 && (i == 8 || i == 13 || i == 18 || i == 23);
    if (separator ? value[i] != '-' : !isxdigit(uint8_t(value[i]))) return false;
  }
  return true;
}

bool armDeadlineLocked(int64_t deadline) {
  esp_timer_stop(pulseTimer);
  controlDeadline = deadline;
  const int64_t remaining = controlDeadline - esp_timer_get_time();
  if (remaining <= 0 || esp_timer_start_once(pulseTimer, uint64_t(remaining)) != ESP_OK) {
    stopLocked("timer_error", true); return false;
  }
  return true;
}

void navigationStartRequest() {
  if (!authorized()) { sendError(403, "unauthorized"); return; }
  const String body = server.arg("plain");
  JsonDocument request;
  if (body.length() > MAX_LINE || deserializeJson(request, body) || !request.is<JsonObject>() ||
      (request.size() != 1 && request.size() != 2) || !request["session"].is<const char *>() ||
      (request.size() == 2 && (!request["resume"].is<bool>() || !request["resume"].as<bool>()))) {
    sendError(400, "invalid_session"); return;
  }
  const char *session = request["session"].as<const char *>();
  const size_t length = request["session"].as<JsonString>().size();
  if (strlen(session) != length || !validSession(session, length)) {
    sendError(400, "invalid_session"); return;
  }
  const bool resuming = request.size() == 2;
  int result = 200;
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  if (navigationActive && esp_timer_get_time() >= controlDeadline) stopLocked("control_timeout", false);
  bool retired = false;
  for (const auto &entry : retiredSessions) if (entry[0] && strcmp(entry, session) == 0) retired = true;
  // Comprobar el motivo bajo el mismo mutex que arma los motores cierra la
  // carrera entre el último GET del ordenador y una parada física o remota.
  if (resuming && resumeBlocked) result = 423;
  else if (navigationActive) {
    if (strcmp(controlSession, session) != 0) result = 409;
    // Repetir HTTP no renueva la concesión ni reinicia los motores.
  } else if (moving || !readyLocked() || retired) result = 409;
  else if (!udpStarted) result = 503;
  else {
    outputsOffLocked(false);
    strlcpy(controlSession, session, sizeof(controlSession));
    navigationActive = true; haveControlSeq = false; controlSeq = 0;
    if (!armDeadlineLocked(esp_timer_get_time() + int64_t(NAV_INITIAL_MS) * 1000)) result = 503;
    else {
      if (!resuming) {
        resumeBlocked = false;
        for (int i = 0; i < 2; i++) wheelControl[i].clearFault();
      }
      strlcpy(stopReason, "navigation", sizeof(stopReason)); markStatus();
    }
  }
  xSemaphoreGive(motorMutex);
  if (result != 200) {
    sendError(result, result == 423 ? "explicit_start_required" :
              result == 409 ? "not_ready_or_busy" : "navigation_unavailable"); return;
  }
  JsonDocument ack; ack["accepted"] = true; ack["session"] = session; sendJson(200, ack);
}

bool applySignedPwmLocked(int left, int right) {
  if (stopHeld() || emergencyRequested) { stopLocked("button_stop", true); return false; }
  const int wheel[] = {left, right};
  const bool anyMotion = left != 0 || right != 0;
  for (int i = 0; i < 2; i++) {
    const int8_t direction = wheel[i] < 0 ? -1 : 1;
    if (wheel[i] != 0 && direction != lastDirection[i]) {
      ledcWrite(i, 0);
      digitalWrite(PH_PIN[i], ((direction > 0) != INVERTIR[i]) ? HIGH : LOW);
      lastDirection[i] = direction;
    }
  }
  if (anyMotion && !driversAwake) {
    ledcWrite(0, 0); ledcWrite(1, 0);
    if (stopHeld() || emergencyRequested) { stopLocked("button_stop", true); return false; }
    for (int i = 0; i < 2; i++) {
      digitalWrite(PH_PIN[i], ((lastDirection[i] > 0) != INVERTIR[i]) ? HIGH : LOW);
      digitalWrite(SLEEP_PIN[i], HIGH);
    }
    driversAwake = true;
    // La ISR puede bajar SLEEP entre el check anterior y este despertar.
    if (stopHeld() || emergencyRequested) { stopLocked("button_stop", true); return false; }
    delay(3);
  }
  if (stopHeld() || emergencyRequested) { stopLocked("button_stop", true); return false; }
  if (driversAwake && (digitalRead(FAULT_PIN[0]) == LOW || digitalRead(FAULT_PIN[1]) == LOW)) {
    driverFault = true; stopLocked("driver_fault", true); return false;
  }
  if (stopHeld() || emergencyRequested) { stopLocked("button_stop", true); return false; }
  ledcWrite(0, abs(left)); ledcWrite(1, abs(right));
  appliedPwm[0] = left; appliedPwm[1] = right;
  moving = anyMotion;
  return true;
}

void applyNavigationPacket(const char *session, uint32_t sequence, bool stopCommand,
                           bool velocityCommand, int left, int right) {
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  if (!navigationActive || strcmp(controlSession, session) != 0) {
    xSemaphoreGive(motorMutex); return;
  }
  const int64_t now = esp_timer_get_time();
  if (now >= controlDeadline) {
    stopLocked("control_timeout", false); esp_timer_stop(pulseTimer);
    xSemaphoreGive(motorMutex); return;
  }
  if (haveControlSeq && int32_t(sequence - controlSeq) <= 0) {
    xSemaphoreGive(motorMutex); return;
  }
  if (stopCommand) {
    controlSeq = sequence;
    stopLocked("navigation_stop", false); esp_timer_stop(pulseTimer);
    xSemaphoreGive(motorMutex); return;
  }
  if (!readyLocked()) {
    stopLocked(stopHeld() || emergencyRequested ? "button_stop" : "not_ready", true);
    esp_timer_stop(pulseTimer); xSemaphoreGive(motorMutex); return;
  }
  const bool anyMotion = left != 0 || right != 0;
  if (velocityCommand) {
    if (!encoderReady) {
      stopLocked("encoder_fault", true); esp_timer_stop(pulseTimer);
      xSemaphoreGive(motorMutex); return;
    }
    if (!velocityMode) outputsOffLocked(false);
    velocityMode = true;
    wheelControl[0].setTarget(left); wheelControl[1].setTarget(right);
    for (int i = 0; i < 2; i++) {
      if (wheelControl[i].target() == 0) { ledcWrite(i, 0); appliedPwm[i] = 0; }
    }
    moving = anyMotion;
  } else {
    if (velocityMode) outputsOffLocked(false);
    if (!applySignedPwmLocked(left, right)) {
      esp_timer_stop(pulseTimer); xSemaphoreGive(motorMutex); return;
    }
  }
  if (armDeadlineLocked(now + int64_t(NAV_LEASE_MS) * 1000)) {
    controlSeq = sequence; haveControlSeq = true;
    strlcpy(stopReason, "navigation", sizeof(stopReason)); markStatus();
  }
  xSemaphoreGive(motorMutex);
}

void serviceNavigationUDP() {
  if (!udpStarted) return;
  for (int n = 0; n < 8; n++) {
    const int length = navigationUDP.parsePacket();
    if (length <= 0) break;
    if (length > int(MAX_LINE)) { navigationUDP.flush(); continue; }
    char packet[MAX_LINE + 1];
    const int received = navigationUDP.read(packet, length);
    if (received != length) { navigationUDP.flush(); continue; }
    packet[received] = 0;
    JsonDocument request;
    if (deserializeJson(request, packet, size_t(received)) || !request.is<JsonObject>() || request.size() != 7 ||
        !request["token"].is<const char *>() || !request["session"].is<const char *>() ||
        !request["command"].is<const char *>() || !request["seq"].is<uint32_t>() ||
        !request["left"].is<int>() || !request["right"].is<int>() || !request["lease_ms"].is<int>()) continue;
    const char *suppliedToken = request["token"].as<const char *>();
    const size_t tokenLength = request["token"].as<JsonString>().size();
    if (strlen(suppliedToken) != tokenLength || !tokenMatches(suppliedToken, tokenLength)) continue;
    const char *session = request["session"].as<const char *>();
    const size_t sessionLength = request["session"].as<JsonString>().size();
    if (strlen(session) != sessionLength || !validSession(session, sessionLength)) continue;
    const char *command = request["command"].as<const char *>();
    if (request["command"].as<JsonString>().size() != strlen(command)) continue;
    const bool stopping = strcmp(command, "stop") == 0;
    const bool velocity = strcmp(command, "velocity") == 0;
    if (!stopping && !velocity && strcmp(command, "drive") != 0) continue;
    const int left = request["left"].as<int>(), right = request["right"].as<int>();
    const int commandLimit = velocity ? MAX_VELOCITY_MM_S : 100;
    if (left < -commandLimit || left > commandLimit || right < -commandLimit || right > commandLimit ||
        request["lease_ms"].as<int>() != int(NAV_LEASE_MS) || (stopping && (left != 0 || right != 0))) continue;
    applyNavigationPacket(session, request["seq"].as<uint32_t>(), stopping, velocity, left, right);
  }
}

bool configureEncoder(int i) {
  const pcnt_unit_t unit = static_cast<pcnt_unit_t>(i);
  pcnt_config_t config = {};
  config.unit = unit; config.counter_h_lim = 30000; config.counter_l_lim = -30000;
  config.lctrl_mode = PCNT_MODE_KEEP; config.hctrl_mode = PCNT_MODE_REVERSE;
  config.pulse_gpio_num = ENC_A[i]; config.ctrl_gpio_num = ENC_B[i]; config.channel = PCNT_CHANNEL_0;
  config.pos_mode = PCNT_COUNT_INC; config.neg_mode = PCNT_COUNT_DEC;
  if (pcnt_unit_config(&config) != ESP_OK) return false;
  config.pulse_gpio_num = ENC_B[i]; config.ctrl_gpio_num = ENC_A[i]; config.channel = PCNT_CHANNEL_1;
  config.pos_mode = PCNT_COUNT_DEC; config.neg_mode = PCNT_COUNT_INC;
  if (pcnt_unit_config(&config) != ESP_OK) return false;
  return pcnt_counter_pause(unit) == ESP_OK && pcnt_intr_disable(unit) == ESP_OK &&
    pcnt_set_filter_value(unit, 80) == ESP_OK && pcnt_filter_enable(unit) == ESP_OK &&
    pcnt_counter_clear(unit) == ESP_OK && pcnt_counter_resume(unit) == ESP_OK;
}

bool readEncodersLocked(double dt) {
  for (int i = 0; i < 2; i++) {
    int16_t actual;
    if (pcnt_get_counter_value(static_cast<pcnt_unit_t>(i), &actual) != ESP_OK) return false;
    int delta = int(actual) - int(previousPCNT[i]);
    if (delta < -15000) delta += 30000;
    if (delta > 15000) delta -= 30000;
    previousPCNT[i] = actual;
    if (abs(delta) > 100.0 + 16000.0 * dt) return false;
    encoderCount[i] += int64_t(SIGNO_ENCODER[i]) * delta;
  }
  return true;
}

void motorControlTask(void *) {
  TickType_t tick = xTaskGetTickCount();
  int64_t previousTime = esp_timer_get_time(), lastTelemetry = previousTime;
  for (;;) {
    // Si se necesitan ambos cerrojos, el orden es UDP -> motor. Ningún HTTP,
    // I2C ni escritura USB forma parte de esta tarea de control de 100 Hz.
    if (xSemaphoreTake(udpMutex, 0) == pdTRUE) {
      serviceNavigationUDP();
      xSemaphoreGive(udpMutex);
    }
    const int64_t now = esp_timer_get_time();
    const double dt = (now - previousTime) * 1e-6;
    previousTime = now;
    xSemaphoreTake(motorMutex, portMAX_DELAY);
    if (stopHeld() || emergencyRequested) {
      stopLocked("button_stop", true); esp_timer_stop(pulseTimer);
    } else if (driversAwake && (digitalRead(FAULT_PIN[0]) == LOW || digitalRead(FAULT_PIN[1]) == LOW)) {
      driverFault = true; stopLocked("driver_fault", true); esp_timer_stop(pulseTimer);
    } else if ((navigationActive || moving) && now >= controlDeadline) {
      stopLocked(navigationActive ? "control_timeout" : "completed", false);
      esp_timer_stop(pulseTimer);
    }
    if (encoderReady && !readEncodersLocked(dt)) {
      encoderReady = false; stopLocked("encoder_fault", true); esp_timer_stop(pulseTimer);
    }
    int command[2] = {};
    for (int i = 0; i < 2; i++) command[i] = wheelControl[i].update(encoderCount[i], now * 1e-6);
    if (velocityMode && navigationActive) {
      if (!readyLocked() || !encoderReady) {
        stopLocked(!encoderReady ? "encoder_fault" : "not_ready", true); esp_timer_stop(pulseTimer);
      } else if (wheelControl[0].stalled() || wheelControl[1].stalled()) {
        stopLocked("encoder_stall", true); esp_timer_stop(pulseTimer);
      } else if (!applySignedPwmLocked(command[0], command[1])) {
        esp_timer_stop(pulseTimer);
      }
    }
    if (now - lastTelemetry >= 100000) { markStatus(); lastTelemetry = now; }
    xSemaphoreGive(motorMutex);
    vTaskDelayUntil(&tick, pdMS_TO_TICKS(10));
  }
}

void pulseRequest() {
  if (!authorized()) { sendError(403, "unauthorized"); return; }
  const String body = server.arg("plain");
  JsonDocument request;
  if (body.length() > MAX_LINE || deserializeJson(request, body) || !request.is<JsonObject>() ||
      request.size() != 4 || !request["id"].is<const char *>() ||
      !request["left"].is<int>() || !request["right"].is<int>() ||
      !request["duration_ms"].is<int>() || request["left"].as<int>() != PULSE_PWM ||
      request["right"].as<int>() != PULSE_PWM || request["duration_ms"].as<int>() != int(PULSE_MS)) {
    sendError(400, "invalid_pulse"); return;
  }
  const char *id = request["id"].as<const char *>();
  const size_t length = strlen(id);
  if (length == 0 || length > MAX_ID || request["id"].as<JsonString>().size() != length) {
    sendError(400, "invalid_id"); return;
  }
  for (size_t i = 0; i < length; i++) if (uint8_t(id[i]) < 32 || uint8_t(id[i]) > 126) {
    sendError(400, "invalid_id"); return;
  }
  bool replayed = false, isMoving = false;
  const int result = beginPulse(id, replayed, isMoving);
  if (result != 200) { sendError(result, result == 409 ? "not_ready_or_busy" : "timer_error"); return; }
  JsonDocument ack; ack["accepted"] = true; ack["id"] = id; ack["moving"] = isMoving;
  if (replayed) ack["replayed"] = true;
  sendJson(200, ack);
}

void configureRoutes() {
  const char *headers[] = {"X-Robot-Token"}; server.collectHeaders(headers, 1);
  server.on("/status", HTTP_GET, [] { JsonDocument doc; buildStatus(doc, false); sendJson(200, doc); });
  server.on("/pulse", HTTP_POST, pulseRequest);
  server.on("/navigation/start", HTTP_POST, navigationStartRequest);
  server.on("/stop", HTTP_POST, [] {
    if (!authorized()) { sendError(403, "unauthorized"); return; }
    stopMotors("remote_stop", false);
    JsonDocument ack; ack["accepted"] = true; ack["moving"] = false; sendJson(200, ack);
  });
  server.onNotFound([] { sendError(404, "not_found"); });
}

bool validToken(const String &value) {
  if (value.length() != 32) return false;
  for (size_t i = 0; i < value.length(); i++) if (!isxdigit(uint8_t(value[i]))) return false;
  return true;
}

void loadSettings() {
  storageOK = preferences.begin("robotracers", false);
  String storedToken = storageOK ? preferences.getString("token", "") : "";
  if (!validToken(storedToken)) {
    snprintf(token, sizeof(token), "%08lx%08lx%08lx%08lx", (unsigned long)esp_random(),
             (unsigned long)esp_random(), (unsigned long)esp_random(), (unsigned long)esp_random());
    if (storageOK && preferences.putString("token", token) == 0) storageOK = false;
  } else strlcpy(token, storedToken.c_str(), sizeof(token));
  if (storageOK) {
    savedSSID = preferences.getString("ssid", "");
    savedPassword = preferences.getString("password", "");
  }
}

void usbCommand(const char *line) {
  JsonDocument request;
  if (deserializeJson(request, line) || !request.is<JsonObject>() || !request["command"].is<const char *>()) {
    strlcpy(usbError, "invalid_json", sizeof(usbError)); return;
  }
  const char *command = request["command"].as<const char *>();
  if (strcmp(command, "status") == 0) { markStatus(); return; }
  if (strcmp(command, "wifi") != 0) { strlcpy(usbError, "unknown_command", sizeof(usbError)); return; }
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  const bool isMoving = moving || navigationActive;
  xSemaphoreGive(motorMutex);
  if (isMoving) { strlcpy(usbError, "busy", sizeof(usbError)); return; }
  if (!request["ssid"].is<const char *>() || !request["password"].is<const char *>()) {
    strlcpy(usbError, "invalid_wifi", sizeof(usbError)); return;
  }
  const String ssid = request["ssid"].as<const char *>(), password = request["password"].as<const char *>();
  if (ssid.length() == 0 || ssid.length() > 32 ||
      (password.length() != 0 && (password.length() < 8 || password.length() > 63))) {
    strlcpy(usbError, "invalid_wifi", sizeof(usbError)); return;
  }
  if (!storageOK) { strlcpy(usbError, "storage_error", sizeof(usbError)); return; }
  preferences.putString("ssid", ssid); preferences.putString("password", password);
  // putString devuelve cero también para una contraseña vacía válida.
  if (preferences.getString("ssid", "") != ssid || preferences.getString("password", "") != password) {
    strlcpy(usbError, "storage_error", sizeof(usbError)); return;
  }
  savedSSID = ssid; savedPassword = password;
  WiFi.disconnect(false, false);
  WiFi.begin(savedSSID.c_str(), savedPassword.c_str());
  lastWifiRetry = millis(); markStatus();
}

void serviceUSB() {
  for (int n = 0; n < 64 && Serial.available(); n++) {
    const char value = char(Serial.read());
    if (value == '\r') continue;
    if (value == '\n') {
      if (usbOverflow) strlcpy(usbError, "line_too_long", sizeof(usbError));
      else if (usbLength) { usbLine[usbLength] = 0; usbCommand(usbLine); }
      usbLength = 0; usbOverflow = false;
    } else if (!usbOverflow) {
      if (usbLength >= MAX_LINE) usbOverflow = true;
      else usbLine[usbLength++] = value;
    }
  }
  const bool serialReady = bool(Serial);
  if (serialReady && !previousSerial) markStatus();
  previousSerial = serialReady;
  if (!serialReady) { outputLength = outputOffset = 0; return; }
  if (outputOffset >= outputLength) {
    outputLength = outputOffset = 0;
    JsonDocument doc;
    if (usbError[0]) {
      doc["device"] = "RobotRacers"; doc["version"] = 1; doc["error"] = usbError; usbError[0] = 0;
    } else if (statusDirty.exchange(false)) buildStatus(doc, true);
    else return;
    if (measureJson(doc) >= sizeof(usbOutput) - 1) {
      doc.clear(); doc["device"] = "RobotRacers"; doc["version"] = 1;
      doc["error"] = "status_too_large";
    }
    outputLength = serializeJson(doc, usbOutput, sizeof(usbOutput) - 1);
    usbOutput[outputLength++] = '\n';
  }
  const size_t remaining = outputLength - outputOffset;
  const size_t chunk = remaining < 48 ? remaining : 48;
  if (Serial.availableForWrite() >= int(chunk)) outputOffset += Serial.write(
    reinterpret_cast<const uint8_t *>(usbOutput + outputOffset), chunk);
}

void setup() {
  pinMode(TURBINA_PIN, OUTPUT); digitalWrite(TURBINA_PIN, LOW);
  pinMode(44, INPUT_PULLUP); pinMode(STOP_PIN, INPUT_PULLUP);
  motorMutex = xSemaphoreCreateMutex();
  udpMutex = xSemaphoreCreateMutex();
  hardwareOK = motorMutex != nullptr && udpMutex != nullptr;
  encoderReady = true;
  for (int i = 0; i < 2; i++) {
    pinMode(SLEEP_PIN[i], OUTPUT); digitalWrite(SLEEP_PIN[i], LOW);
    pinMode(PWM_PIN[i], OUTPUT); digitalWrite(PWM_PIN[i], LOW);
    pinMode(PH_PIN[i], OUTPUT); digitalWrite(PH_PIN[i], LOW);
    pinMode(FAULT_PIN[i], INPUT_PULLUP);
    if (ledcSetup(i, 20000, 8) == 0) hardwareOK = false;
    ledcAttachPin(PWM_PIN[i], i); ledcWrite(i, 0);
    if (!configureEncoder(i)) { encoderReady = false; hardwareOK = false; }
    wheelControl[i].reset(0, esp_timer_get_time() * 1e-6);
  }
  attachInterrupt(digitalPinToInterrupt(STOP_PIN), emergencyISR, FALLING);
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif
  Wire.setTimeOut(20);
  if (!Wire.begin(41, 40, 100000)) hardwareOK = false;
  esp_timer_create_args_t timerArgs = {};
  timerArgs.callback = timerExpired; timerArgs.dispatch_method = ESP_TIMER_TASK; timerArgs.name = "pulse_stop";
  if (esp_timer_create(&timerArgs, &pulseTimer) != ESP_OK) hardwareOK = false;
  snprintf(bootId, sizeof(bootId), "%08lx%08lx", (unsigned long)esp_random(), (unsigned long)esp_random());
  WiFi.persistent(false); WiFi.mode(WIFI_STA); WiFi.setSleep(false); WiFi.setAutoReconnect(true);
  robotMac = WiFi.macAddress(); loadSettings(); configureRoutes();
  if (savedSSID.length() && savedSSID.length() <= 32) {
    WiFi.begin(savedSSID.c_str(), savedPassword.c_str()); lastWifiRetry = millis();
  }
  rgb.begin(); rgb.clear(); rgb.show();
  lastPowerCheck = millis() - 1000;
  if (hardwareOK && xTaskCreatePinnedToCore(motorControlTask, "wheel_control", 8192,
                                           nullptr, 3, &motorTask, 1) != pdPASS) {
    hardwareOK = false; encoderReady = false;
    if (motorMutex) {
      xSemaphoreTake(motorMutex, portMAX_DELAY); outputsOffLocked(true); xSemaphoreGive(motorMutex);
    }
  }
  if (!hardwareOK) encoderReady = false;
}

void loop() {
  if (!motorMutex) { delay(10); return; }
  const bool stopNow = stopHeld();
  if (emergencyRequested || (stopNow && !previousStopHeld)) {
    stopMotors("button_stop", true);
    xSemaphoreTake(motorMutex, portMAX_DELAY); driverFault = false; xSemaphoreGive(motorMutex);
    emergencyRequested = false; markStatus();
  }
  if (stopNow != previousStopHeld) markStatus();
  previousStopHeld = stopNow;
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  const bool awake = driversAwake, active = moving || navigationActive;
  xSemaphoreGive(motorMutex);
  if (awake && (digitalRead(FAULT_PIN[0]) == LOW || digitalRead(FAULT_PIN[1]) == LOW)) {
    xSemaphoreTake(motorMutex, portMAX_DELAY); driverFault = true; xSemaphoreGive(motorMutex);
    stopMotors("driver_fault", true); markStatus();
  }
  if (!active && uint32_t(millis() - lastPowerCheck) >= 1000) {
    lastPowerCheck = millis();
    xSemaphoreTake(motorMutex, portMAX_DELAY);
    const bool previous = powerInitOK;
    xSemaphoreGive(motorMutex);
    bool nextPower = previous;
    if (!previous && hardwareOK && !stopHeld()) nextPower = configurePower();
    else if (previous) { uint8_t value; nextPower = readRegister(7, value) && !(value & 0xE0); }
    xSemaphoreTake(motorMutex, portMAX_DELAY);
    powerInitOK = nextPower;
    if (!powerInitOK) { stopLocked("power_unavailable", true); esp_timer_stop(pulseTimer); }
    if (powerInitOK != previous) markStatus();
    xSemaphoreGive(motorMutex);
  }
  const wl_status_t wifi = WiFi.status();
  const String ip = WiFi.localIP().toString();
  if (wifi != previousWifi || ip != previousIP) { previousWifi = wifi; previousIP = ip; markStatus(); }
  if (wifi != WL_CONNECTED && udpStarted) {
    stopMotors("wifi_disconnected", true);
    // No anidar motor -> UDP: detener motores libera su mutex primero.
    xSemaphoreTake(udpMutex, portMAX_DELAY);
    navigationUDP.stop(); udpStarted = false;
    xSemaphoreGive(udpMutex);
  }
  if (wifi == WL_CONNECTED && !udpStarted && udpMutex) {
    xSemaphoreTake(udpMutex, portMAX_DELAY);
    udpStarted = navigationUDP.begin(NAV_UDP_PORT) == 1;
    xSemaphoreGive(udpMutex);
  }
  if (wifi == WL_CONNECTED && !serverStarted) { server.begin(); serverStarted = true; }
  if (serverStarted) server.handleClient();
  if (savedSSID.length() && wifi != WL_CONNECTED && uint32_t(millis() - lastWifiRetry) >= 10000) {
    lastWifiRetry = millis(); WiFi.reconnect();
  }
  serviceUSB();
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  const bool canRun = readyLocked(), isMoving = moving;
  const bool fault = driverFault || !hardwareOK || !encoderReady ||
    wheelControl[0].stalled() || wheelControl[1].stalled();
  xSemaphoreGive(motorMutex);
  const uint32_t color = fault ? rgb.Color(30,0,0) : isMoving ? rgb.Color(0,0,35)
    : canRun && wifi == WL_CONNECTED ? rgb.Color(0,25,0) : rgb.Color(18,10,0);
  static uint32_t lastColor = UINT32_MAX;
  if (color != lastColor) { lastColor = color; rgb.setPixelColor(0, color); rgb.show(); }
  delay(1);
}
