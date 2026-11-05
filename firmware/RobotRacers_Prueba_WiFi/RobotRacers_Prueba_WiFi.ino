// Última modificación: 2026-11-05 14:46 | America/Santiago (UTC-03).
// ESP32-S3, Arduino-ESP32 2.0.14. Prueba WiFi: 100/100 durante 1000 ms.
// La duración se cumple en un temporizador local; el encendido nunca mueve motores.
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <freertos/semphr.h>
#include <soc/gpio_struct.h>
#include <atomic>

const uint8_t PWM_PIN[] = {47, 8}, PH_PIN[] = {5, 9};
const uint8_t SLEEP_PIN[] = {6, 10}, FAULT_PIN[] = {7, 11};
const bool INVERTIR[] = {false, true};
const uint8_t STOP_PIN = 48, TURBINA_PIN = 12;
const int PULSE_PWM = 100;
const uint32_t PULSE_MS = 1000;
const size_t MAX_LINE = 512, MAX_ID = 64;

WebServer server(80);
Preferences preferences;
Adafruit_NeoPixel rgb(1, 45, NEO_GRB + NEO_KHZ800);
SemaphoreHandle_t motorMutex = nullptr;
esp_timer_handle_t pulseTimer = nullptr;
std::atomic<bool> statusDirty(true);
volatile bool emergencyRequested = false;
bool moving = false, driversAwake = false, driverFault = false;
bool powerInitOK = false, hardwareOK = false, storageOK = false;
bool serverStarted = false;
int64_t pulseDeadline = 0;
char bootId[17] = {}, token[33] = {}, lastId[MAX_ID + 1] = {};
char stopReason[40] = "boot", recentIds[8][MAX_ID + 1] = {};
uint8_t nextRecent = 0;
String robotMac, savedSSID, savedPassword;
uint32_t lastPowerCheck = 0, lastWifiRetry = 0;
wl_status_t previousWifi = WL_NO_SHIELD;
String previousIP;
char usbLine[MAX_LINE + 1] = {}, usbOutput[768] = {}, usbError[64] = {};
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
  ledcWrite(0, 0); ledcWrite(1, 0);
  if (sleepDrivers) {
    digitalWrite(SLEEP_PIN[0], LOW); digitalWrite(SLEEP_PIN[1], LOW);
    driversAwake = false;
  }
  moving = false;
}

void stopLocked(const char *reason, bool sleepDrivers) {
  outputsOffLocked(sleepDrivers);
  if (strcmp(stopReason, reason) != 0) {
    strlcpy(stopReason, reason, sizeof(stopReason));
    markStatus();
  }
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
  // Un callback ya encolado de un pulso anterior no cancela uno nuevo.
  if (moving && esp_timer_get_time() >= pulseDeadline) {
    stopLocked("completed", false); // PWM cero + nSLEEP alto: freno activo.
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
  replayed = seenId(id);
  if (replayed) { isMoving = moving; xSemaphoreGive(motorMutex); return 200; }
  if (moving || !readyLocked()) {
    isMoving = moving; xSemaphoreGive(motorMutex); return 409;
  }
  outputsOffLocked(true);
  for (int i = 0; i < 2; i++) {
    digitalWrite(PH_PIN[i], (true != INVERTIR[i]) ? HIGH : LOW);
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
  pulseDeadline = esp_timer_get_time() + int64_t(PULSE_MS) * 1000;
  ledcWrite(0, PULSE_PWM); ledcWrite(1, PULSE_PWM);
  moving = true;
  const int64_t remaining = pulseDeadline - esp_timer_get_time();
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

bool authorized() {
  const String supplied = server.header("X-Robot-Token");
  uint8_t difference = supplied.length() == 32 ? 0 : 1;
  for (size_t i = 0; i < 32; i++) difference |= token[i] ^ (i < supplied.length() ? supplied[i] : 0);
  return difference == 0;
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
  const bool isMoving = moving;
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
  hardwareOK = motorMutex != nullptr;
  for (int i = 0; i < 2; i++) {
    pinMode(SLEEP_PIN[i], OUTPUT); digitalWrite(SLEEP_PIN[i], LOW);
    pinMode(PWM_PIN[i], OUTPUT); digitalWrite(PWM_PIN[i], LOW);
    pinMode(PH_PIN[i], OUTPUT); digitalWrite(PH_PIN[i], LOW);
    pinMode(FAULT_PIN[i], INPUT_PULLUP);
    if (ledcSetup(i, 20000, 8) == 0) hardwareOK = false;
    ledcAttachPin(PWM_PIN[i], i); ledcWrite(i, 0);
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
  WiFi.persistent(false); WiFi.mode(WIFI_STA); WiFi.setAutoReconnect(true);
  robotMac = WiFi.macAddress(); loadSettings(); configureRoutes();
  if (savedSSID.length() && savedSSID.length() <= 32) {
    WiFi.begin(savedSSID.c_str(), savedPassword.c_str()); lastWifiRetry = millis();
  }
  rgb.begin(); rgb.clear(); rgb.show();
  lastPowerCheck = millis() - 1000;
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
  const bool awake = driversAwake, active = moving;
  xSemaphoreGive(motorMutex);
  if (awake && (digitalRead(FAULT_PIN[0]) == LOW || digitalRead(FAULT_PIN[1]) == LOW)) {
    xSemaphoreTake(motorMutex, portMAX_DELAY); driverFault = true; xSemaphoreGive(motorMutex);
    stopMotors("driver_fault", true); markStatus();
  }
  if (!active && uint32_t(millis() - lastPowerCheck) >= 1000) {
    lastPowerCheck = millis();
    const bool previous = powerInitOK;
    if (!powerInitOK && hardwareOK && !stopHeld()) powerInitOK = configurePower();
    else if (powerInitOK) { uint8_t value; powerInitOK = readRegister(7, value) && !(value & 0xE0); }
    if (!powerInitOK) stopMotors("power_unavailable", true);
    if (powerInitOK != previous) markStatus();
  }
  const wl_status_t wifi = WiFi.status();
  const String ip = WiFi.localIP().toString();
  if (wifi != previousWifi || ip != previousIP) { previousWifi = wifi; previousIP = ip; markStatus(); }
  if (wifi == WL_CONNECTED && !serverStarted) { server.begin(); serverStarted = true; }
  if (serverStarted) server.handleClient();
  if (savedSSID.length() && wifi != WL_CONNECTED && uint32_t(millis() - lastWifiRetry) >= 10000) {
    lastWifiRetry = millis(); WiFi.reconnect();
  }
  serviceUSB();
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  const bool canRun = readyLocked(), isMoving = moving, fault = driverFault || !hardwareOK;
  xSemaphoreGive(motorMutex);
  const uint32_t color = fault ? rgb.Color(30,0,0) : isMoving ? rgb.Color(0,0,35)
    : canRun && wifi == WL_CONNECTED ? rgb.Color(0,25,0) : rgb.Color(18,10,0);
  static uint32_t lastColor = UINT32_MAX;
  if (color != lastColor) { lastColor = color; rgb.setPixelColor(0, color); rgb.show(); }
  delay(1);
}
