// Ultima modificacion: 2026-10-28 22:15:18 | America/Santiago (UTC-03).
#include <Arduino.h>
#include <Wire.h>
#include <driver/pcnt.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// REV3.11: D44 espera 1 segundo; 4 avances por tiempo + 4 giros izquierdos de 90 grados.
// AVANCE validado REV3.6; GIRO continuo validado REV3.10. Al finalizar se detiene.
// D44 espera 1 segundo y luego inicia una vez. D48 detiene/rearma. No arranca al encender.
// ESP32-S3 / Arduino-ESP32 2.0.14 / hardware fabricado Pololu #5178.
// Giro continuo: arranque gradual, meseta hasta 40 grados y descenso suave hasta 90.
// Perfil por angulo y limites de aceleracion: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html
const uint32_t TIEMPO_AVANCE_MS = 2000, RAMPA_AVANCE_US = 500000UL;
const int PWM_AVANCE = 110, PWM_ARRANQUE_AVANCE = 160;
const uint32_t IMPULSO_AVANCE_US = 100000UL;
const float CORRECCION_FRENO = 12.0f; // PWM/mm al desacelerar la recta.
const float VELOCIDAD_GIRO = 100.0f, ACELERACION_GIRO = 150.0f; // mm/s y mm/s^2 por rueda.
const float VELOCIDAD_FINAL = 35.0f, INICIO_DESCENSO_GRADOS = 40.0f;
const float KP_VELOCIDAD = 0.10f, KI_VELOCIDAD = 1.0f, K_SINCRONIA = 2.0f;
const float FF_ROZAMIENTO = 35.0f, FF_VELOCIDAD = 0.125f;
const int PWM_MAXIMO = 160;
const float PWM_INICIAL = 35.0f, SUBIDA_ARRANQUE = 300.0f;
const float TRANSFERENCIA_ARRANQUE = 0.40f;
const uint32_t CONTROL_US = 10000UL;
const float FILTRO_VEL_S = 0.024f; // Suaviza la variacion entre cuentas del encoder.
const float ANTICIPO_FRENO_S = 0.080f, ANTICIPO_MAX_MM = 2.0f;
const uint32_t ESPERA_INICIO_US = 1000000UL, ATASCO_US = 5000000UL;
const uint32_t LIMITE_GIRO_US = 30000000UL;
const uint32_t FRENO_US = 800000UL, PAUSA_US = 300000UL;
const uint32_t QUIETUD_US = 150000UL, REPOSO_MAX_US = 5000000UL;
const float DIAMETRO_MM[] = {23.0f, 23.0f};
const float SEPARACION_MM = 190.0f; // 230 mm exterior menos 40 mm de goma.
const float CUENTAS_VUELTA = 12.0f * 4.995f; // 59.94; ya incluye cuadratura x4.
const float ANGULO_GIRO = 90.0f;
const uint8_t PWM[] = {47, 8}, PH[] = {5, 9}, SLEEP[] = {6, 10}, FAULT[] = {7, 11};
const uint8_t ENC_A[] = {15, 17}, ENC_B[] = {16, 18};
const bool INVERTIR_MOTOR[] = {false, true}; // Mismo sentido que la prueba funcional.
const int8_t SIGNO_ENCODER[] = {-1, 1}; // Polaridades verificadas por USB en este robot.

Adafruit_NeoPixel rgb(1, 45, NEO_GRB + NEO_KHZ800);
enum Estado { PARADO, AVANCE, GIRO, FRENO, PAUSA, ESPERA };
Estado estado = PARADO;
uint8_t paso = 0; // 0 recta, 1 giro, ... 6 recta, 7 giro final.
int16_t anteriorPCNT[2] = {0, 0};
int32_t cuentas[2] = {0, 0}, base[2] = {0, 0}, cuentaRespuesta[2] = {0, 0};
int32_t muestraVel[2] = {0, 0}, referenciaQuietud[2] = {0, 0};
int8_t signoEncoder[2] = {0, 0};
float mmCuenta[2], recorrido[2] = {0, 0}, objetivoMM[2];
float velocidad[2] = {0, 0}, velocidadEvento[2] = {0, 0}, integral[2] = {0, 0}, consigna = 0;
uint32_t tiempoCuenta[2] = {0, 0};
bool desacelerando = false, ruedaFrenada[2] = {false, false};
float inicioDesaceleracion[2] = {0, 0};
int32_t maxProgreso[2] = {0, 0}, progresoImpulso[2] = {0, 0};
uint32_t inicioImpulso[2] = {0, 0};
float pwmArranque[2] = {PWM_INICIAL, PWM_INICIAL};
bool arrancado[2] = {false, false}, llegada[2] = {false, false};
uint32_t inicioTramo = 0, cambioEstado = 0, ultimoControl = 0, ultimoProgreso[2] = {0, 0};
uint32_t ultimoPI = 0, ultimoCambioEncoder = 0;
float anguloAlCorte = 0, velocidadAlCorte[2] = {0, 0};
std::atomic<uint32_t> colorPedido(0);
bool error = false, falloInicial = false, serialAnterior = false;
bool botonAnterior = HIGH, botonEstable = HIGH;
uint32_t cambioBoton = 0;
const char *ultimoMensaje = "REV3.11 USB lista: D44 inicia; D48 detiene.";

// Informe persistente: se conserva al parar y se puede repetir enviando D por USB.
char informe[1024] = {};
size_t envioInforme = 0;
uint32_t ultimoLog = 0;
struct CapturaError {
  Estado fase; uint8_t paso; uint32_t tiempo, sinProgreso[2];
  int fallo[2], sleep[2], ph[2], pwm[2];
  int16_t raw[2]; int32_t cuentas[2], base[2]; int8_t signo[2];
  float mm[2];
} captura;

void mensaje(const char *texto);

const char *nombreEstado(Estado e) {
  const char *n[] = {"PARADO", "AVANCE", "GIRO", "FRENO", "PAUSA", "ESPERA"};
  return n[unsigned(e)];
}

void capturarError() {
  // Capturar GPIO y PWM ANTES de dormir los drivers; despues nFAULT puede cambiar.
  captura.fase = estado; captura.paso = paso; captura.tiempo = uint32_t(micros() - inicioTramo);
  for (int i = 0; i < 2; i++) {
    captura.fallo[i] = digitalRead(FAULT[i]); captura.sleep[i] = digitalRead(SLEEP[i]);
    captura.ph[i] = digitalRead(PH[i]); captura.pwm[i] = ledcRead(i);
    captura.raw[i] = anteriorPCNT[i]; captura.cuentas[i] = cuentas[i]; captura.base[i] = base[i];
    captura.signo[i] = signoEncoder[i]; captura.mm[i] = recorrido[i];
    captura.sinProgreso[i] = uint32_t(micros() - ultimoProgreso[i]) / 1000;
  }
}

void prepararInforme(const char *motivo) {
  // Formatear solo DESPUES de apagar motores. No hacer I2C antes de una parada.
  const float angulo = (captura.paso % 2 ? captura.mm[0] + captura.mm[1] : captura.mm[1] - captura.mm[0])
                        * 180.0f / (PI * SEPARACION_MM);
  snprintf(informe, sizeof(informe),
    "\n=== DIAGNOSTICO REV3.11 ===\nMotivo: %s\nFase=%s paso=%u/8 tiempo=%lu ms\n"
    "nFAULT L/R=%d/%d (0=falla driver)\nSLEEP L/R=%d/%d PH L/R=%d/%d\n"
    "PWM L/R=%d/%d\nPCNT raw L/R=%d/%d\nCuentas L/R=%ld/%ld base=%ld/%ld\n"
    "Signos encoder L/R=%d/%d\nRecorrido normalizado L/R=%.2f/%.2f mm\n"
    "Diferencia L-R=%.2f mm (solo informativa)\nAngulo odometrico de fase=%.2f grados\n"
    "Objetivo angular=%.2f; error=%.2f grados (encoder)\n"
    "Sin progreso L/R=%lu/%lu ms (limite=5000)\n"
    "Angulo al corte=%.2f grados; velocidad de GIRO al corte L/R=%.1f/%.1f mm/s\n"
    "D=repetir informe. D48=rearmar. D44=nuevo intento manual.\n=== FIN DIAGNOSTICO ===\n",
    motivo, nombreEstado(captura.fase), unsigned(captura.paso+1), (unsigned long)(captura.tiempo/1000),
    captura.fallo[0], captura.fallo[1], captura.sleep[0], captura.sleep[1], captura.ph[0], captura.ph[1],
    captura.pwm[0], captura.pwm[1], captura.raw[0], captura.raw[1],
    (long)captura.cuentas[0], (long)captura.cuentas[1], (long)captura.base[0], (long)captura.base[1],
    captura.signo[0], captura.signo[1], captura.mm[0], captura.mm[1], captura.mm[0]-captura.mm[1],
    angulo, captura.paso%2 ? ANGULO_GIRO : 0.0f, angulo-(captura.paso%2 ? ANGULO_GIRO : 0.0f), (unsigned long)captura.sinProgreso[0], (unsigned long)captura.sinProgreso[1],
    anguloAlCorte, velocidadAlCorte[0], velocidadAlCorte[1]);
  envioInforme = 0;
}

void atenderUSB() {
  // No se reciben ordenes de movimiento por serie: solo D para ver el ultimo fallo.
  for (int n = 0; n < 16 && Serial.available(); n++) {
    const int c = Serial.read();
    if (c == 'D' || c == 'd' || c == '?') {
      if (informe[0]) envioInforme = 0;
      else mensaje("Todavia no hay un error guardado.");
    }
  }
  if (!Serial) return;
  if (!serialAnterior) {
    mensaje(ultimoMensaje);
    if (informe[0]) envioInforme = 0;
  }
  // Fragmentos de hasta 48 bytes: funcionan incluso con el buffer TinyUSB de 64.
  // Solo se envia el informe completo con motores detenidos; nunca bloquea esperando espacio.
  const size_t largo = strlen(informe);
  if (estado == PARADO && envioInforme < largo) {
    const size_t n = (largo - envioInforme > 48) ? 48 : largo - envioInforme;
    if (Serial.availableForWrite() >= int(n))
      envioInforme += Serial.write(reinterpret_cast<const uint8_t *>(informe + envioInforme), n);
  }
  if ((estado == AVANCE || estado == GIRO || estado == FRENO || estado == PAUSA) && uint32_t(millis() - ultimoLog) >= 100) {
    ultimoLog = millis();
    const float a = (paso%2 ? recorrido[0]+recorrido[1] : recorrido[1]-recorrido[0])*180.0f/(PI*SEPARACION_MM);
    char l[64]; const int n = snprintf(l,sizeof(l),"%s t=%lu L=%.0f R=%.0f A=%.1f P=%u/%u F=%d/%d\n",
      estado == GIRO ? "G" : estado == AVANCE ? "A" : "F",(unsigned long)(uint32_t(micros()-inicioTramo)/1000),
      recorrido[0],recorrido[1],a,unsigned(ledcRead(0)),unsigned(ledcRead(1)),
      digitalRead(FAULT[0]),digitalRead(FAULT[1]));
    if (n>0 && n<int(sizeof(l)) && Serial.availableForWrite()>=n)
      Serial.write(reinterpret_cast<const uint8_t *>(l),n);
  }
}

void mensaje(const char *texto) {
  ultimoMensaje = texto;
  if (!Serial) return;
  char linea[64]; int n = snprintf(linea, sizeof(linea), "%s\n", texto);
  if (n > 63) { n = 63; linea[62] = '\n'; }
  if (n > 0 && Serial.availableForWrite() >= n)
    Serial.write(reinterpret_cast<const uint8_t *>(linea), n);
}

void actualizarRGB() {
  const uint32_t color = error ? rgb.Color(40, 0, 0) : estado == PARADO ? rgb.Color(0, 30, 0)
    : rgb.Color(0, 0, (millis() / 250) % 2 ? 50 : 3);
  colorPedido.store(color); // Solo comunicar el color: nunca esperar al RMT en el control.
}

void tareaRGB(void *) {
  uint32_t anterior = 0xFFFFFFFF;
  for (;;) {
    const uint32_t color = colorPedido.load();
    if (color != anterior) { rgb.setPixelColor(0, color); rgb.show(); anterior = color; }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void apagarMotores() {
  for (int i = 0; i < 2; i++) { ledcWrite(i, 0); digitalWrite(SLEEP[i], LOW); }
}

void detener(const char *motivo, bool fallo) {
  if (fallo) capturarError();
  apagarMotores(); estado = PARADO; error = fallo;
  if (fallo) prepararInforme(motivo);
  mensaje(motivo);
}

bool leerRegistro(uint8_t reg, uint8_t &valor) {
  Wire.beginTransmission(0x74); Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(uint8_t(0x74), uint8_t(1)) != 1) return false;
  valor = Wire.read(); return true;
}

bool escribirRegistro(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(0x74); Wire.write(reg); Wire.write(valor);
  if (Wire.endTransmission() != 0) return false;
  uint8_t recibido; return leerRegistro(reg, recibido) && recibido == valor;
}

bool alimentarMotores() {
  // U11: salida 6 V, limite de corriente y lectura de confirmacion.
  const uint8_t r[][2] = {{6,0x22},{4,0x83},{0,0xF7},{1,0x07},{2,0xE4},{3,0x01},{5,0xE0},{6,0xA2}};
  for (const auto &x : r) {
    if (!escribirRegistro(x[0], x[1])) { escribirRegistro(6, 0x22); return false; }
  }
  delay(30);
  uint8_t valor;
  if (!leerRegistro(7, valor) || (valor & 0xE0)) { escribirRegistro(6, 0x22); return false; }
  return true;
}

bool configurarEncoder(int i) {
  const pcnt_unit_t u = static_cast<pcnt_unit_t>(i);
  pcnt_config_t c = {};
  c.unit = u; c.counter_h_lim = 30000; c.counter_l_lim = -30000;
  c.lctrl_mode = PCNT_MODE_KEEP; c.hctrl_mode = PCNT_MODE_REVERSE;
  c.pulse_gpio_num = ENC_A[i]; c.ctrl_gpio_num = ENC_B[i]; c.channel = PCNT_CHANNEL_0;
  c.pos_mode = PCNT_COUNT_INC; c.neg_mode = PCNT_COUNT_DEC;
  if (pcnt_unit_config(&c) != ESP_OK) return false;
  c.pulse_gpio_num = ENC_B[i]; c.ctrl_gpio_num = ENC_A[i]; c.channel = PCNT_CHANNEL_1;
  c.pos_mode = PCNT_COUNT_DEC; c.neg_mode = PCNT_COUNT_INC;
  if (pcnt_unit_config(&c) != ESP_OK) return false;
  return pcnt_counter_pause(u) == ESP_OK && pcnt_intr_disable(u) == ESP_OK &&
         pcnt_set_filter_value(u, 80) == ESP_OK && pcnt_filter_enable(u) == ESP_OK &&
         pcnt_counter_clear(u) == ESP_OK && pcnt_counter_resume(u) == ESP_OK;
}

bool leerEncoders(float dt, uint32_t ahora) {
  for (int i = 0; i < 2; i++) {
    int16_t actual;
    if (pcnt_get_counter_value(static_cast<pcnt_unit_t>(i), &actual) != ESP_OK) return false;
    int delta = int(actual) - anteriorPCNT[i];
    if (delta < -15000) delta += 30000;
    if (delta > 15000) delta -= 30000; // PCNT reinicia en cero en +/-30000.
    anteriorPCNT[i] = actual;
    if (abs(delta) > 100 + 16000 * dt) return false;
    cuentas[i] += delta;
    const int32_t relativa = cuentas[i] - base[i];
    const int32_t fisica = signoEncoder[i] ? signoEncoder[i] * relativa : abs(relativa);
    const int32_t progreso = (paso % 2 && i == 0) ? -fisica : fisica;
    recorrido[i] = progreso * mmCuenta[i];
    if (progreso >= maxProgreso[i] + 2) maxProgreso[i] = progreso;
    if (progreso >= cuentaRespuesta[i] + 1) { cuentaRespuesta[i] = progreso; ultimoProgreso[i] = ahora; }
  }
  return true;
}

bool nuevaPulsacion() {
  const bool b = digitalRead(44);
  if (b != botonAnterior) { botonAnterior = b; cambioBoton = millis(); }
  if (b != botonEstable && uint32_t(millis() - cambioBoton) >= 30) {
    botonEstable = b; return b == LOW;
  }
  return false;
}

int32_t posicionRueda(int i) {
  return (paso%2 && i == 0 ? -1 : 1)*signoEncoder[i]*(cuentas[i]-base[i]);
}

float anguloActual() { return (paso%2 ? recorrido[0]+recorrido[1] : recorrido[1]-recorrido[0])*180.0f/(PI*SEPARACION_MM); }

float velocidadPerfil(float angulo) {
  // Meseta hasta 40 grados; descenso continuo sin escalones de consigna.
  // La curva tiene pendiente cero en ambos extremos para evitar cambios bruscos.
  const float u = constrain((angulo-INICIO_DESCENSO_GRADOS) /
    (ANGULO_GIRO-INICIO_DESCENSO_GRADOS), 0.0f, 1.0f);
  const float suave = u*u*(3.0f-2.0f*u);
  return VELOCIDAD_GIRO+(VELOCIDAD_FINAL-VELOCIDAD_GIRO)*suave;
}

void comenzarTramo() {
  const bool giro = paso%2;
  apagarMotores(); consigna = 0; anguloAlCorte = 0; desacelerando = false;
  for (int i = 0; i < 2; i++) {
    base[i] = cuentas[i]; cuentaRespuesta[i] = muestraVel[i] = maxProgreso[i] = 0; recorrido[i] = 0;
    ruedaFrenada[i] = false; inicioDesaceleracion[i] = 0; progresoImpulso[i] = 0;
    velocidadAlCorte[i] = 0;
    velocidad[i] = velocidadEvento[i] = integral[i] = 0; arrancado[i] = llegada[i] = false;
    pwmArranque[i] = PWM_INICIAL;
    const bool adelante = !(giro && i == 0);
    digitalWrite(PH[i], (adelante != INVERTIR_MOTOR[i]) ? HIGH : LOW);
    digitalWrite(SLEEP[i], HIGH);
  }
  delay(3);
  if (digitalRead(48) == LOW) { detener("Parada D48.", false); return; }
  if (digitalRead(FAULT[0]) == LOW || digitalRead(FAULT[1]) == LOW) {
    detener("ERROR nFAULT: revisar motor/alimentacion.", true); return;
  }
  mensaje(giro ? "GIRO: velocidad estable hasta 40 grados, luego descenso suave."
               : "AVANCE: 2 segundos; encoders corrigen y frenan gradual.");
  estado = giro ? GIRO : AVANCE;
  inicioTramo = ultimoControl = ultimoPI = micros();
  ultimoProgreso[0] = ultimoProgreso[1] = inicioTramo;
  tiempoCuenta[0] = tiempoCuenta[1] = inicioTramo;
  for (int i = 0; i < 2; i++) {
    inicioImpulso[i] = inicioTramo;
    if (!giro) ledcWrite(i, PWM_ARRANQUE_AVANCE);
  }
}

void iniciar() {
  apagarMotores(); error = false;
  if (!alimentarMotores()) { detener("ERROR U11: revisar bateria e interruptor.", true); return; }
  for (int i = 0; i < 2; i++) {
    const pcnt_unit_t u = static_cast<pcnt_unit_t>(i);
    if (pcnt_counter_pause(u) != ESP_OK || pcnt_counter_clear(u) != ESP_OK || pcnt_counter_resume(u) != ESP_OK) {
      detener("ERROR: no se pudo reiniciar PCNT.", true); return;
    }
    cuentas[i] = base[i] = anteriorPCNT[i] = 0; signoEncoder[i] = SIGNO_ENCODER[i];
  }
  paso = 0; comenzarTramo();
}

void programarInicio() {
  apagarMotores(); error = false; estado = ESPERA;
  cambioEstado = ultimoControl = inicioTramo = micros();
  mensaje("ESPERA: inicia en 1 segundo; D48 cancela.");
}

void terminarTramo(uint32_t ahora) {
  anguloAlCorte = anguloActual();
  for (int i = 0; i < 2; i++) {
    velocidadAlCorte[i] = paso%2 ? velocidad[i] : 0; integral[i] = 0;
    ledcWrite(i, 0); referenciaQuietud[i] = posicionRueda(i);
  }
  estado = FRENO; cambioEstado = ultimoCambioEncoder = ahora;
  mensaje("FRENO: comprobando que el robot quede quieto.");
}

// Avance de REV3.6: misma duracion, PWM, sincronizacion y rampa validadas.
void actualizarAvance(uint32_t ahora) {
  const uint32_t duracion = TIEMPO_AVANCE_MS*1000UL;
  const uint32_t transcurrido = uint32_t(ahora-inicioTramo);
  if (transcurrido >= duracion) { terminarTramo(ahora); return; }
  const uint32_t rampa = duracion < RAMPA_AVANCE_US ? duracion : RAMPA_AVANCE_US;
  if (!desacelerando && duracion-transcurrido <= rampa) {
    desacelerando = true;
    inicioDesaceleracion[0] = recorrido[0]; inicioDesaceleracion[1] = recorrido[1];
  }
  if (desacelerando) {
    const int limite = int(PWM_AVANCE*float(duracion-transcurrido)/rampa);
    const float adicionales[2] = {recorrido[0]-inicioDesaceleracion[0], recorrido[1]-inicioDesaceleracion[1]};
    for (int i = 0; i < 2; i++) {
      const float adelanto = adicionales[i]-adicionales[1-i];
      const int reduccion = adelanto > 0 ? int(CORRECCION_FRENO*adelanto) : 0;
      const int orden = constrain(limite-reduccion, 0, limite);
      if (orden > 0 && ledcRead(i) == 0) ultimoProgreso[i] = ahora;
      if (orden > 0 && uint32_t(ahora-ultimoProgreso[i]) >= ATASCO_US) {
        detener(i == 0 ? "ERROR: izquierdo sin progreso durante 5 segundos." : "ERROR: derecho sin progreso durante 5 segundos.", true); return;
      }
      ledcWrite(i, orden);
    }
    return;
  }
  const float cuentaMedia = 0.5f*(mmCuenta[0]+mmCuenta[1]);
  for (int i = 0; i < 2; i++) {
    const float adelanto = recorrido[i]-recorrido[1-i];
    if (!ruedaFrenada[i] && adelanto >= 6*cuentaMedia) ruedaFrenada[i] = true;
    else if (ruedaFrenada[i] && adelanto <= 3*cuentaMedia) ruedaFrenada[i] = false;
    const bool reanuda = !ruedaFrenada[i] && ledcRead(i) == 0;
    if (reanuda) {
      ultimoProgreso[i] = inicioImpulso[i] = ahora;
      progresoImpulso[i] = maxProgreso[i];
    }
    const bool impulso = uint32_t(ahora-inicioImpulso[i]) < IMPULSO_AVANCE_US &&
                         maxProgreso[i]-progresoImpulso[i] < 4;
    const int orden = ruedaFrenada[i] ? 0 : impulso ? PWM_ARRANQUE_AVANCE : PWM_AVANCE;
    if (orden > 0 && uint32_t(ahora-ultimoProgreso[i]) >= ATASCO_US) {
      detener(i == 0 ? "ERROR: izquierdo sin progreso durante 5 segundos." : "ERROR: derecho sin progreso durante 5 segundos.", true); return;
    }
    ledcWrite(i, orden);
  }
}

void actualizar(uint32_t ahora) {
  const float dtLectura = uint32_t(ahora-ultimoControl)*1e-6f; ultimoControl = ahora;
  if (dtLectura > 0.25f) { detener("ERROR: pausa de control mayor a 250 ms.", true); return; }
  if (estado == ESPERA) {
    if (uint32_t(ahora-cambioEstado) >= ESPERA_INICIO_US) iniciar();
    return;
  }
  if (!leerEncoders(dtLectura, ahora)) { detener("ERROR: lectura de encoder imposible.", true); return; }
  if (estado == FRENO || estado == PAUSA) {
    bool cambio = false;
    for (int i = 0; i < 2; i++) if (abs(posicionRueda(i)-referenciaQuietud[i]) > 1) cambio = true;
    if (cambio) {
      ultimoCambioEncoder = ahora;
      for (int i = 0; i < 2; i++) referenciaQuietud[i] = posicionRueda(i);
    }
    const bool quieto = uint32_t(ahora-ultimoCambioEncoder) >= QUIETUD_US;
    if (estado == FRENO && uint32_t(ahora-cambioEstado) >= FRENO_US && quieto) {
      estado = PAUSA; cambioEstado = ahora;
    } else if (estado == PAUSA && uint32_t(ahora-cambioEstado) >= PAUSA_US && quieto) {
      if (paso == 7) {
        capturarError(); apagarMotores(); estado = PARADO; error = false;
        prepararInforme("FIN: cuatro rectas y cuatro giros completos; robot quieto.");
        mensaje("FIN: cuatro rectas y cuatro giros completos.");
      } else { paso++; comenzarTramo(); }
    } else if (uint32_t(ahora-cambioEstado) >= REPOSO_MAX_US) {
      detener("ERROR: movimiento despues de frenar durante 5 segundos.", true);
    }
    return;
  }
  if (estado == AVANCE) { actualizarAvance(ahora); return; }
  if (uint32_t(ahora-inicioTramo) >= LIMITE_GIRO_US) {
    detener("ERROR: giro sin terminar en 30 segundos.", true); return;
  }
  // Estimar velocidad por el tiempo entre cambios del contador. Responde pronto
  // si acelera mucho y conserva resolucion a baja velocidad sin pulsos artificiales.
  for (int i = 0; i < 2; i++) {
    const int32_t posicion = posicionRueda(i), delta = posicion-muestraVel[i];
    if (delta) {
      const float dtCuenta = uint32_t(ahora-tiempoCuenta[i])*1e-6f;
      const float medida = delta*mmCuenta[i]/fmaxf(1e-6f, arrancado[i] ? dtCuenta : fminf(0.04f,dtCuenta));
      const float alfa = dtCuenta/(FILTRO_VEL_S+dtCuenta);
      velocidadEvento[i] += alfa*(medida-velocidadEvento[i]);
      muestraVel[i] = posicion; tiempoCuenta[i] = ahora;
    }
    const float sinCuenta = uint32_t(ahora-tiempoCuenta[i])*1e-6f;
    const float limiteSinCuenta = sinCuenta > 0 ? mmCuenta[i]/sinCuenta : 1e6f;
    velocidad[i] = fminf(fmaxf(0,velocidadEvento[i]),limiteSinCuenta);
  }
  // Anticipacion pequena y acotada al final, mientras se avanza ya muy despacio.
  for (int i = 0; i < 2; i++) {
    const float anticipo = fminf(ANTICIPO_MAX_MM, fmaxf(0,velocidad[i])*ANTICIPO_FRENO_S);
    if (!llegada[i] && (recorrido[i] >= objetivoMM[i] ||
        (velocidad[i] <= 55 && recorrido[i] >= objetivoMM[i]-anticipo))) {
      llegada[i] = true; ledcWrite(i, 0); integral[i] = 0;
    }
    if (!llegada[i] && uint32_t(ahora-ultimoProgreso[i]) >= ATASCO_US) {
      detener(i == 0 ? "ERROR: izquierdo sin avance durante 5 segundos." : "ERROR: derecho sin avance durante 5 segundos.", true); return;
    }
  }
  if (llegada[0] && llegada[1]) { terminarTramo(ahora); return; }
  if (uint32_t(ahora-ultimoPI) < CONTROL_US) return;
  const float dt = uint32_t(ahora-ultimoPI)*1e-6f; ultimoPI = ahora;
  const float deseada = velocidadPerfil(anguloActual());
  consigna += constrain(deseada-consigna, -ACELERACION_GIRO*dt, ACELERACION_GIRO*dt);
  for (int i = 0; i < 2; i++) {
    if (llegada[i]) { ledcWrite(i, 0); continue; }
    const float correccion = llegada[1-i] ? 0 : K_SINCRONIA*(recorrido[1-i]-recorrido[i]);
    const float referencia = constrain(consigna+correccion, 0.0f, VELOCIDAD_GIRO);
    const float errorVel = referencia-velocidad[i];
    const float ff = FF_ROZAMIENTO+FF_VELOCIDAD*referencia;
    if (!arrancado[i]) {
      if (posicionRueda(i) >= 1) {
        arrancado[i] = true;
        integral[i] = constrain(TRANSFERENCIA_ARRANQUE*(pwmArranque[i]-ff-KP_VELOCIDAD*errorVel), -120.0f, 125.0f);
      } else {
        pwmArranque[i] = fminf(PWM_MAXIMO, pwmArranque[i]+SUBIDA_ARRANQUE*dt);
        ledcWrite(i, int(pwmArranque[i])); continue;
      }
    }
    const float libre = ff+KP_VELOCIDAD*errorVel+integral[i];
    if ((libre > 0 && libre < PWM_MAXIMO) || (libre >= PWM_MAXIMO && errorVel < 0) || (libre <= 0 && errorVel > 0))
      integral[i] = constrain(integral[i]+KI_VELOCIDAD*errorVel*dt, -120.0f, 125.0f);
    const int orden = constrain(int(lroundf(ff+KP_VELOCIDAD*errorVel+integral[i])), 0, PWM_MAXIMO);
    ledcWrite(i, orden); // PWM continuo; no hay ciclos de impulso/freno ni cambios de PH.
  }
}

void setup() {
  pinMode(12, OUTPUT); digitalWrite(12, LOW); // Turbina apagada.
  pinMode(44, INPUT_PULLUP); pinMode(48, INPUT_PULLUP);
  botonAnterior = botonEstable = digitalRead(44);
  for (int i = 0; i < 2; i++) {
    pinMode(SLEEP[i], OUTPUT); digitalWrite(SLEEP[i], LOW);
    pinMode(PWM[i], OUTPUT); digitalWrite(PWM[i], LOW);
    pinMode(PH[i], OUTPUT); digitalWrite(PH[i], LOW);
    pinMode(FAULT[i], INPUT_PULLUP);
    mmCuenta[i] = PI * DIAMETRO_MM[i] / CUENTAS_VUELTA;
    objetivoMM[i] = PI * SEPARACION_MM * ANGULO_GIRO / 360.0f;
    if (ledcSetup(i, 20000, 8) == 0 || !configurarEncoder(i)) falloInicial = true;
    ledcAttachPin(PWM[i], i); ledcWrite(i, 0);
  }
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif
  Wire.setTimeOut(20);
  if (!Wire.begin(41, 40, 100000)) falloInicial = true;
  rgb.begin(); error = falloInicial; actualizarRGB();
  if (xTaskCreatePinnedToCore(tareaRGB, "rgb", 3072, nullptr, 1, nullptr, 0) != pdPASS) {
    falloInicial = error = true;
    rgb.setPixelColor(0, rgb.Color(40,0,0)); rgb.show();
  }
  mensaje(falloInicial ? "ERROR de inicializacion: revisar y RESET." : "REV3.11 USB: D44 inicia; D48 detiene; D muestra el fallo.");
}

void loop() {
  const bool pulsacion = nuevaPulsacion();
  if (digitalRead(48) == LOW) {
    if (estado != PARADO || error) detener("Parada/rearme D48; soltar y pulsar D44.", falloInicial);
  } else if (estado == PARADO && pulsacion) {
    if (falloInicial) mensaje("ERROR de inicializacion: requiere RESET.");
    else programarInicio(); // Siempre esperar 1 s; un error permite nuevo intento manual.
  }
  if ((estado == AVANCE || estado == GIRO || estado == FRENO || estado == PAUSA) &&
      (digitalRead(FAULT[0]) == LOW || digitalRead(FAULT[1]) == LOW))
    detener("ERROR nFAULT: revisar motor/alimentacion.", true);
  const uint32_t ahora = micros();
  if (estado != PARADO && uint32_t(ahora-ultimoControl) >= (estado == AVANCE ? 2000UL : 1000UL)) actualizar(ahora);
  actualizarRGB(); // Publicar el color no espera al RMT; otra tarea mueve el LED.
  if (estado != GIRO && estado != AVANCE) {
    const bool conectado = bool(Serial);
    atenderUSB(); serialAnterior = conectado;
  }
  delay(1);
}
