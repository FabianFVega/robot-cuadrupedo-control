/*
  ============================================================================
  ESP8266 PRINCIPAL -- PID de las 4 rodillas (lazo cerrado real) -- VERSION 2
  Robot Cuadrupedo (ACEBOTT/QD020)
  ============================================================================

  Misma arquitectura que la version anterior:
    - theta1..4 llegan por Serial de HARDWARE desde la ESP-Sensores (AS5600)
    - setpoint y telemetria por WiFi hacia/desde Python en la PC
    - mismo mapeo de pines, misma red, mismo formato de telemetria (17 campos)

  Cambios respecto a la version anterior (ver el resumen en el chat):
    1. Feedforward: el comando parte de la referencia convertida a write()
       con la calibracion de cada pata, no de un WRITE_BASE fijo en 90.
       Antes, cualquier referencia lejos de 275 grados dejaba error permanente.
    2. Modelo de referencia: el PID compara theta contra "donde deberia ir la
       pata si el servo respondiera como el nominal" (wn = 30 rad/s, zeta = 1),
       no contra el setpoint final. Asi no pelea contra el retardo normal del
       servo y no se pasa, sea el servo rigido o blando.
    3. PID discreto (Tustin) con Ts fijo de 20 ms: P e I sobre el error,
       D sobre la medicion y filtrada (sin golpe al cambiar la referencia).
    4. Anti-windup por integracion condicional (no integra si write satura).
    5. Lectura del Serial NO bloqueante (antes readStringUntil podia frenar
       el lazo hasta 1 s esperando el fin de linea).
    6. Calibracion por pata (theta en write=0 y grados por unidad de write,
       negativo si el servo esta montado en espejo).
    7. Ganancias y modo ajustables por WiFi sin recompilar.

  !! IMPORTANTE: BAUD_SENSORES = 115200. La ESP-Sensores debe usar la MISMA
     velocidad (Serial.begin(115200)) y enviar una linea cada 20 ms o menos.
     A 9600 baudios una linea de ~60 caracteres tarda ~60 ms: el PID recibia
     un dato nuevo cada 3 ciclos.

  Red WiFi (Access Point):  SSID RobotPID, clave 12345678, IP 192.168.4.1:80

  Protocolo Python -> ESP (una linea por comando, termina en \n):
      ref1=250.0 ... ref4=250.0   setpoint de una pata (FI, FD, TI, TD)
      ref=250.0                   las 4 patas al mismo valor
      kp=0.8   ki=12   kd=0.008   cambia ganancias (reinicia el integrador)
      cal1=193.3,0.655            calibracion de una pata en vivo (cal1..cal4):
                                  theta = THETA_W0 + GRADOS_POR_WRITE * write
                                  (la genera el comando "calibrar" de Python)
      tab1=v40,v50,...,v140       tabla de linealizacion del AS5600 (11 valores)
      tab1=off                    desactiva la tabla (lectura cruda)
                                  (las genera el comando "linealizar" de Python)
      modo=pid                    lazo cerrado
      modo=abierto                solo feedforward (por defecto al encender): sirve para
                                  verificar la calibracion y para comparar
  Protocolo ESP -> Python (igual que antes, 17 campos por linea):
      t_ms, ref1,theta1,error1,write1, ..., ref4,theta4,error4,write4
      (error = referencia - theta medida)
  Ademas, lineas que empiezan con '#': confirmacion de cada comando, avisos
  de referencia fuera de rango y cada 2 s el estado del enlace con la
  ESP-Sensores (bytes/s, lineas/s, ultima linea recibida).

  DIAGNOSTICO POR USB: esta placa NO recibe comandos por el monitor serie
  (su Serial es el enlace con la ESP-Sensores). Con DEBUG_SERIAL = true
  imprime cada 2 s el estado en el monitor serie a 115200 baudios.
  ============================================================================
*/

#include <ESP8266WiFi.h>
#include <Servo.h>

// ---------------------------------------------------------------
// WiFi (Access Point)
// ---------------------------------------------------------------
const char* SSID_AP = "RobotPID";
const char* PASSWORD_AP = "12345678";
WiFiServer servidor(80);
WiFiClient cliente;

// ---------------------------------------------------------------
// Servos -- mapeo real del shield (igual que la version anterior)
// ---------------------------------------------------------------
enum Pata { FI, FD, TI, TD, NUM_PATAS };
// orden FI, FD, TI, TD -- confirmado en el robot el 30-sep:
//   FI = D8 (GPIO15), FD = D4 (GPIO2), TI = D5 (GPIO14), TD = D0 (GPIO16)
// (antes estaba {16, 14, 2, 15}: las etiquetas de las 4 patas estaban invertidas)
const int PIN_TIBIA[NUM_PATAS] = {15, 2, 14, 16};

Servo servoTibia[NUM_PATAS];
Servo servo_5;  // Coxa FI (fija)
Servo servo_12; // Coxa FD (fija)
Servo servo_13; // Coxa TD (fija)
Servo servo_4;  // Coxa TI (fija)

// ---------------------------------------------------------------
// Calibracion por pata: theta = THETA_W0 + GRADOS_POR_WRITE * write
// Valores de la calibracion de FI (write 0 -> 220, write 180 -> 330).
// Verificar las otras 3 con modo=abierto: si al subir ref la pata BAJA,
// ese servo esta en espejo -> poner GRADOS_POR_WRITE negativo y THETA_W0
// igual al angulo medido con write = 0.
// ---------------------------------------------------------------
float THETA_W0[NUM_PATAS]         = {220.0, 220.0, 220.0, 220.0};
float GRADOS_POR_WRITE[NUM_PATAS] = {0.6111, 0.6111, 0.6111, 0.6111};

// ---------------------------------------------------------------
// Linealizacion del AS5600 por tabla (montaje con el iman fuera del eje)
// El iman va pegado al cuerpo del servo, que gira alrededor del eje de salida
// (no del centro del cuerpo): el AS5600 ve el iman describir un arco y su
// lectura sale deformada, aunque de forma repetible. La tabla guarda la
// lectura del sensor con write = 40, 50, ..., 140 (la mide Python con el
// comando "linealizar"). Con ella cada lectura se convierte en el write
// equivalente y de ahi en un angulo lineal:
//     theta_lineal = THETA_W0 + GRADOS_POR_WRITE * write_equivalente
// Sin tabla (tablaOK = false) se usa la lectura tal cual.
// ---------------------------------------------------------------
#define N_TABLA 11
const int TABLA_W0 = 40, TABLA_PASO = 10;      // writes 40, 50, ..., 140
float tabla[NUM_PATAS][N_TABLA];
bool  tablaOK[NUM_PATAS] = {false, false, false, false};

float linealizar(uint8_t p, float y) {
  if (!tablaOK[p]) return y;
  const float* t = tabla[p];
  // la tabla esta "desenrollada" (sin salto 360->0): acercar y a ella
  float medio = t[N_TABLA / 2];
  while (y - medio > 180.0) y -= 360.0;
  while (y - medio < -180.0) y += 360.0;
  bool crece = t[N_TABLA - 1] > t[0];
  int i = 0;                                    // segmento [i, i+1] que contiene y
  while (i < N_TABLA - 2 && (crece ? y > t[i + 1] : y < t[i + 1])) i++;
  float d = t[i + 1] - t[i];
  float w = TABLA_W0 + TABLA_PASO * i;
  if (fabs(d) > 1e-3) w += TABLA_PASO * (y - t[i]) / d;   // fuera de la tabla: extrapola
  return THETA_W0[p] + GRADOS_POR_WRITE[p] * w;
}

#define WRITE_MIN   40
#define WRITE_MAX  140

// ---------------------------------------------------------------
// PID (angulos en grados; Kp adimensional, Ki en 1/s, Kd en s)
// ---------------------------------------------------------------
float KP = 0.8;
float KI = 12.0;
float KD = 0.008;
const float TS = 0.020;                 // s, periodo fijo de control (50 Hz)
const unsigned long PERIODO_CONTROL_MS = 20;
const float TF = TS;                    // s, filtro de la derivada

// coeficientes de la ecuacion en diferencias (se recalculan si cambian las ganancias)
float c_i, c1, c2;
void calcularCoeficientes() {
  c_i = KI * TS / 2.0;                  // integral por Tustin
  c1  = (2.0 * TF - TS) / (2.0 * TF + TS);
  c2  = 2.0 * KD / (2.0 * TF + TS);     // derivada filtrada por Tustin
}

// Modelo de referencia: servo nominal wn = 30 rad/s, zeta = 1, discretizado
// con retenedor de orden cero a Ts = 20 ms:
//   ym_k = 1.09762*ym_(k-1) - 0.30119*ym_(k-2) + 0.12190*r_(k-1) + 0.08167*r_(k-2)
const float RM_A1 =  1.09762327, RM_A2 = -0.30119421;
const float RM_B1 =  0.12190138, RM_B2 =  0.08166956;

// estado por pata
float referencia[NUM_PATAS] = {245.0, 245.0, 245.0, 245.0};
float integral[NUM_PATAS], derivada[NUM_PATAS];
float e_prev[NUM_PATAS], y_prev[NUM_PATAS];
float ym1[NUM_PATAS], ym2[NUM_PATAS];   // salida del modelo de referencia
float r1[NUM_PATAS], r2[NUM_PATAS];     // referencias anteriores
bool  inicializado[NUM_PATAS] = {false, false, false, false};
int   writeActual[NUM_PATAS];
bool  modoPID = false;   // arranca en lazo abierto: el PID solo se activa con modo=pid (tras cargar la calibracion)
unsigned long ultimoCicloControl = 0;

// ---------------------------------------------------------------
// Conversiones con la calibracion de cada pata
// ---------------------------------------------------------------
int writeDesdeTheta(uint8_t p, float thetaDeg) {
  float w = (thetaDeg - THETA_W0[p]) / GRADOS_POR_WRITE[p];
  return (int)lroundf(w);
}

// ---------------------------------------------------------------
// Recepcion de la ESP-Sensores por Serial de HARDWARE (no bloqueante)
// ---------------------------------------------------------------
const long BAUD_SENSORES = 115200;      // DEBE coincidir con la ESP-Sensores
#define VALOR_NO_DISPONIBLE -999.0
float theta[NUM_PATAS] = {VALOR_NO_DISPONIBLE, VALOR_NO_DISPONIBLE,
                          VALOR_NO_DISPONIBLE, VALOR_NO_DISPONIBLE};
unsigned long ultimoDatoRecibido = 0;
const unsigned long TIMEOUT_DATOS_MS = 100;   // 5 ciclos sin dato -> se congela el lazo

float parsearCampo(const String &linea, const String &clave) {
  int inicio = linea.indexOf(clave);
  if (inicio == -1) return VALOR_NO_DISPONIBLE;
  inicio += clave.length();
  int fin = linea.indexOf(',', inicio);
  if (fin == -1) fin = linea.length();
  String valorStr = linea.substring(inicio, fin);
  valorStr.trim();
  if (valorStr == "NA") return VALOR_NO_DISPONIBLE;
  return valorStr.toFloat();
}

// contadores para el diagnostico
unsigned long bytesSensores = 0, lineasSensores = 0, lineasSinTheta = 0;
String ultimaLineaSensores = "";

void procesarLineaSensores(const String &linea) {
  lineasSensores++;
  ultimaLineaSensores = linea.substring(0, 90);
  if (linea.indexOf("theta") == -1) { lineasSinTheta++; return; }   // basura o formato distinto
  theta[FI] = parsearCampo(linea, "theta1=");
  theta[FD] = parsearCampo(linea, "theta2=");
  theta[TI] = parsearCampo(linea, "theta3=");
  theta[TD] = parsearCampo(linea, "theta4=");
  for (uint8_t p = 0; p < NUM_PATAS; p++)         // angulo lineal si hay tabla
    if (theta[p] > VALOR_NO_DISPONIBLE + 1.0) theta[p] = linealizar(p, theta[p]);
  ultimoDatoRecibido = millis();
}

String bufferSerial = "";
void revisarSerialSensores() {
  while (Serial.available()) {
    char c = Serial.read();
    bytesSensores++;
    if (c == '\n') {
      bufferSerial.trim();
      if (bufferSerial.length() > 0) procesarLineaSensores(bufferSerial);
      bufferSerial = "";
    } else if (c != '\r') {
      bufferSerial += c;
      if (bufferSerial.length() > 200) bufferSerial = "";   // linea corrupta
    }
  }
}

// ---------------------------------------------------------------
// Comandos de Python por WiFi (no bloqueante)
// ---------------------------------------------------------------
String bufferEntradaWifi = "";

bool esNumero(const String &s) {
  if (s.length() == 0) return false;
  bool visto = false;
  for (uint16_t i = 0; i < s.length(); i++) {
    char ch = s.charAt(i);
    if (isDigit(ch) || ch == '.' || ch == '-') visto = true;
    else return false;
  }
  return visto;
}

void reiniciarControlador() {
  for (uint8_t p = 0; p < NUM_PATAS; p++) inicializado[p] = false;
}

void mensajeWifi(const String &m) {        // lineas que empiezan con '#' = mensajes, no telemetria
  if (cliente && cliente.connected()) cliente.print("# " + m + "\r\n");
}

void avisarSiFueraDeRango(uint8_t p, float r) {
  float a = THETA_W0[p] + GRADOS_POR_WRITE[p] * WRITE_MIN;
  float b = THETA_W0[p] + GRADOS_POR_WRITE[p] * WRITE_MAX;
  float lo = min(a, b), hi = max(a, b);
  if (r < lo || r > hi) {
    mensajeWifi("AVISO pata " + String(p + 1) + ": ref " + String(r, 1) +
                " fuera del rango alcanzable [" + String(lo, 1) + ", " + String(hi, 1) +
                "] (WRITE_MIN/MAX). La pata se queda en el limite.");
  }
}

void aplicarComando(String linea) {
  linea.trim();
  if (linea.length() == 0) return;
  // acepta "ref=250", "ref 250" o "ref:250"
  int igual = linea.indexOf('=');
  if (igual == -1) igual = linea.indexOf(' ');
  if (igual == -1) igual = linea.indexOf(':');
  if (igual == -1) { mensajeWifi("comando no reconocido: " + linea); return; }
  String clave = linea.substring(0, igual);
  String v = linea.substring(igual + 1);
  v.trim();

  clave.trim();
  if (clave == "modo") {
    if (v == "pid") modoPID = true;
    else if (v == "abierto") modoPID = false;
    else { mensajeWifi("modo desconocido: " + v + " (usar pid o abierto)"); return; }
    reiniciarControlador();
    mensajeWifi("OK modo=" + v);
    return;
  }
  // calibracion en vivo: cal1=THETA_W0,GRADOS_POR_WRITE  (cal1..cal4 = FI, FD, TI, TD)
  if (clave.length() == 4 && clave.startsWith("cal")) {
    int p = clave.charAt(3) - '1';
    int coma = v.indexOf(',');
    if (p < 0 || p >= NUM_PATAS || coma == -1) { mensajeWifi("formato: cal1=theta_w0,grados_por_write"); return; }
    float w0 = v.substring(0, coma).toFloat();
    float gpw = v.substring(coma + 1).toFloat();
    if (fabs(gpw) < 0.05) { mensajeWifi("grados_por_write demasiado pequeno"); return; }
    THETA_W0[p] = w0; GRADOS_POR_WRITE[p] = gpw;
    reiniciarControlador();
    mensajeWifi("OK calibracion pata " + String(p + 1) + ": theta = " + String(w0, 2) + " + (" +
                String(gpw, 4) + ")*write. Rango alcanzable: " +
                String(w0 + gpw * WRITE_MIN, 1) + " a " + String(w0 + gpw * WRITE_MAX, 1) +
                ". Se pierde al reiniciar: copiarla en THETA_W0/GRADOS_POR_WRITE.");
    return;
  }
  // tabla de linealizacion: tab1=v40,v50,...,v140  (11 valores)  o  tab1=off
  if (clave.length() == 4 && clave.startsWith("tab")) {
    int p = clave.charAt(3) - '1';
    if (p < 0 || p >= NUM_PATAS) { mensajeWifi("formato: tab1=11 valores o tab1=off"); return; }
    if (v == "off") {
      tablaOK[p] = false;
      mensajeWifi("OK tabla pata " + String(p + 1) + " desactivada (lectura cruda)");
      reiniciarControlador();
      return;
    }
    float vals[N_TABLA];
    int n = 0, desde = 0;
    while (n < N_TABLA) {
      int coma = v.indexOf(',', desde);
      String item = (coma == -1) ? v.substring(desde) : v.substring(desde, coma);
      item.trim();
      if (item.length() == 0) break;
      vals[n++] = item.toFloat();
      if (coma == -1) break;
      desde = coma + 1;
    }
    if (n != N_TABLA) { mensajeWifi("tabla con " + String(n) + " valores, se esperaban 11"); return; }
    for (int i = 0; i < N_TABLA; i++) tabla[p][i] = vals[i];
    tablaOK[p] = true;
    reiniciarControlador();
    mensajeWifi("OK tabla pata " + String(p + 1) + " cargada: " + String(vals[0], 1) + " .. " +
                String(vals[N_TABLA - 1], 1));
    return;
  }
  if (!esNumero(v)) { mensajeWifi("valor no numerico en: " + linea); return; }
  float val = v.toFloat();

  if      (clave == "ref1") { referencia[FI] = val; avisarSiFueraDeRango(FI, val); }
  else if (clave == "ref2") { referencia[FD] = val; avisarSiFueraDeRango(FD, val); }
  else if (clave == "ref3") { referencia[TI] = val; avisarSiFueraDeRango(TI, val); }
  else if (clave == "ref4") { referencia[TD] = val; avisarSiFueraDeRango(TD, val); }
  else if (clave == "ref")  {
    for (uint8_t p = 0; p < NUM_PATAS; p++) { referencia[p] = val; avisarSiFueraDeRango(p, val); }
  }
  else if (clave == "kp")   { KP = val; calcularCoeficientes(); reiniciarControlador(); }
  else if (clave == "ki")   { KI = val; calcularCoeficientes(); reiniciarControlador(); }
  else if (clave == "kd")   { KD = val; calcularCoeficientes(); reiniciarControlador(); }
  else { mensajeWifi("comando no reconocido: " + linea); return; }
  mensajeWifi("OK " + clave + "=" + String(val, 3));
}

// ---------------------------------------------------------------
// Diagnostico cada 2 s: por WiFi (siempre) y por el monitor serie (si DEBUG_SERIAL)
// El monitor serie debe estar a 115200 baudios.
// ---------------------------------------------------------------
#define DEBUG_SERIAL true
unsigned long ultimoDiagnostico = 0, bytesAnt = 0, lineasAnt = 0;

void diagnostico() {
  unsigned long ahora = millis();
  if (ahora - ultimoDiagnostico < 2000) return;
  float dt = (ahora - ultimoDiagnostico) / 1000.0;
  ultimoDiagnostico = ahora;
  float bps = (bytesSensores - bytesAnt) / dt;
  float lps = (lineasSensores - lineasAnt) / dt;
  bytesAnt = bytesSensores; lineasAnt = lineasSensores;

  String estado;
  if (bps < 1)                         estado = "NO LLEGA NADA de la ESP-Sensores (revisar cable TX->RX, GND comun y que este encendida)";
  else if (lps < 1)                    estado = "llegan bytes pero no lineas completas: BAUDIOS DISTINTOS (esta placa usa " + String(BAUD_SENSORES) + ")";
  else if (millis() - ultimoDatoRecibido > TIMEOUT_DATOS_MS) estado = "llegan lineas pero sin theta: revisar formato (ver ultima linea)";
  else if (lps < 45)                   estado = "OK pero lento: " + String(lps, 0) + " lineas/s (el PID necesita 50)";
  else                                 estado = "OK";

  String s = "estado sensores: " + estado + " | " + String(bps, 0) + " B/s, " + String(lps, 0) +
             " lineas/s | modo=" + String(modoPID ? "pid" : "abierto") +
             " | theta=" + String(theta[FI], 1) + "," + String(theta[FD], 1) + "," +
             String(theta[TI], 1) + "," + String(theta[TD], 1) +
             " | write=" + String(writeActual[FI]) + "," + String(writeActual[FD]) + "," +
             String(writeActual[TI]) + "," + String(writeActual[TD]);
  mensajeWifi(s);
  mensajeWifi("ultima linea recibida: " + (ultimaLineaSensores.length() ? ultimaLineaSensores : String("(ninguna)")));
  if (DEBUG_SERIAL) {
    Serial.print("[dbg] "); Serial.println(s);
    Serial.print("[dbg] ultima linea: "); Serial.println(ultimaLineaSensores);
    Serial.print("[dbg] WiFi: "); Serial.print(WiFi.softAPgetStationNum());
    Serial.print(" equipo(s) en la red, cliente Python ");
    Serial.println((cliente && cliente.connected()) ? "CONECTADO" : "no conectado");
  }
}

void revisarComandoWifi() {
  if (!cliente || !cliente.connected()) return;
  while (cliente.available()) {
    char c = cliente.read();
    if (c == '\n') {
      aplicarComando(bufferEntradaWifi);
      bufferEntradaWifi = "";
    } else if (c != '\r') {
      bufferEntradaWifi += c;
      if (bufferEntradaWifi.length() > 200) bufferEntradaWifi = "";
    }
  }
}

// ---------------------------------------------------------------
// Un paso del controlador de una pata. Devuelve el write a enviar.
// ---------------------------------------------------------------
int pasoControl(uint8_t p, float r, float y) {
  if (!inicializado[p]) {
    // arranque sin salto: el modelo de referencia parte de la posicion actual
    integral[p] = 0; derivada[p] = 0;
    e_prev[p] = 0;   y_prev[p] = y;
    ym1[p] = y; ym2[p] = y; r1[p] = y; r2[p] = y;
    inicializado[p] = true;
  }

  // 1) modelo de referencia: trayectoria que seguiria un servo nominal
  float ym = RM_A1 * ym1[p] + RM_A2 * ym2[p] + RM_B1 * r1[p] + RM_B2 * r2[p];
  ym2[p] = ym1[p]; ym1[p] = ym;
  r2[p] = r1[p];   r1[p] = r;

  // 2) PID sobre el error respecto al modelo
  float e = ym - y;
  float integralNueva = integral[p] + c_i * (e + e_prev[p]);
  derivada[p] = c1 * derivada[p] - c2 * (y - y_prev[p]);

  // 3) feedforward (la referencia) + correccion del PID
  float thetaCmd = r + KP * e + integralNueva + derivada[p];
  int w = writeDesdeTheta(p, thetaCmd);
  int wSat = constrain(w, WRITE_MIN, WRITE_MAX);

  if (w == wSat) integral[p] = integralNueva;   // anti-windup
  e_prev[p] = e;
  y_prev[p] = y;
  return wSat;
}

// ---------------------------------------------------------------
// Telemetria en una sola escritura por ciclo
// ---------------------------------------------------------------
void enviarTelemetria(unsigned long t, const bool *valida, const float *error) {
  if (!cliente || !cliente.connected()) return;
  String s;
  s.reserve(160);
  s += t;
  for (uint8_t p = 0; p < NUM_PATAS; p++) {
    s += ','; s += String(referencia[p], 2);
    s += ','; s += valida[p] ? String(theta[p], 2) : String("NA");
    s += ','; s += valida[p] ? String(error[p], 2) : String("NA");
    s += ','; s += writeActual[p];
  }
  s += "\r\n";
  cliente.print(s);
}

void setup() {
  Serial.begin(BAUD_SENSORES);   // enlace con la ESP-Sensores (NO con Python)

  for (uint8_t p = 0; p < NUM_PATAS; p++) {
    writeActual[p] = constrain(writeDesdeTheta(p, referencia[p]), WRITE_MIN, WRITE_MAX);
    servoTibia[p].attach(PIN_TIBIA[p]);
    servoTibia[p].write(writeActual[p]);
  }
  servo_5.attach(5);   servo_5.write(0);
  servo_12.attach(12); servo_12.write(180);
  servo_13.attach(13); servo_13.write(0);
  servo_4.attach(4);   servo_4.write(180);

  calcularCoeficientes();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID_AP, PASSWORD_AP);
  servidor.begin();

  delay(300);
  Serial.println();
  Serial.println("[dbg] ESP principal v2 lista. WiFi RobotPID en 192.168.4.1:80");
  ultimoCicloControl = millis();
  ultimoDatoRecibido = 0;          // sin datos hasta que llegue la primera linea
}

void loop() {
  // 1) conexion y comandos de Python por WiFi
  // Si Python se reconecta (se cayo el WiFi, se cerro el programa), la conexion
  // vieja puede seguir "abierta" para la ESP varios minutos. Por eso cualquier
  // conexion nueva reemplaza a la anterior, en vez de esperar a que muera.
  WiFiClient nuevo = servidor.available();
  if (nuevo) {
    if (cliente) cliente.stop();
    cliente = nuevo;
    cliente.setNoDelay(true);
    bufferEntradaWifi = "";
    mensajeWifi("ESP principal v2 conectada. Comandos: ref=, ref1..4=, kp=, ki=, kd=, modo=pid|abierto, cal1..4=");
  }
  revisarComandoWifi();

  // 2) datos de la ESP-Sensores (sin bloquear) y diagnostico
  revisarSerialSensores();
  diagnostico();
  bool enlaceActivo = (millis() - ultimoDatoRecibido) < TIMEOUT_DATOS_MS;

  // 3) ciclo de control a periodo fijo
  unsigned long ahora = millis();
  if (ahora - ultimoCicloControl < PERIODO_CONTROL_MS) return;
  ultimoCicloControl += PERIODO_CONTROL_MS;          // sin deriva acumulada
  if (ahora - ultimoCicloControl > PERIODO_CONTROL_MS) ultimoCicloControl = ahora;

  bool valida[NUM_PATAS];
  float error[NUM_PATAS];

  for (uint8_t p = 0; p < NUM_PATAS; p++) {
    valida[p] = enlaceActivo && (theta[p] > VALOR_NO_DISPONIBLE + 1.0);
    error[p] = valida[p] ? referencia[p] - theta[p] : 0;

    if (!modoPID) {
      // lazo abierto: solo la calibracion (sirve para verificarla)
      writeActual[p] = constrain(writeDesdeTheta(p, referencia[p]), WRITE_MIN, WRITE_MAX);
      servoTibia[p].write(writeActual[p]);
      inicializado[p] = false;
    } else if (valida[p]) {
      writeActual[p] = pasoControl(p, referencia[p], theta[p]);
      servoTibia[p].write(writeActual[p]);
    } else {
      // sin medicion: mantener el ultimo write y reiniciar al volver el dato
      inicializado[p] = false;
    }
  }

  // 4) telemetria a Python
  enviarTelemetria(ahora, valida, error);
}
