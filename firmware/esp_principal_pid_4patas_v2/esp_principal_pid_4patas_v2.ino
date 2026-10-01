/*
  ============================================================================
  ESP8266 PRINCIPAL -- PID de las 4 rodillas (lazo cerrado real), con:
    - Sensores (theta1..4) recibidos por Serial de HARDWARE desde la ESP-Sensores
    - Setpoint y telemetria por WIFI hacia/desde Python en la PC
  Robot Cuadrupedo (ACEBOTT/QD020)
  ============================================================================

  Igual que esp_principal_pid_wifi.ino (que ya te funciono con la pata FI),
  pero extendido a las 4 patas a la vez. Mismo mapeo real de pines que tu
  codigo de referencia:
    Coxas (FIJAS):  servo_5=FI, servo_12=FD, servo_13=TD, servo_4=TI
    Tibias (ACTIVAS, orden FI,FD,TI,TD): pines 16, 14, 2, 15

  Red WiFi (Access Point, esta placa ES la red):
    SSID:     RobotPID
    Password: 12345678
    IP fija:  192.168.4.1
    Puerto:   80

  Protocolo por WiFi:
    Python -> ESP: setpoint de UNA pata o de las 4 iguales:
        ref1=250.0   -> solo FI
        ref2=250.0   -> solo FD
        ref3=250.0   -> solo TI
        ref4=250.0   -> solo TD
        ref=250.0    -> las 4 patas al mismo valor
    ESP -> Python: una linea por ciclo de control, formato CSV (17 campos):
        t_ms,
        ref1,theta1,error1,write1,
        ref2,theta2,error2,write2,
        ref3,theta3,error3,write3,
        ref4,theta4,error4,write4
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
// Servos -- mapeo real del shield (igual que esp_principal_gait_sensores.ino)
// ---------------------------------------------------------------
enum Pata { FI, FD, TI, TD, NUM_PATAS };
const char* NOMBRE_PATA[NUM_PATAS] = {"FI", "FD", "TI", "TD"};
const int PIN_TIBIA[NUM_PATAS] = {16, 14, 2, 15}; // orden FI,FD,TI,TD -- mismo que tu codigo de referencia

Servo servoTibia[NUM_PATAS];
Servo servo_5;  // Coxa FI (fija)
Servo servo_12; // Coxa FD (fija)
Servo servo_13; // Coxa TD (fija)
Servo servo_4;  // Coxa TI (fija)

// ---------------------------------------------------------------
// PID de posicion -- mismas ganancias que ya veniamos usando, una
// instancia de estado POR PATA (integral/derivada no se comparten)
// ---------------------------------------------------------------
float KP_POS = 0.5;
float KI_POS = 0.05;
float KD_POS = 0.02;

#define WRITE_BASE   90
#define WRITE_MIN    40
#define WRITE_MAX   140
#define INTEGRAL_MAX 50.0
#define PERIODO_CONTROL_MS 20 // 50 Hz

float referencia[NUM_PATAS]   = {245.0, 245.0, 245.0, 245.0};
float integral_e[NUM_PATAS]   = {0, 0, 0, 0};
float e_prev[NUM_PATAS]       = {0, 0, 0, 0};
bool  primerCiclo[NUM_PATAS]  = {true, true, true, true};
unsigned long ultimoCicloControl = 0;

// ---------------------------------------------------------------
// Recepcion de la ESP-Sensores por Serial de HARDWARE (igual que
// esp_principal_receptor.ino) -- de aqui salen theta1..4
// ---------------------------------------------------------------
#define VALOR_NO_DISPONIBLE -999.0
float theta[NUM_PATAS] = {VALOR_NO_DISPONIBLE, VALOR_NO_DISPONIBLE, VALOR_NO_DISPONIBLE, VALOR_NO_DISPONIBLE};
unsigned long ultimoDatoRecibido = 0;
const unsigned long TIMEOUT_DATOS_MS = 1000;

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

void procesarLineaSensores(const String &linea) {
  theta[FI] = parsearCampo(linea, "theta1=");
  theta[FD] = parsearCampo(linea, "theta2=");
  theta[TI] = parsearCampo(linea, "theta3=");
  theta[TD] = parsearCampo(linea, "theta4=");
  ultimoDatoRecibido = millis();
}

// ---- lee comandos de Python (setpoint) que lleguen por WIFI -- no bloqueante ----
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

void aplicarComando(String linea) {
  linea.trim();
  if (linea.length() == 0) return;

  if (linea.startsWith("ref1=")) { String v = linea.substring(5); if (esNumero(v)) referencia[FI] = v.toFloat(); }
  else if (linea.startsWith("ref2=")) { String v = linea.substring(5); if (esNumero(v)) referencia[FD] = v.toFloat(); }
  else if (linea.startsWith("ref3=")) { String v = linea.substring(5); if (esNumero(v)) referencia[TI] = v.toFloat(); }
  else if (linea.startsWith("ref4=")) { String v = linea.substring(5); if (esNumero(v)) referencia[TD] = v.toFloat(); }
  else if (linea.startsWith("ref=")) {
    String v = linea.substring(4);
    if (esNumero(v)) {
      float val = v.toFloat();
      for (uint8_t p = 0; p < NUM_PATAS; p++) referencia[p] = val;
    }
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
    }
  }
}

void setup() {
  Serial.begin(9600); // este Serial es el enlace hacia la ESP-Sensores (NO hacia Python)

  for (uint8_t p = 0; p < NUM_PATAS; p++) {
    servoTibia[p].attach(PIN_TIBIA[p]);
    servoTibia[p].write(WRITE_BASE);
  }
  servo_5.attach(5);   servo_5.write(0);
  servo_12.attach(12); servo_12.write(180);
  servo_13.attach(13); servo_13.write(0);
  servo_4.attach(4);   servo_4.write(180);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID_AP, PASSWORD_AP);
  servidor.begin();

  delay(300);
  ultimoCicloControl = millis();
  ultimoDatoRecibido = millis();
}

void loop() {
  // ---- 1) aceptar/mantener la conexion de Python por WiFi ----
  if (!cliente || !cliente.connected()) {
    cliente = servidor.available();
  }
  revisarComandoWifi();

  // ---- 2) leer lo que llegue de la ESP-Sensores por Serial de hardware ----
  if (Serial.available()) {
    String linea = Serial.readStringUntil('\n');
    linea.trim();
    if (linea.length() > 0) procesarLineaSensores(linea);
  }
  bool enlaceSensoresActivo = (millis() - ultimoDatoRecibido) < TIMEOUT_DATOS_MS;

  // ---- 3) ciclo de control PID, cada PERIODO_CONTROL_MS, para las 4 patas ----
  unsigned long ahora = millis();
  if (ahora - ultimoCicloControl < PERIODO_CONTROL_MS) return;
  float dt_s = (ahora - ultimoCicloControl) / 1000.0;
  ultimoCicloControl = ahora;

  int nuevoWrite[NUM_PATAS];
  float error[NUM_PATAS];
  bool thetaValida[NUM_PATAS];

  for (uint8_t p = 0; p < NUM_PATAS; p++) {
    nuevoWrite[p] = WRITE_BASE;
    error[p] = 0;
    thetaValida[p] = enlaceSensoresActivo && (theta[p] > VALOR_NO_DISPONIBLE + 1.0);

    if (thetaValida[p]) {
      error[p] = referencia[p] - theta[p];

      if (primerCiclo[p]) {
        e_prev[p] = error[p];
        primerCiclo[p] = false;
      }

      integral_e[p] += error[p] * dt_s;
      integral_e[p] = constrain(integral_e[p], -INTEGRAL_MAX, INTEGRAL_MAX);

      float derivada = (dt_s > 0) ? (error[p] - e_prev[p]) / dt_s : 0;
      e_prev[p] = error[p];

      float correccion = KP_POS * error[p] + KI_POS * integral_e[p] + KD_POS * derivada;
      nuevoWrite[p] = constrain((int)(WRITE_BASE + correccion), WRITE_MIN, WRITE_MAX);
      servoTibia[p].write(nuevoWrite[p]);
    }
  }

  // ---- 4) mandar telemetria a Python por WiFi (si hay cliente conectado) ----
  if (cliente && cliente.connected()) {
    cliente.print(ahora);
    for (uint8_t p = 0; p < NUM_PATAS; p++) {
      cliente.print(","); cliente.print(referencia[p], 2);
      cliente.print(","); cliente.print(thetaValida[p] ? String(theta[p], 2) : "NA");
      cliente.print(","); cliente.print(thetaValida[p] ? String(error[p], 2) : "NA");
      cliente.print(","); cliente.print(nuevoWrite[p]);
    }
    cliente.println();
  }
}
