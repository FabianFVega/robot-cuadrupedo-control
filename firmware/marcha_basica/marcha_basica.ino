/*
  MARCHA BASICA (20-nov) -- solo servos, sin PID, sin sensores, sin orbita.
  ESP principal (NodeMCU ESP8266). Para volver a lo de antes, cargar de nuevo
  esp_principal_pid_4patas_v2.

  Objetivo: encontrar un paso que avance recto, con lo minimo. Cada rodilla sigue
      angulo = centro + sentido * amp * sen(2*pi*frec*t + fase de la pata)
  y el angulo se pasa a write con la calibracion de siempre (0.6 grados por unidad
  de write, servos derechos en espejo). No lee nada de la ESP-Sensores.

  Dos pasos (andar=):
    trote : diagonales juntas  -> FI con TD, FD con TI      fases 0, 180, 180,   0
    salto : delanteras juntas, traseras juntas ("bound")    fases 0,   0, 180, 180

  WiFi: red RobotPID, clave 12345678, 192.168.4.1 puerto 80 (el mismo programa de
  la PC; los comandos de calibracion que manda al conectar se ignoran).

  Comandos (uno por linea):
    andar=trote | andar=salto   tipo de paso
    marcha=1 / marcha=0         arranca / para (rampa de 1 s)
    amp=20                      amplitud (grados, 0..35)
    frec=1.5                    pasos por segundo (0.2..4)
    sentido=1 / sentido=-1      invierte el movimiento de todas las patas
    desfase=0                   atrasa las traseras (grados) respecto a las delanteras
    damp=0                      diferencia de amplitud izquierda-derecha (grados, para
                                corregir si se va de lado: izq = amp - damp/2, der = amp + damp/2)
    ref=270                     marcha=0 y las 4 patas a ese angulo (prueba en el soporte)
    c1=97 c2=111 c3=90 c4=90    write con el que cada pata queda VERTICAL (270)
    ver                         muestra todo (el programa de la PC usa "estado" para
                                otra cosa y no lo manda a la ESP)
  Patas: 1=FI (GPIO15), 2=FD (GPIO2), 3=TI (GPIO14), 4=TD (GPIO16).
*/
#include <ESP8266WiFi.h>
#include <Servo.h>
#include <core_version.h>

enum { FI, FD, TI, TD, NP };
const char *NOMBRE[NP] = {"FI", "FD", "TI", "TD"};
const int PIN[NP] = {15, 2, 14, 16};
// write que deja cada rodilla vertical (de THETA_W0/GRADOS_POR_WRITE de la v2:
// FI (270-211.8)/0.6 = 97, FD (270-336.6)/-0.6 = 111, TI 90, TD 90)
int   CENTRO_W[NP] = {97, 111, 90, 90};
// +1: subir el angulo = subir el write; -1: servo en espejo (lado derecho)
const int   SIGNO[NP] = {+1, -1, +1, -1};
const float GRADOS_POR_WRITE = 0.6;
const bool  TRASERA[NP] = {false, false, true, true};
const bool  IZQ[NP]     = {true, false, true, false};
const int   WMIN = 10, WMAX = 170;

float FASE_TROTE[NP] = {0, 180, 180, 0};
float FASE_SALTO[NP] = {0, 0, 180, 180};
bool  pasoSalto = false;

float AMP = 20.0, FREC = 1.5, SENTIDO = 1.0, DESFASE = 0.0, DAMP = 0.0;
bool  marcha = false;
float rampa = 0.0;            // 0..1
float fase = 0.0;             // rad
float angulo[NP] = {270, 270, 270, 270};
int   writeAct[NP] = {-1, -1, -1, -1};

Servo tibia[NP];
Servo coxa5, coxa12, coxa13, coxa4;   // coxas fijas (como en la v2)

WiFiServer servidor(80);
WiFiClient cliente;
String bufEntrada;
unsigned long tUltimo = 0, tTele = 0;

void msg(const String &m) {
  Serial.println(m);
  if (cliente && cliente.connected()) cliente.print("# " + m + "\r\n");
}

int writeDe(uint8_t p, float ang) {
  float w = CENTRO_W[p] + SIGNO[p] * (ang - 270.0) / GRADOS_POR_WRITE;
  return constrain((int)lroundf(w), WMIN, WMAX);
}

void mover(uint8_t p, float ang) {
  angulo[p] = ang;
  int w = writeDe(p, ang);
  if (w != writeAct[p]) { tibia[p].write(w); writeAct[p] = w; }
}

String resumen() {
  String s = String("andar=") + (pasoSalto ? "salto" : "trote") + " marcha=" + (marcha ? "1" : "0") +
             " amp=" + String(AMP, 1) + " frec=" + String(FREC, 2) + " sentido=" + String((int)SENTIDO) +
             " desfase=" + String(DESFASE, 0) + " damp=" + String(DAMP, 1) +
             " | centros c1..c4=" + String(CENTRO_W[0]) + "," + String(CENTRO_W[1]) + "," +
             String(CENTRO_W[2]) + "," + String(CENTRO_W[3]) +
             " | write=" + String(writeAct[0]) + "," + String(writeAct[1]) + "," + String(writeAct[2]) + "," + String(writeAct[3]);
  return s;
}

void comando(String linea) {
  linea.trim();
  if (linea.length() == 0) return;
  int k = linea.indexOf('*');                 // por si llega con suma de verificacion
  if (k > 0) linea = linea.substring(0, k);
  String l = linea; l.toLowerCase();
  if (l == "ver" || l == "estado") { msg("MARCHA BASICA (sin PID ni sensores) | " + resumen()); return; }
  int i = l.indexOf('=');
  if (i < 0) { msg("comando no reconocido: " + linea); return; }
  String c = l.substring(0, i), v = l.substring(i + 1);
  c.trim(); v.trim();
  float x = v.toFloat();

  if (c == "andar") {
    if (v == "trote") pasoSalto = false;
    else if (v == "salto") pasoSalto = true;
    else { msg("andar=trote o andar=salto"); return; }
  }
  else if (c == "marcha")  { marcha = (x != 0); }
  else if (c == "amp")     { AMP = constrain(x, 0.0, 35.0); }
  else if (c == "frec")    { FREC = constrain(x, 0.2, 4.0); }
  else if (c == "sentido") { SENTIDO = (x >= 0) ? 1.0 : -1.0; }
  else if (c == "desfase") { DESFASE = constrain(x, -180.0, 180.0); }
  else if (c == "damp")    { DAMP = constrain(x, -30.0, 30.0); }
  else if (c == "ref") {
    marcha = false; rampa = 0.0;
    for (uint8_t p = 0; p < NP; p++) mover(p, constrain(x, 230.0, 310.0));
  }
  else if (c.length() == 2 && c[0] == 'c' && c[1] >= '1' && c[1] <= '4') {
    uint8_t p = c[1] - '1';
    CENTRO_W[p] = constrain((int)x, WMIN, WMAX);
    if (!marcha && rampa <= 0.0) mover(p, angulo[p]);
  }
  else {                                         // cal1=, tab1=, ganancias=... de la PC
    msg("ignorado en la marcha basica: " + linea);
    return;
  }
  msg("OK " + resumen());
}

void setup() {
  Serial.begin(115200);
  for (uint8_t p = 0; p < NP; p++) { tibia[p].attach(PIN[p]); mover(p, 270.0); }
  coxa5.attach(5);   coxa5.write(0);
  coxa12.attach(12); coxa12.write(180);
  coxa13.attach(13); coxa13.write(0);
  coxa4.attach(4);   coxa4.write(180);
#if defined(ARDUINO_ESP8266_MAJOR) && (ARDUINO_ESP8266_MAJOR >= 3)
  enablePhaseLockedWaveform();
#endif
  WiFi.mode(WIFI_AP);
  WiFi.setOutputPower(14.0);
  WiFi.softAP("RobotPID", "12345678");
  servidor.begin();
  Serial.println("\nMARCHA BASICA lista: WiFi RobotPID, 192.168.4.1:80");
  tUltimo = millis();
}

void loop() {
  // --- WiFi: un cliente a la vez
  WiFiClient nuevo = servidor.accept();
  if (nuevo) {
    if (cliente) cliente.stop();
    cliente = nuevo; cliente.setNoDelay(true);
    msg("ESP principal: MARCHA BASICA (sin PID ni sensores). Comandos: andar=trote|salto, marcha=1|0, amp=, frec=, sentido=, desfase=, damp=, ref=, c1..c4=, ver");
    msg(resumen());
  }
  while (cliente && cliente.available()) {
    char ch = cliente.read();
    if (ch == '\n') { comando(bufEntrada); bufEntrada = ""; }
    else if (ch != '\r' && bufEntrada.length() < 200) bufEntrada += ch;
  }
  while (Serial.available()) Serial.read();      // lo que mande la ESP-Sensores se descarta

  // --- marcha a 50 Hz
  unsigned long ahora = millis();
  if (ahora - tUltimo < 20) return;
  float dt = (ahora - tUltimo) / 1000.0;
  tUltimo = ahora;
  float obj = marcha ? 1.0 : 0.0;
  if (rampa < obj) rampa = min(obj, rampa + dt);
  else if (rampa > obj) rampa = max(obj, rampa - dt);
  if (rampa > 0.0 || marcha) {
    fase += 2.0 * PI * FREC * dt;
    if (fase > 2.0 * PI) fase -= 2.0 * PI;
    const float *f = pasoSalto ? FASE_SALTO : FASE_TROTE;
    for (uint8_t p = 0; p < NP; p++) {
      float a = IZQ[p] ? AMP - DAMP / 2.0 : AMP + DAMP / 2.0;
      float fp = (f[p] + (TRASERA[p] ? DESFASE : 0.0)) * PI / 180.0;
      mover(p, 270.0 + SENTIDO * rampa * a * sin(fase + fp));
    }
  }

  // --- telemetria minima cada 100 ms (el programa de la PC la espera; theta = NA)
  if (cliente && cliente.connected() && ahora - tTele >= 100) {
    tTele = ahora;
    String s; s.reserve(140);
    s += ahora;
    for (uint8_t p = 0; p < NP; p++) {
      s += ','; s += String(angulo[p], 2); s += ",NA,NA,"; s += writeAct[p];
    }
    s += ",NA,NA,NA,"; s += String(DAMP, 2); s += ",0.0\r\n";
    if (cliente.availableForWrite() >= (int)s.length()) cliente.print(s);
  }
}
