/*
  ============================================================================
  ESP-SENSORES -- VERSION 2 (para el PID de las 4 rodillas)
  Robot Cuadrupedo (ACEBOTT/QD020)
  ============================================================================

  Lee los 4 AS5600 (via multiplexor TCA9548A), el GY-521 y el VL53L0X, y
  manda UNA linea cada 20 ms (50 por segundo) por Serial de hardware hacia
  la ESP principal:

    theta1=..,theta2=..,theta3=..,theta4=..,psi_dot=..,delta_omega=..,distancia=..

  Cambios respecto a la version anterior:
    1. Serial a 115200 baudios (antes 9600). DEBE coincidir con
       BAUD_SENSORES de la ESP principal.
    2. Una linea cada 20 ms con periodo fijo (antes delay(200) = 5 por segundo).
    3. I2C a 400 kHz: las 4 lecturas de AS5600 tardan ~1 ms en total.
    4. VL53L0X en modo continuo: ya no frena el ciclo ~30 ms por medicion.
       La distancia se actualiza cada 50 ms y se reenvia la ultima valida.
    5. Opcional: offset y signo por pata (por defecto se manda el angulo
       crudo del AS5600, igual que antes).

  Cableado I2C: D1 = SCL, D2 = SDA.
  Mux TCA9548A en 0x70, canales de los AS5600 (confirmados en el robot):
      FI = canal 0, FD = canal 7, TI = canal 1, TD = canal 2
  GY-521 y VL53L0X: ver SENSORES_NAV_EN_MUX abajo.

  Prueba: abrir el monitor serie a 115200 con esta placa sola; deben verse
  primero las lineas de diagnostico y luego ~50 lineas theta por segundo.
  Luego conectar TX de esta placa -> RX de la principal, y GND con GND.
  ============================================================================
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>

#define PIN_SDA 4   // D2
#define PIN_SCL 5   // D1

const long BAUD_ENLACE = 115200;            // igual que BAUD_SENSORES en la principal
const unsigned long PERIODO_ENVIO_MS = 20;  // 50 lineas por segundo

// ---------------------------------------------------------------
// Multiplexor y canales
// ---------------------------------------------------------------
#define DIR_MUX    0x70
#define DIR_AS5600 0x36      // fija en los 4 -- por eso el mux
#define NUM_PATAS  4
#define CANAL_SIN_USAR 3     // canal vacio (3 y 4 libres)

// orden FI, FD, TI, TD -- canal real de cada AS5600 (NO es 0,1,2,3)
// Confirmado en el robot el 30-sep: FI = canal 0 (servo D8), FD = canal 7 (D4),
// TI = canal 1 (D5), TD = canal 2 (D0). El cruce que se vio en las pruebas
// venia de los pines de los servos en la ESP principal, no de estos canales.
const uint8_t CANAL_AS5600[NUM_PATAS] = {0, 7, 1, 2};

// GY-521 y VL53L0X:
//   0 = conectados directo al bus principal (D1/D2), sin pasar por el mux
//   1 = detras del mux, en los canales de abajo
#define SENSORES_NAV_EN_MUX 0
#define CANAL_GY521   5
#define CANAL_VL53L0X 6

// ---------------------------------------------------------------
// Offset y signo por pata (opcional):
//   theta enviada = SIGNO * (angulo_AS5600 - OFFSET), llevada a [0, 360)
// Con OFFSET = 0 y SIGNO = 1 se manda el angulo crudo (como antes).
// ---------------------------------------------------------------
// Valores del comando "cero" (30-sep): con las patas verticales las 4 leen 270°.
// VERIFICAR que coincidan con los que estan cargados en el robot.
float OFFSET_AS5600[NUM_PATAS] = {326.41, 156.01, 334.78, 10.81};
int   SIGNO_AS5600[NUM_PATAS]  = {1, -1, 1, -1};

Adafruit_MPU6050 mpu;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

bool mpuOK = false, loxOK = false, muxPresente = false;
bool as5600OK[NUM_PATAS] = {false, false, false, false};
float omega0 = 0.314;   // rad/s, velocidad angular de referencia de la trayectoria

// ---------------------------------------------------------------
// I2C: multiplexor y AS5600
// ---------------------------------------------------------------
void seleccionarCanalMux(uint8_t canal) {
  Wire.beginTransmission(DIR_MUX);
  Wire.write(1 << canal);
  Wire.endTransmission();
}

bool detectarMux() {
  Wire.beginTransmission(DIR_MUX);
  return (Wire.endTransmission() == 0);
}

// RAW ANGLE (registros 0x0C/0x0D, 12 bits). Devuelve -1 si no responde.
float leerAS5600Activo() {
  Wire.beginTransmission(DIR_AS5600);
  Wire.write(0x0C);
  if (Wire.endTransmission(false) != 0) return -1.0;
  Wire.requestFrom((uint8_t)DIR_AS5600, (uint8_t)2);
  if (Wire.available() < 2) return -1.0;
  uint8_t alto = Wire.read();
  uint8_t bajo = Wire.read();
  uint16_t crudo = (((uint16_t)alto << 8) | bajo) & 0x0FFF;
  return crudo * 360.0 / 4096.0;
}

float aplicarCalibracion(uint8_t p, float crudo) {
  float t = SIGNO_AS5600[p] * (crudo - OFFSET_AS5600[p]);
  while (t < 0) t += 360.0;
  while (t >= 360.0) t -= 360.0;
  return t;
}

void canalNav(uint8_t canal) {
#if SENSORES_NAV_EN_MUX
  seleccionarCanalMux(canal);
#else
  (void)canal;
#endif
}

// ---------------------------------------------------------------
void setup() {
  Serial.begin(BAUD_ENLACE);   // este Serial es el enlace hacia la ESP principal
  delay(200);
  Serial.println();
  Serial.println("=== Diagnostico ESP-Sensores v2 (115200 baudios) ===");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);       // I2C rapido

  muxPresente = detectarMux();
  Serial.print("Multiplexor TCA9548A (0x70): ");
  Serial.println(muxPresente ? "DETECTADO" : "NO DETECTADO -- revisar VCC/GND/RST/SDA/SCL del mux");

  if (muxPresente) {
    const char* nombrePata[NUM_PATAS] = {"FI", "FD", "TI", "TD"};
    for (uint8_t p = 0; p < NUM_PATAS; p++) {
      seleccionarCanalMux(CANAL_AS5600[p]);
      delay(5);
      float a = leerAS5600Activo();
      as5600OK[p] = (a >= 0.0);
      Serial.print("  AS5600 "); Serial.print(nombrePata[p]);
      Serial.print(" (canal "); Serial.print(CANAL_AS5600[p]); Serial.print("): ");
      if (as5600OK[p]) { Serial.print("OK, angulo = "); Serial.println(a, 2); }
      else Serial.println("NO RESPONDE");
    }
    seleccionarCanalMux(CANAL_SIN_USAR);
  }

  canalNav(CANAL_GY521);
  mpuOK = mpu.begin();
  Serial.print("GY-521: ");
  Serial.println(mpuOK ? "OK" : "NO RESPONDE");
  if (mpuOK) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  canalNav(CANAL_VL53L0X);
  loxOK = lox.begin();
  Serial.print("VL53L0X: ");
  Serial.println(loxOK ? "OK (modo continuo)" : "NO RESPONDE");
  if (loxOK) lox.startRangeContinuous();   // mide solo, sin bloquear el ciclo

  if (muxPresente) seleccionarCanalMux(CANAL_SIN_USAR);
  Serial.println("=================================================");
}

// ---------------------------------------------------------------
void loop() {
  // ---- periodo fijo de 20 ms (sin delay) ----
  static unsigned long tUltimo = 0;
  unsigned long ahora = millis();
  if (ahora - tUltimo < PERIODO_ENVIO_MS) return;
  tUltimo += PERIODO_ENVIO_MS;
  if (ahora - tUltimo > PERIODO_ENVIO_MS) tUltimo = ahora;   // si se atraso, no acumula

  // ---- 4x AS5600 (uno por pata, via multiplexor) ----
  float theta[NUM_PATAS];
  for (uint8_t p = 0; p < NUM_PATAS; p++) {
    theta[p] = -1.0;
    if (muxPresente && as5600OK[p]) {
      seleccionarCanalMux(CANAL_AS5600[p]);
      float crudo = leerAS5600Activo();
      if (crudo >= 0.0) theta[p] = aplicarCalibracion(p, crudo);
    }
  }
  if (muxPresente) seleccionarCanalMux(CANAL_SIN_USAR);

  // ---- GY-521: velocidad angular del chasis (psi') ----
  float psi_dot = 0, delta_omega = 0;
  if (mpuOK) {
    canalNav(CANAL_GY521);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    psi_dot = g.gyro.z;
    delta_omega = psi_dot - omega0;
  }

  // ---- VL53L0X: lectura no bloqueante (nueva medicion cada ~50 ms) ----
  static float distancia_m = -1;
  static bool distValida = false;
  if (loxOK) {
    canalNav(CANAL_VL53L0X);
    if (lox.isRangeComplete()) {
      uint16_t mm = lox.readRange();
      distValida = (mm > 0 && mm < 2000);   // fuera de rango -> NA
      if (distValida) distancia_m = mm / 1000.0;
    }
  }
  if (muxPresente) seleccionarCanalMux(CANAL_SIN_USAR);

  // ---- una sola linea hacia la ESP principal ----
  String s;
  s.reserve(140);
  s += "theta1=";       s += theta[0] >= 0 ? String(theta[0], 2) : String("NA");
  s += ",theta2=";      s += theta[1] >= 0 ? String(theta[1], 2) : String("NA");
  s += ",theta3=";      s += theta[2] >= 0 ? String(theta[2], 2) : String("NA");
  s += ",theta4=";      s += theta[3] >= 0 ? String(theta[3], 2) : String("NA");
  s += ",psi_dot=";     s += mpuOK ? String(psi_dot, 4) : String("NA");
  s += ",delta_omega="; s += mpuOK ? String(delta_omega, 4) : String("NA");
  s += ",distancia=";   s += distValida ? String(distancia_m, 4) : String("NA");
  Serial.println(s);
}
