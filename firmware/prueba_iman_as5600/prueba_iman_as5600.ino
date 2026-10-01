/*
  ============================================================================
  PRUEBA DEL IMAN de un AS5600 (via multiplexor) -- para la ESP-Sensores
  Robot Cuadrupedo (ACEBOTT/QD020)
  ============================================================================

  Para que sirve: saber si el iman de una pata esta bien puesto (centrado,
  a buena distancia) o si el problema es mecanico.

  Uso:
    1. Cargar en la ESP-Sensores (la misma placa del multiplexor).
    2. Elegir la pata con PATA (0 = FI, 1 = FD, 2 = TI, 3 = TD).
    3. Desconectar la senal del servo de esa pata (o apagar la ESP principal)
       para poder girar la pata con la mano.
    4. Monitor serie a 115200. Girar la pata LENTO de un extremo al otro.

  Que mirar:
    - "angulo" debe cambiar parejo con el giro: si la pata gira ~60° y el
      angulo cambia 20°, o sube y luego baja, el iman no esta centrado en el eje.
    - "magnitud" debe mantenerse casi constante al girar (variacion < 10 %).
      Si sube y baja mucho con el giro -> iman descentrado.
    - "AGC" ideal cerca de la mitad (con 3.3 V: ~64; con 5 V: ~128).
      Pegado a 0 -> iman muy cerca / muy fuerte. Pegado al maximo -> muy lejos.
    - "estado": MD = iman detectado; ML = campo muy debil; MH = muy fuerte.
      Lo correcto es "MD" solo.
    Al final de cada giro imprime un resumen con el rango y la variacion.

  Hacer la misma prueba en FI (sana) y comparar con FD y TD.
  ============================================================================
*/
#include <Wire.h>

#define PIN_SDA 4   // D2
#define PIN_SCL 5   // D1
#define DIR_MUX    0x70
#define DIR_AS5600 0x36

const uint8_t CANAL_AS5600[4] = {0, 7, 1, 2};   // FI, FD, TI, TD
const char*   NOMBRE[4]       = {"FI", "FD", "TI", "TD"};

#define PATA 1   // <- 0 = FI, 1 = FD, 2 = TI, 3 = TD

void seleccionarCanalMux(uint8_t canal) {
  Wire.beginTransmission(DIR_MUX);
  Wire.write(1 << canal);
  Wire.endTransmission();
}

int leer8(uint8_t reg) {
  Wire.beginTransmission(DIR_AS5600);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  Wire.requestFrom((uint8_t)DIR_AS5600, (uint8_t)1);
  return Wire.available() ? Wire.read() : -1;
}

int leer12(uint8_t reg) {
  Wire.beginTransmission(DIR_AS5600);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  Wire.requestFrom((uint8_t)DIR_AS5600, (uint8_t)2);
  if (Wire.available() < 2) return -1;
  int alto = Wire.read(), bajo = Wire.read();
  return ((alto << 8) | bajo) & 0x0FFF;
}

// estadisticas del giro actual
float angMin = 1e9, angMax = -1e9;
int magMin = 99999, magMax = -1;
unsigned long ultimoMovimiento = 0;
unsigned long vecesDebil = 0;   // lecturas con ML o sin MD durante el giro
float angAnterior = -1;

void setup() {
  Serial.begin(115200);
  delay(200);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  seleccionarCanalMux(CANAL_AS5600[PATA]);
  delay(5);
  Serial.println();
  Serial.print("=== Prueba del iman: pata "); Serial.print(NOMBRE[PATA]);
  Serial.print(" (canal "); Serial.print(CANAL_AS5600[PATA]); Serial.println(") ===");
  Serial.println("Gira la pata lento de un extremo al otro. Resumen tras 2 s quieta.");
  Serial.println("angulo   magnitud  AGC  estado");
}

void loop() {
  int crudo = leer12(0x0C);        // RAW ANGLE
  int mag   = leer12(0x1B);        // MAGNITUDE
  int agc   = leer8(0x1A);         // AGC
  int st    = leer8(0x0B);         // STATUS: bit5 MD, bit4 ML, bit3 MH
  if (crudo < 0) {
    Serial.println("El AS5600 no responde en este canal: revisar cableado/canal.");
    delay(1000);
    return;
  }
  float ang = crudo * 360.0 / 4096.0;

  String estado = "";
  if (st & 0x20) estado += "MD ";
  if (st & 0x10) estado += "ML(debil) ";
  if (st & 0x08) estado += "MH(fuerte) ";
  if (estado.length() == 0) estado = "SIN IMAN";

  Serial.print(ang, 2); Serial.print("   ");
  Serial.print(mag);    Serial.print("      ");
  Serial.print(agc);    Serial.print("   ");
  Serial.println(estado);

  if ((st & 0x10) || !(st & 0x20)) vecesDebil++;
  if (angAnterior >= 0 && fabs(ang - angAnterior) > 0.5) ultimoMovimiento = millis();
  angAnterior = ang;
  angMin = min(angMin, ang); angMax = max(angMax, ang);
  magMin = min(magMin, mag); magMax = max(magMax, mag);

  // resumen cuando la pata lleva 2 s quieta despues de haberse movido
  if (ultimoMovimiento && millis() - ultimoMovimiento > 2000) {
    float varMag = 100.0 * (magMax - magMin) / max(1, magMax);
    Serial.println("-----------------------------------------------");
    Serial.print("Rango de angulo medido: "); Serial.print(angMax - angMin, 1);
    Serial.println(" grados (comparar con lo que giro la pata a la vista)");
    Serial.print("Magnitud: "); Serial.print(magMin); Serial.print(" a "); Serial.print(magMax);
    Serial.print("  -> variacion "); Serial.print(varMag, 0); Serial.println(" %");
    if (angMax - angMin < 20)
      Serial.println("  Casi no giro: girar la pata de un extremo al otro para evaluar el centrado.");
    if (vecesDebil > 0 || magMax < 1500)
      Serial.println("  CAMPO DEBIL (ML / AGC al maximo): iman muy lejos del chip o no es diametral.");
    else if (varMag > 15)
      Serial.println("  La magnitud cambia mucho al girar: IMAN DESCENTRADO respecto al eje.");
    else if (angMax - angMin >= 20)
      Serial.println("  Iman bien: campo suficiente y estable en todo el giro.");
    Serial.println("-----------------------------------------------");
    angMin = 1e9; angMax = -1e9; magMin = 99999; magMax = -1;
    ultimoMovimiento = 0; vecesDebil = 0;
  }
  delay(100);
}
