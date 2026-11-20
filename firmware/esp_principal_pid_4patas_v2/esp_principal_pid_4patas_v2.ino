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
      ganancias=aire | marcha     carga un juego completo (al encender: marcha)
      kp=0.8   ki=12   kd=0.008   ganancias de las 4 patas (reinicia el integrador)
      kp3=0.3  ki3=4   kd3=0      ganancias de UNA pata (1..4 = FI, FD, TI, TD)
      imax3=8                     tope de la correccion integral de una pata (grados)
      banda3=1.5                  zona muerta del integrador de una pata (grados):
                                  con |error| menor no integra (evita que la holgura
                                  del montaje haga "cazar" a la pata). banda=x -> las 4
      cal1=193.3,0.655            calibracion de una pata en vivo (cal1..cal4):
                                  theta = THETA_W0 + GRADOS_POR_WRITE * write
                                  (la genera el comando "calibrar" de Python)
      tab1=v40,v50,...,v140       tabla de linealizacion del AS5600 (11 valores)
      tab1=off                    desactiva la tabla (lectura cruda)
                                  (las genera el comando "linealizar" de Python)
      marcha=1 / marcha=0         enciende / detiene el trote diagonal (rampa de 1 s)
      amp=10  damp=0  frec=1      amplitud (°), diferencia izq/der (°) y frecuencia (Hz)
      sentido=-1  centro=270      invierte el avance / angulo central de las rodillas
      cen2=5  esc2=1.3            por pata: corre el centro (°) / multiplica la amplitud
      modo=pid                    lazo cerrado
      modo=abierto                solo feedforward (por defecto al encender): sirve para
                                  verificar la calibracion y para comparar
      --- 16-nov, caracterizacion de sensores (docs/Caracterizacion_de_sensores.xlsx) ---
      mapa=1 / mapa=0             usa / no usa la tabla de ANGULO REAL de cada rodilla
                                  (con mapa=1, ref= y theta son grados reales del transportador)
      real1=v220,...,v310         tabla de angulo real de una pata (10 valores: el angulo
                                  real cuando el sensor marca 220, 230, ..., 310)
      escgiro=1.0165              escala del giroscopio (angulo real / angulo leido)
      lasera=0.9721  laserb=-1.047  recta del laser: cm reales = a*lectura cruda + b
      vel=4                       velocidad de avance (cm/s): al encender la orbita se calcula
                                  wref = vel / radio para el dref pedido (wref= lo fija a mano)
      kuff=0.0478                 prealimentacion: grados/s que gira el robot por cada grado de
                                  ddesf (ddesf de partida = wref/kuff). kuff=0 -> u0 fijo, como antes.
                                  Con signo: negativo si ddesf<0 hace girar a la izquierda (+)
      orbsentido=-1 / 1           orbita horaria (adelante, tubo a la derecha) / antihoraria
                                  (hacia atras con desforb=45 y el laser a la derecha)
      control=1 / control=0       lazo de distancia ajustado con el modelo (kpe 0.3, kde 5,
                                  kie 0.03, wlim 5) / el del 28-oct (0.14, 2, 0.02, 3)
      epscorr=0 / 1               parada de la orbita solo con el giroscopio / con la
                                  correccion por acercarse o alejarse del tubo
  Protocolo ESP -> Python (igual que antes, 17 campos por linea):
      t_ms, ref1,theta1,error1,write1, ..., ref4,theta4,error4,write4,
      giro_dps, phi_deg, dist_cm   (20 campos desde el 7-oct)
      damp                         (21 campos desde el 9-oct)
      ddesf                        (22 campos desde el 21-oct)
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
// 19-oct: A0 no se usa; el ADC mide el voltaje de alimentacion de esta ESP (3.3 V
// despues del regulador). Si baja de ~3.0 V cuando los servos tiran corriente, la
// bateria compartida no alcanza. 'estado' muestra el minimo de cada 2 s.
ADC_MODE(ADC_VCC);
#if __has_include(<core_version.h>)
#include <core_version.h>           // ARDUINO_ESP8266_MAJOR (nucleo 3.x)
#endif

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
// Valores medidos con "linealizar" (1-oct 23:38) y "vertical" (2-oct 00:02):
float THETA_W0[NUM_PATAS]         = {211.800, 336.600, 216.000, 324.000};
float GRADOS_POR_WRITE[NUM_PATAS] = {0.6000, -0.6000, 0.6000, -0.6000};

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
// tablas medidas con "linealizar" el 2026-10-01 23:38 (se pueden reemplazar con tab1..4=)
float tabla[NUM_PATAS][N_TABLA] = {
  {274.19, 276.07, 277.88, 279.95, 282.46, 285.21, 288.25, 291.39, 294.89, 298.53, 301.92},   // FI
  {311.20, 308.74, 305.86, 301.92, 297.63, 293.19, 287.95, 282.77, 276.76, 268.36, 261.02},   // FD
  {273.50, 277.36, 281.82, 286.00, 289.95, 293.71, 297.70, 301.69, 306.20, 310.41, 314.91},   // TI
  {203.86, 197.90, 191.51, 185.68, 179.95, 173.06, 166.64, 162.20, 155.00, 149.19, 142.38}   // TD
};
bool  tablaOK[NUM_PATAS] = {true, true, true, true};

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

// ---------------------------------------------------------------
// ANGULO REAL de cada rodilla (16-nov, caracterizacion con transportador)
// La tabla de arriba deja la lectura del AS5600 en "grados nominales del servo"
// (theta = THETA_W0 + 0.6 * write). Pero 0.6 grados por unidad de write es el
// valor de catalogo: medido con el transportador, cada rodilla gira entre 0.50 y
// 0.56 grados por unidad, y con ref=270 no queda vertical:
//      FI 268.0   FD 273.5   TI 271.0   TD 275.0
// y con amp=20 las patas recorrian +-18.5, +-17.0, +-18.8 y +-17.5 grados.
// realTab guarda, para cada pata, el angulo REAL cuando el sensor marca
// 220, 230, ..., 310 (10 pruebas por punto, modo pid). Con mapa=1:
//   - la lectura se pasa a grados reales antes del PID y de la telemetria;
//   - la referencia (grados reales) se pasa a "grados del sensor" antes de
//     calcular el write. Asi ref=270 es vertical de verdad en las 4 patas y
//     amp=20 son 20 grados reales en las 4, en modo pid Y en modo abierto.
// La tabla depende del servo y de como quedo montado el horn (no del AS5600):
// 'recalibrar N' no la cambia, pero si se cambia un servo o se mueve el horn hay
// que volver a medirla (real1..4=) o corregir con cen1..4=.
// Solo se usa si la pata tiene su tabla de linealizacion activa: los comandos de
// calibracion de la PC (que mandan tabN=off) siguen trabajando en grados nominales.
// ---------------------------------------------------------------
#define N_REAL 10
const float REAL_S0 = 220.0, REAL_PASO = 10.0;   // el sensor marca 220, 230, ..., 310
float realTab[NUM_PATAS][N_REAL] = {
  {221.5, 233.5, 242.0, 250.0, 257.0, 268.0, 277.0, 287.0, 295.0, 305.0},   // FI
  {241.0, 242.5, 246.5, 256.0, 265.0, 273.5, 280.5, 290.0, 299.0, 306.5},   // FD
  {225.0, 233.5, 243.0, 252.0, 263.0, 271.0, 281.0, 289.5, 300.0, 309.0},   // TI
  {232.0, 240.0, 250.0, 260.0, 268.0, 275.0, 285.0, 295.0, 302.0, 312.0}    // TD
};
bool mapaReal = true;
bool usaMapa(uint8_t p) { return mapaReal && tablaOK[p]; }

// grados del sensor -> grados reales (fuera de la tabla sigue con pendiente 1)
float realDesdeSensor(uint8_t p, float sdeg) {
  if (!usaMapa(p)) return sdeg;
  const float* t = realTab[p];
  float x = (sdeg - REAL_S0) / REAL_PASO;
  if (x <= 0.0) return t[0] + (sdeg - REAL_S0);
  if (x >= N_REAL - 1) return t[N_REAL - 1] + (sdeg - (REAL_S0 + REAL_PASO * (N_REAL - 1)));
  int i = (int)x;
  return t[i] + (t[i + 1] - t[i]) * (x - i);
}

// grados reales -> grados del sensor (para calcular el write).
// En los extremos puede haber tramos casi planos (FD por debajo de 246: la pata
// llega a su tope y el sensor sigue cambiando). Invertir ahi multiplicaria por 6
// cualquier correccion, asi que esos tramos del borde no se invierten: desde el
// primer tramo "sano" (mas de 0.5 grados reales por grado de sensor) hacia afuera
// se sigue con pendiente 1.
float sensorDesdeReal(uint8_t p, float rdeg) {
  if (!usaMapa(p)) return rdeg;
  const float* t = realTab[p];
  int i0 = 0, i1 = N_REAL - 1;
  while (i0 < N_REAL - 2 && (t[i0 + 1] - t[i0]) < 0.5 * REAL_PASO) i0++;
  while (i1 > i0 + 1 && (t[i1] - t[i1 - 1]) < 0.5 * REAL_PASO) i1--;
  if (rdeg <= t[i0]) return REAL_S0 + REAL_PASO * i0 + (rdeg - t[i0]);
  if (rdeg >= t[i1]) return REAL_S0 + REAL_PASO * i1 + (rdeg - t[i1]);
  int i = i0;
  while (i < i1 - 1 && rdeg > t[i + 1]) i++;
  float d = t[i + 1] - t[i];
  float f = (d > 1e-3) ? (rdeg - t[i]) / d : 0.0;
  return REAL_S0 + REAL_PASO * (i + f);
}

// Limites del write de las rodillas. Antes 40..140; se amplio a 10..170 para que la
// marcha pueda dar pasos mas grandes (FD esta vertical en write 111 y con 40..140
// no podia pasar de unos 17°). El codigo basico de ACEBOTT usa 0..180 sin choques.
#define WRITE_MIN   10
#define WRITE_MAX  170

// ---------------------------------------------------------------
// PID (angulos en grados; Kp adimensional, Ki en 1/s, Kd en s)
// ---------------------------------------------------------------
// Ganancias POR PATA (FI, FD, TI, TD): las izquierdas tienen mas holgura en el
// montaje del AS5600 y necesitan ganancias mas suaves que las derechas.
// Dos juegos de ganancias (comando ganancias=aire | ganancias=marcha):
//  aire:   escalon con la pata al aire (2-oct), sigue la referencia en ~0.2 s
//  marcha: solo integral, para caminar (4-oct). A 3 Hz el integrador casi no ve la
//          oscilacion: corrige el valor MEDIO de cada pata (lo que cede con el peso),
//          como un "cen" automatico. FD sin correccion (ki2=0). Con este juego el
//          robot avanzo mucho mas que en lazo abierto. TI tambien sin correccion (6-oct):
//          con carga su AS5600 se satura (207/333) y el integrador deformaba su paso.
// Al encender se carga el de marcha.
float KP[NUM_PATAS], KI[NUM_PATAS], KD[NUM_PATAS], BANDA_I[NUM_PATAS], IMAX[NUM_PATAS];

void cargarGanancias(bool marcha) {
  const float kpA[NUM_PATAS] = {0.5, 0.8, 0.4, 0.8},  kiA[NUM_PATAS] = {6.0, 12.0, 5.0, 0.1};
  const float kdA[NUM_PATAS] = {0.0, 0.008, 0.0, 0.008}, bA[NUM_PATAS] = {1.0, 0.5, 1.0, 0.5};
  const float kiM[NUM_PATAS] = {3.0, 0.0, 0.0, 3.0};   // FD y TI sin correccion (sensor de TI falla con carga)
  for (uint8_t p = 0; p < NUM_PATAS; p++) {
    KP[p]      = marcha ? 0.0 : kpA[p];
    KI[p]      = marcha ? kiM[p] : kiA[p];
    KD[p]      = marcha ? 0.0 : kdA[p];
    BANDA_I[p] = marcha ? 1.5 : bA[p];
    IMAX[p]    = marcha ? 15.0 : 30.0;     // grados: tope de la correccion integral
  }
}
const float TS = 0.020;                 // s, periodo fijo de control (50 Hz)
const unsigned long PERIODO_CONTROL_MS = 20;
const float TF = TS;                    // s, filtro de la derivada

// coeficientes de la ecuacion en diferencias (se recalculan si cambian las ganancias)
float c_i[NUM_PATAS], c2[NUM_PATAS], c1;
void calcularCoeficientes() {
  c1 = (2.0 * TF - TS) / (2.0 * TF + TS);
  for (uint8_t p = 0; p < NUM_PATAS; p++) {
    c_i[p] = KI[p] * TS / 2.0;                 // integral por Tustin
    c2[p]  = 2.0 * KD[p] / (2.0 * TF + TS);    // derivada filtrada por Tustin
  }
}

// Modelo de referencia: servo nominal wn = 30 rad/s, zeta = 1, discretizado
// con retenedor de orden cero a Ts = 20 ms:
//   ym_k = 1.09762*ym_(k-1) - 0.30119*ym_(k-2) + 0.12190*r_(k-1) + 0.08167*r_(k-2)
const float RM_A1 =  1.09762327, RM_A2 = -0.30119421;
const float RM_B1 =  0.12190138, RM_B2 =  0.08166956;

// estado por pata
float referencia[NUM_PATAS] = {270.0, 270.0, 270.0, 270.0};   // al encender: rodillas verticales
float integral[NUM_PATAS], derivada[NUM_PATAS];
float e_prev[NUM_PATAS], y_prev[NUM_PATAS];
float ym1[NUM_PATAS], ym2[NUM_PATAS];   // salida del modelo de referencia
float r1[NUM_PATAS], r2[NUM_PATAS];     // referencias anteriores
bool  inicializado[NUM_PATAS] = {false, false, false, false};
int   writeActual[NUM_PATAS];
bool  modoPID = false;   // arranca en lazo abierto: el PID solo se activa con modo=pid (tras cargar la calibracion)
unsigned long ultimoCicloControl = 0;

// ---------------------------------------------------------------
// Generador de marcha: trote diagonal (modelo, seccion 8.5 y 13)
//   par 1 = FI + TD (fase 0), par 2 = FD + TI (fase 180°)
//   ref_i = CENTRO + SENTIDO * A_lado * sin(2*pi*FREC*t + fase_i)
//   A_izq = AMP - DAMP/2 (FI, TI),  A_der = AMP + DAMP/2 (FD, TD)
// AMP controla la velocidad de avance y DAMP el giro (entradas de las
// plantas G_v y G_w que se identifican con el robot).
// La amplitud sube y baja en rampa para no dar golpes al arrancar/parar.
// ---------------------------------------------------------------
bool  marchaActiva = false;

// ---------------------------------------------------------------
// ARRANQUE AUTOMATICO (sin WiFi): al encender, espera ESPERA_ARRANQUE_MS con las
// rodillas verticales, activa el PID con las ganancias de marcha y camina
// DURACION_MARCHA_S segundos; luego se detiene. Si no llegan datos de la
// ESP-Sensores, camina en lazo abierto. Cualquier comando marcha= o modo= por
// WiFi cancela el arranque automatico. false = se maneja solo por WiFi.
// ---------------------------------------------------------------
const bool ARRANQUE_AUTOMATICO = false;   // 22-oct: ya no camina solo al encender (true = camina 30 s a los 5 s)
const unsigned long ESPERA_ARRANQUE_MS = 5000;   // tiempo para soltarlo en el piso
const float DURACION_MARCHA_S = 30.0;
uint8_t estadoAuto = 0;                          // 0 esperando, 1 caminando, 2 terminado
unsigned long tInicioAuto = 0;
float AMP = 20.0;          // grados. 10 = paso corto (camino 1.22 m); 20 = paso mas largo
float DAMP = 0.0;          // grados (positivo: patas derechas con mas amplitud)
float FREC = 3.0;          // Hz (ciclos de paso por segundo); 3 Hz es la que mejor camino
float CENTRO = 270.0;      // grados (270 = pata vertical)
int   SENTIDO = 1;         // +1 / -1: invierte el sentido de avance
const float RAMPA_S = 1.0; // s para llegar a la amplitud completa
// Fase de cada pata (rad). 20-nov: ya no es constante, para poder probar otros
// pasos (comando andar= o fase1..4= en grados):
//   trote     (diagonales juntas: FI+TD y FD+TI)      0, 180, 180,   0  (por defecto)
//   ambladura (mismo lado juntas: FI+TI y FD+TD)      0, 180,   0, 180  (= trote con desfase=180)
//   salto     (delanteras juntas y traseras juntas)    0,   0, 180, 180  ("bound")
//   brinco    (las cuatro juntas)                      0,   0,   0,   0  ("pronk")
// desfase= y ddesf= se siguen sumando a las traseras encima de estas fases.
float FASE_PATA[NUM_PATAS] = {0.0, PI, PI, 0.0};   // FI, FD, TI, TD
const bool  PATA_TRASERA[NUM_PATAS] = {false, false, true, true};
// desfase= (grados): atrasa las traseras respecto a las delanteras. Con 0 el
// trote es simetrico en el tiempo y "sentido" no cambia nada (invertir el seno
// es lo mismo que intercambiar los pares); con +-60..90 aparece una onda
// adelante->atras y sentido=1/-1 elige hacia donde camina.
float DESFASE = 0.0;
// ddesf= (grados, 20-oct): diferencia de desfase entre lados, como damp con la
// amplitud. TI usa DESFASE + DDESF/2 y TD usa DESFASE - DDESF/2. En las pruebas
// del 20-oct desfase=+90 camino hacia adelante (y mas rapido) y desfase=-90 hacia
// atras: con desfase=0 y ddesf=180 un lado empuja hacia adelante y el otro hacia
// atras -> el robot gira sobre si mismo.
float DDESF = 0.0;
const bool  PATA_IZQ[NUM_PATAS]  = {true, false, true, false};
// Ajuste por pata (para compensar una pata que apoya menos o se mueve menos):
//   cen1..4=  corrimiento del centro de esa pata (grados, se suma a CENTRO)
//   esc1..4=  factor de amplitud de esa pata (1 = igual que las demas)
float CEN_PATA[NUM_PATAS] = {0.0, 0.0, 0.0, 0.0};
float ESC_PATA[NUM_PATAS] = {1.0, 1.0, 1.0, 1.0};
float faseMarcha = 0.0;    // rad
float escalaMarcha = 0.0;  // 0..1 (rampa)

// Tiempo REAL entre ciclos de control. Si el lazo se atrasa (WiFi ocupado), el ciclo
// de 20 ms se salta; antes la fase de la marcha y el angulo del giroscopio avanzaban
// un TS fijo por ciclo y quedaban cortos (marcha mas lenta que FREC, giro a la mitad).
float dtCiclo = 0.020;            // s
unsigned long ciclosControl = 0;  // para el diagnostico (ciclos por segundo)

float desfasePata(uint8_t p) {
  if (!PATA_TRASERA[p]) return 0.0;
  float d = PATA_IZQ[p] ? DESFASE + DDESF / 2.0 : DESFASE - DDESF / 2.0;
  return d * PI / 180.0;
}

void pasoMarcha() {
  float objetivo = marchaActiva ? 1.0 : 0.0;
  float d = dtCiclo / RAMPA_S;
  if (escalaMarcha < objetivo) escalaMarcha = min(objetivo, escalaMarcha + d);
  else if (escalaMarcha > objetivo) escalaMarcha = max(objetivo, escalaMarcha - d);
  if (escalaMarcha <= 0.0 && !marchaActiva) return;      // detenida: no toca las referencias
  faseMarcha += 2.0 * PI * FREC * dtCiclo;
  if (faseMarcha > 2.0 * PI) faseMarcha -= 2.0 * PI;
  for (uint8_t p = 0; p < NUM_PATAS; p++) {
    float a = PATA_IZQ[p] ? AMP - DAMP / 2.0 : AMP + DAMP / 2.0;
    referencia[p] = CENTRO + CEN_PATA[p] +
                    SENTIDO * escalaMarcha * ESC_PATA[p] * a *
                    sin(faseMarcha + FASE_PATA[p] + desfasePata(p));
  }
}

// ---------------------------------------------------------------
// Conversiones con la calibracion de cada pata
// ---------------------------------------------------------------
int writeDesdeTheta(uint8_t p, float thetaDeg) {
  // thetaDeg viene en grados reales (si mapa=1): primero a grados del sensor
  float w = (sensorDesdeReal(p, thetaDeg) - THETA_W0[p]) / GRADOS_POR_WRITE[p];
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

// ---------------------------------------------------------------
// Filtro de lecturas imposibles (robot apoyado en el piso)
// Con carga el iman fuera del eje puede dar lecturas absurdas (ej. 470°):
//  - fuera del rango alcanzable (write 40..140) con un margen -> se descarta
//  - salto mayor que MAX_SALTO en 20 ms (el MG90S gira ~12° en 20 ms) -> se descarta
// Una lectura descartada se reemplaza por la ultima valida, asi el PID no da
// golpes. Si hay mas de MAX_DESCARTES seguidas, se acepta la nueva (puede
// que la pata realmente este ahi).
// ---------------------------------------------------------------
const float MAX_SALTO = 20.0;          // grados por muestra
const float MARGEN_RANGO = 15.0;       // grados fuera del rango alcanzable
const uint8_t MAX_DESCARTES = 3;
float thetaValida[NUM_PATAS];
bool  hayValida[NUM_PATAS] = {false, false, false, false};
uint8_t descartesSeguidos[NUM_PATAS] = {0, 0, 0, 0};
// 22-oct: el sensor de FI se cayo; su lectura quedo fuera de rango y el PID llevo la
// rodilla al tope (servo forzado, mas corriente, ESP inestable). Ahora cada pata
// lleva un nivel de "lecturas malas" (+3 por mala, -1 por buena, 0..150): con 90 o
// mas la pata pasa sola a LAZO ABIERTO; vuelve al PID cuando el nivel baja a 0.
uint8_t nivelMalas[NUM_PATAS] = {0, 0, 0, 0};
bool sensorFalla[NUM_PATAS] = {false, false, false, false};
void mensajeWifi(const String &m);   // definida mas abajo
const char *NOMBRE_PATA[NUM_PATAS] = {"FI", "FD", "TI", "TD"};
unsigned long lecturasDescartadas[NUM_PATAS] = {0, 0, 0, 0};

float filtrarLectura(uint8_t p, float y) {
  float a = THETA_W0[p] + GRADOS_POR_WRITE[p] * WRITE_MIN;
  float b = THETA_W0[p] + GRADOS_POR_WRITE[p] * WRITE_MAX;
  float lo = min(a, b) - MARGEN_RANGO, hi = max(a, b) + MARGEN_RANGO;
  bool fueraRango = tablaOK[p] && (y < lo || y > hi);
  bool salto = hayValida[p] && fabs(y - thetaValida[p]) > MAX_SALTO;
  if (fueraRango || salto) nivelMalas[p] = min(150, nivelMalas[p] + 3);
  else if (nivelMalas[p] > 0) nivelMalas[p]--;
  if (!sensorFalla[p] && nivelMalas[p] >= 90) {
    sensorFalla[p] = true;
    mensajeWifi(String("AVISO: el sensor de ") + NOMBRE_PATA[p] + " da lecturas fuera de rango: " +
                NOMBRE_PATA[p] + " pasa a lazo abierto. Revisar el sensor y correr 'recalibrar " + String(p + 1) + "'");
  } else if (sensorFalla[p] && nivelMalas[p] == 0) {
    sensorFalla[p] = false;
    mensajeWifi(String("OK el sensor de ") + NOMBRE_PATA[p] + " volvio a leer bien: " + NOMBRE_PATA[p] + " vuelve al PID");
  }
  if ((fueraRango || salto) && hayValida[p] && descartesSeguidos[p] < MAX_DESCARTES) {
    descartesSeguidos[p]++;
    lecturasDescartadas[p]++;
    return thetaValida[p];                       // mantener la ultima buena
  }
  if (fueraRango) y = constrain(y, lo, hi);      // nunca pasar al PID algo absurdo
  descartesSeguidos[p] = 0;
  thetaValida[p] = y;
  hayValida[p] = true;
  return y;
}

// contadores para el diagnostico
unsigned long bytesSensores = 0, lineasSensores = 0, lineasSinTheta = 0;
String ultimaLineaSensores = "";

// ---------------------------------------------------------------
// Sensores de navegacion (vienen en la misma linea de la ESP-Sensores):
//   psi_dot   = giroscopio z del MPU6050 (rad/s)  -> velocidad de giro del robot
//   distancia = VL53L0X (m)                        -> distancia al tubo
// El giro se integra aqui para tener el angulo girado PHI (grados). El sesgo
// del giroscopio se estima al encender, en los primeros 3 s (robot quieto).
// Comando phi=0 pone el angulo en cero.
// ---------------------------------------------------------------
// El MPU6050 esta montado con el eje z hacia abajo: al girar a la izquierda daba
// negativo. SIGNO_GIRO = -1 deja positivo = giro a la izquierda (antihorario).
const float SIGNO_GIRO = -1.0;
// 16-nov, caracterizacion (4 angulos x 15 giros a mano contra el transportador):
// el angulo integrado leia 89.3 / 177.0 / 265.1 / 354.3 en 90 / 180 / 270 / 360, es
// decir lectura = 0.9838 * real (R2 = 0.999). El MPU6050 tiene +-3 % de tolerancia de
// escala. Corregido con 1/0.9838 el error medio queda en menos de 1 grado en los 4
// angulos. Sin esto la orbita de 360 se pasaba unos 6 grados. Comando escgiro=.
float ESCALA_GIRO = 1.0165;
float psiDotDeg = 0.0;            // °/s, ya sin sesgo y con la escala corregida
float sesgoGiro = 0.0;            // °/s
float phiDeg = 0.0;               // ° girados (positivo = antihorario visto desde arriba)
float distanciaCm = -1.0;         // cm (-1 = sin dato)
// cm reales = A*lectura cruda + B. 9-oct (laser al lado izquierdo): 0.891 y 0.50.
// 16-nov, caracterizacion con el laser ya al lado derecho (5 distancias x 15
// pruebas): con esa recta leia 24.2 / 19.9 / 15.5 / 10.6 / 5.9 en 25 / 20 / 15 / 10 /
// 5 cm (lectura = 0.9165*real + 1.46). Compuesta con la anterior:
//   A = 0.891/0.9165 = 0.9721,  B = (0.50 - 1.46)/0.9165 = -1.047
// y el error medio queda en +-0.3 cm de 5 a 25 cm. Comandos lasera= y laserb=.
float LASER_A = 0.9721, LASER_B = -1.047;
bool  hayGiro = false;
unsigned long nSesgo = 0;
double sumaSesgo = 0.0;
// El sesgo se mide en los primeros 2 s DESDE QUE LLEGA EL PRIMER DATO del giroscopio
// (antes era desde el arranque de esta ESP: si la ESP-Sensores tardaba en arrancar
// no se media nada y el angulo se iba solo). sesgo=1 lo vuelve a medir (robot quieto).
void mensajeWifi(const String &m);   // definidas mas abajo
void loopInterno();
void sumarSeccion(uint8_t i, unsigned long d);

const unsigned long T_SESGO_MS = 2000;
unsigned long tInicioSesgo = 0;   // 0 = aun no llega el giroscopio
bool avisoSesgoPendiente = false;
float sesgoBueno = 0.0;           // ultimo sesgo creible (menos de 5 grados/s)
bool midiendoSesgo() { return tInicioSesgo == 0 || millis() - tInicioSesgo < T_SESGO_MS; }

// Desde el 9-oct la ESP-Sensores manda yaw_dot = giro alrededor de la vertical
// (proyeccion sobre la gravedad, ya positivo = antihorario), que no depende de
// como quedo montado el MPU. Si no llega (ESP-Sensores vieja) se usa psi_dot (eje z).
bool  usaYawDot = false;
float incMpu = -1.0;              // ° entre el eje z del MPU y la vertical
long  reiniciosLaser = -1;        // veces que la ESP-Sensores reinicio el VL53L0X (-1 = version vieja)
long  reiniciosMpuSens = -1, recupBusSens = -1;
unsigned long tUltimoGiro = 0;    // millis del ultimo dato valido del giroscopio

void leerNavegacion(const String &linea) {
  float yd = parsearCampo(linea, "yaw_dot=");
  float pd = parsearCampo(linea, "psi_dot=");
  float im = parsearCampo(linea, "inc_mpu=");
  if (im > VALOR_NO_DISPONIBLE + 1.0) incMpu = im;
  float lr = parsearCampo(linea, "lox_rst=");
  if (lr > VALOR_NO_DISPONIBLE + 1.0) reiniciosLaser = (long)lr;
  float mr = parsearCampo(linea, "mpu_rst=");
  if (mr > VALOR_NO_DISPONIBLE + 1.0) reiniciosMpuSens = (long)mr;
  float br = parsearCampo(linea, "bus_rst=");
  if (br > VALOR_NO_DISPONIBLE + 1.0) recupBusSens = (long)br;
  float gdeg = 0.0;
  bool hay = false;
  if (yd > VALOR_NO_DISPONIBLE + 1.0)      { gdeg = yd * 57.2958; usaYawDot = true; hay = true; }
  else if (pd > VALOR_NO_DISPONIBLE + 1.0) { gdeg = SIGNO_GIRO * pd * 57.2958; usaYawDot = false; hay = true; }
  if (hay && fabs(gdeg) > 520.0) hay = false;      // fuera del rango del MPU (±500 °/s): dato danado
  if (hay) tUltimoGiro = millis();
  if (hay) {
    if (tInicioSesgo == 0) tInicioSesgo = millis();
    if (midiendoSesgo()) {
      sumaSesgo += gdeg; nSesgo++; sesgoGiro = sumaSesgo / nSesgo;
      avisoSesgoPendiente = true;
    } else if (avisoSesgoPendiente) {             // termino la medicion: revisar que sea creible
      avisoSesgoPendiente = false;
      if (fabs(sesgoGiro) > 5.0) {
        mensajeWifi("AVISO: sesgo medido = " + String(sesgoGiro, 1) +
                    " grados/s (normal: menos de 3): el robot se movio. Se descarta y se usa " +
                    String(sesgoBueno, 2) + "; se corrige solo con el robot quieto unos segundos");
        sesgoGiro = sesgoBueno;
      } else {
        sesgoBueno = sesgoGiro;
        mensajeWifi("OK sesgo del giroscopio = " + String(sesgoGiro, 2) + " grados/s");
      }
    } else if (!marchaActiva && escalaMarcha <= 0.0 && fabs(gdeg - sesgoGiro) < 6.0) {
      // 14-oct: robot detenido -> el sesgo se sigue ajustando despacio (constante de 5 s).
      // Asi un sesgo mal medido (se movio al encender) se corrige solo en ~10 s quieto.
      // Giros a mano (> 6 grados/s) no lo afectan.
      sesgoGiro += (gdeg - sesgoGiro) * (0.02 / 5.0);
      if (fabs(sesgoGiro) < 5.0) sesgoBueno = sesgoGiro;
    }
    psiDotDeg = (gdeg - sesgoGiro) * ESCALA_GIRO;
    hayGiro = true;
  }
  float d = parsearCampo(linea, "distancia=");
  // calibracion contra regla (ver LASER_A y LASER_B)
  distanciaCm = (d > VALOR_NO_DISPONIBLE + 1.0) ? LASER_A * d * 100.0 + LASER_B : -1.0;
}

// ---------------------------------------------------------------
// Orbita alrededor del tubo (21-oct): control en CASCADA.
// Prueba del 21-oct con un solo lazo de distancia: el laser lateral solo ve el
// tubo (11.5 cm) si el robot va casi tangente (mas de ~16 grados de error de
// rumbo y el haz pasa de largo y lee el fondo, 35-47 cm). El lazo veia saltos de
// distancia, ddesf iba de 0 a 240 y el rumbo se perdia mas.
//   Lazo interno (rapido, giroscopio): lleva la velocidad de giro filtrada (1 s,
//     quita el vaiven de 3 Hz del trote) a w_obj moviendo u (ddesf o damp):
//       u = U0 + Iw + KPW*(w_obj - w),  Iw += KIW*(w_obj - w)*dt
//   Lazo externo (lento, laser): solo cuando la distancia es creible (3..DMAX cm
//     y sin saltos de mas de 8 cm), corrige w_obj alrededor de WREF:
//       w_obj = WREF + sgn(WREF)*limite(KPE*e + KDE*de/dt + Ie, +-WLIM)
//     Sin dato del laser, w_obj = WREF (+Ie): sigue el circulo y el haz vuelve.
//   WREF = v/R (R = 5.75 + dref + 6 cm): con v ~ 4 cm/s y dref 15, ~8.5 grados/s.
//   Ie (KIE) corrige despacio un WREF mal estimado.
// Simulacion con haz que se pierde, vaiven y giro de la marcha variable:
// modelado/orbita_cascada_sim.py.
// ---------------------------------------------------------------
bool  orbitaActiva = false;
float DREF = 15.0;          // cm, distancia lateral al tubo (10..25)
float PHI_OBJ = 360.0;      // grados a recorrer alrededor del tubo
uint8_t ACTUADOR = 1;       // 0 = damp, 1 = ddesf
//                  damp    ddesf
// 25-oct: pruebas de marcha (20 s, lazo abierto): desfase=-45 con ddesf=-180 avanza
// hacia ADELANTE girando horario -8.6 grados/s en un circulo que se cierra (el mas
// cerrado de todos); con el laser a la derecha el tubo queda dentro del circulo.
// desfase=0/+45 con ddesf=+180 giraba +8.7..9.4 grados/s pero hacia atras y en un
// circulo grande (la orbita se abria aun con ddesf al tope). Por defecto: el horario.
float U0[2]     = {-12.0, -180.0};   // valor de partida
float UMIN[2]   = {-30.0, -240.0};
float UMAX[2]   = { 30.0,    0.0};
float WREF = -8.5;          // grados/s; negativo = horario (avanza con el tubo a la derecha)
float KPW = 4.0, KIW = 6.0; // lazo interno: unidades de u por grado/s (y por grado)
float KPE = 0.14, KDE = 2.0, KIE = 0.02;   // lazo externo: grados/s por cm, por cm/s, por cm*s
float WLIM = 3.0;           // grados/s, maxima correccion por distancia
float IELIM = 4.0;          // grados/s, tope de la integral de distancia
float DMAX_ORB = 35.0;      // cm, mas lejos = el haz no esta en el tubo
float DESFASE_ORBITA = -45.0; // desfase base durante la orbita con ddesf (desforb=)
// 16-nov: modelo de la marcha en la orbita (modelado/orbita_sensores_sim.py).
//   VEL_AVANCE: velocidad de avance (cm/s). Al encender la orbita, wref = vel/R con
//     R = 5.75 + dref + 6: asi wref corresponde al dref pedido (antes era -8.5 fijo,
//     que solo vale para dref = 15). Si se escribe wref= a mano, se respeta.
//   KUFF: grados/s que gira el robot por cada grado de ddesf (25-oct: ddesf = -180
//     giraba -8.6 grados/s -> 0.0478). Prealimentacion: ddesf parte de (giro pedido /
//     KUFF) y el PI del lazo interno solo corrige lo que falte. Antes partia siempre
//     de u0 = -180 y el PI tardaba varios segundos en llegar al valor de otro dref: en
//     la simulacion eso abria la orbita y el laser perdia el tubo. kuff=0 lo desactiva.
// Los dos valores son del 25-oct: volver a medirlos (README, pruebas M1 a M3) porque
// cambiaron el servo de TI, el peso (395.2 g) y la amplitud real de las patas.
float VEL_AVANCE = 4.0;
float KUFF = 0.0478;
bool  wrefManual = false;
// 19-nov: sentido de la orbita visto desde arriba. -1 = horario (tubo a la derecha
// del robot caminando HACIA ADELANTE, como el 25-oct). +1 = antihorario: es el caso
// del robot caminando HACIA ATRAS con el laser a la derecha (desforb=45): el tubo
// queda a la izquierda del sentido de avance. Comando orbsentido=1|-1.
float SENTIDO_ORB = -1.0;
float phiInicioOrbita = 0.0;
float distFilt = -1.0, dDistFilt = 0.0;
float wFilt = 0.0, iW = 0.0, iE = 0.0, wObj = 0.0, uOrbita = 0.0;
float tSinLaser = 0.0;
bool  laserOK = false;
// 25-oct: angulo ALREDEDOR DEL TUBO. El giroscopio mide cuanto giro el robot
// (su rumbo), que solo coincide con el angulo recorrido alrededor del tubo si
// el robot va tangente. Si se aleja o se acerca (de/dt), su rumbo esta torcido
// eps = asin(dd/dt / v) respecto a la tangente, con v = |w| * R y
// R = 5.75 + d + 6 cm. Angulo alrededor del tubo = phi - (eps - eps_inicio).
// Con 1 cm/s de acercamiento a ~4 cm/s, eps ~ 14 grados: por eso la parada por
// phi quedaba "desfasada" algunos grados.
float epsInicio = 0.0, epsAhora = 0.0;
bool  epsInicioListo = false;
float anguloTubo = 0.0;
// 16-nov: por defecto la parada vuelve a ser SOLO con el giroscopio (ya con la
// escala corregida). El 28-oct la correccion eps dio -378.6 contra phi = -355.1:
// 23 grados de diferencia que no son creibles. eps sale de la DERIVADA del laser
// (ruido de 0.2 a 0.6 cm segun la caracterizacion, y eps de partida tomado de una
// sola muestra), asi que mete mas error del que quita. El giroscopio corregido
// tiene error medio menor a 1 grado. epscorr=1 la vuelve a activar para comparar;
// el angulo corregido se sigue calculando y mostrando en 'estado'.
bool  USAR_EPS = false;

void aplicarActuador(float u) {
  if (ACTUADOR == 0) DAMP = u; else DDESF = u;
}

// valor de partida del actuador para un giro pedido (grados/s)
float baseActuador(float giro) {
  if (ACTUADOR == 1 && fabs(KUFF) > 1e-4) return constrain(giro / KUFF, UMIN[1], UMAX[1]);
  return U0[ACTUADOR];
}

void pasoOrbita() {
  if (!orbitaActiva) return;
  // 27-oct: 500 ms era muy poco: al caminar llegan huecos de lineas (reinicios del
  // laser) y la orbita se detenia aunque el MPU estaba bien. Mientras tanto se usa el
  // ultimo giro (el robot no cambia de giro tan rapido); se detiene si pasan 1.5 s.
  if (millis() - tUltimoGiro > 1500) {              // 25-oct: sin giroscopio no hay orbita
    orbitaActiva = false;
    marchaActiva = false;
    mensajeWifi("AVISO: orbita detenida: no llegan datos validos del giroscopio (revisar MPU / bus I2C)");
    return;
  }
  uint8_t k = ACTUADOR;
  float dt = dtCiclo;
  // giro medido, filtrado 1 s
  if (hayGiro) wFilt += dt / (1.0 + dt) * (psiDotDeg - wFilt);
  // distancia creible?
  float d = distanciaCm;
  laserOK = (d > 3.0 && d < DMAX_ORB) &&
            (distFilt < 0.0 || fabs(d - distFilt) < 8.0 || tSinLaser > 0.4);
  float sg = (WREF >= 0.0) ? 1.0 : -1.0;
  float corr = 0.0;
  if (laserOK) {
    tSinLaser = 0.0;
    if (distFilt < 0.0) { distFilt = d; dDistFilt = 0.0; }
    float a = dt / (0.3 + dt);
    float nuevo = distFilt + a * (d - distFilt);
    float der = (nuevo - distFilt) / dt;
    dDistFilt += dt / (0.8 + dt) * (der - dDistFilt);
    distFilt = nuevo;
    float e = constrain(distFilt - DREF, -10.0, 10.0);
    iE = constrain(iE + KIE * e * dt, -IELIM, IELIM);
    corr = constrain(KPE * e + KDE * dDistFilt + iE, -WLIM, WLIM);
  } else {
    tSinLaser += dt;
    corr = iE;
  }
  wObj = WREF + sg * corr;
  // correccion del angulo recorrido (solo con distancia creible)
  if (laserOK) {
    float R = 5.75 + distFilt + 6.0;                       // cm, radio al centro del robot
    float v = fabs(wFilt) * PI / 180.0 * R;                // cm/s
    if (v > 1.0) epsAhora = -sg * asin(constrain(dDistFilt / v, -0.5, 0.5)) * 180.0 / PI;   // +-30 grados
    if (!epsInicioListo && tSinLaser == 0.0 && fabs(phiDeg - phiInicioOrbita) > 5.0) {
      epsInicio = epsAhora; epsInicioListo = true;         // tras ~5 grados ya hay derivada estable
    }
  }
  anguloTubo = (phiDeg - phiInicioOrbita) - (epsInicioListo ? (epsAhora - epsInicio) : 0.0);
  // lazo interno de giro
  float ew = wObj - wFilt;
  float base = baseActuador(wObj);
  iW = constrain(iW + KIW * ew * dt, UMIN[k] - base, UMAX[k] - base);
  uOrbita = constrain(base + iW + KPW * ew, UMIN[k], UMAX[k]);
  aplicarActuador(uOrbita);
  // parada: con el giro del robot (giroscopio); con epscorr=1 y el laser viendo el
  // tubo, con el angulo corregido por el error de rumbo
  float recorrido = (USAR_EPS && laserOK) ? anguloTubo : (phiDeg - phiInicioOrbita);
  // la rampa de parada dura RAMPA_S y el robot sigue girando mientras baja la
  // amplitud (~la mitad de la rampa a velocidad plena): se anticipa la parada
  float anticipo = fabs(wFilt) * RAMPA_S * 0.5;
  if (fabs(recorrido) + anticipo >= PHI_OBJ) {
    orbitaActiva = false;
    marchaActiva = false;                               // rampa de parada de 1 s
    mensajeWifi("OK orbita terminada: giro del robot phi=" + String(phiDeg - phiInicioOrbita, 1) +
                " (con la correccion eps seria " + String(anguloTubo, 1) + "; parada por " +
                ((USAR_EPS && laserOK) ? "eps" : "phi") + ") dist=" + String(distFilt, 1) + " cm");
  }
}

String resumenOrbita() {
  uint8_t k = ACTUADOR;
  return String("actuador=") + (k == 0 ? "damp" : "ddesf") + " dref=" + String(DREF, 1) +
         " vel=" + String(VEL_AVANCE, 1) + " wref=" + String(WREF, 2) + (wrefManual ? "(a mano)" : "") +
         " kuff=" + String(KUFF, 4) + " orbsentido=" + String(SENTIDO_ORB > 0 ? "+1(antihorario)" : "-1(horario)") +
         " desforb=" + String(DESFASE_ORBITA, 0) + " u0=" + String(baseActuador(WREF), 0) +
         " umin=" + String(UMIN[k], 0) + " umax=" + String(UMAX[k], 0) +
         " kpw=" + String(KPW, 1) + " kiw=" + String(KIW, 1) +
         " kpe=" + String(KPE, 2) + " kde=" + String(KDE, 2) + " kie=" + String(KIE, 3) +
         " wlim=" + String(WLIM, 1) + " dmax=" + String(DMAX_ORB, 0) + " phiobj=" + String(PHI_OBJ, 0) +
         " epscorr=" + String(USAR_EPS ? 1 : 0);
}

String estadoOrbita() {
  return String("orbita: ") + (orbitaActiva ? "ACTIVA" : "detenida") +
         " | giro filtrado=" + String(wFilt, 1) + " objetivo=" + String(wObj, 1) + " grados/s" +
         " | dist filtrada=" + String(distFilt, 1) + " cm " + (laserOK ? "(laser OK)" : "(sin tubo)") +
         " | u=" + String(uOrbita, 0) + " iE=" + String(iE, 2) +
         " | recorrido: tubo=" + String(anguloTubo, 0) + " phi=" + String(phiDeg - phiInicioOrbita, 0) +
         " de " + String(PHI_OBJ, 0) + " (eps=" + String(epsAhora - epsInicio, 1) + ")";
}

unsigned long lineasMalSuma = 0;   // lineas descartadas por suma de verificacion
// Devuelve false si la linea trae "*XX" al final y no coincide con el XOR de lo anterior.
// Una linea sin "*" (ESP-Sensores vieja) se acepta como antes.
bool sumaOK(String &linea) {
  int k = linea.lastIndexOf('*');
  if (k < 0 || k != (int)linea.length() - 3) return true;
  uint8_t x = 0;
  for (int i = 0; i < k; i++) x ^= (uint8_t)linea.charAt(i);
  uint8_t recibida = (uint8_t)strtol(linea.substring(k + 1).c_str(), NULL, 16);
  linea = linea.substring(0, k);
  return x == recibida;
}

unsigned long reiniciosEspSensores = 0;
void procesarLineaSensores(String linea) {
  int kb = linea.indexOf("#boot reset=");
  if (kb >= 0) {                                    // la ESP-Sensores acaba de arrancar
    reiniciosEspSensores++;
    mensajeWifi("AVISO: la ESP-Sensores se reinicio (" + linea.substring(kb + 12) +
                "), van " + String(reiniciosEspSensores) + ". Si se repite: voltaje bajo o un sensor/cable en corto");
    return;
  }
  if (!sumaOK(linea)) { lineasMalSuma++; return; }
  lineasSensores++;
  ultimaLineaSensores = linea.substring(0, 90);
  if (linea.indexOf("theta") == -1) { lineasSinTheta++; return; }   // basura o formato distinto
  theta[FI] = parsearCampo(linea, "theta1=");
  theta[FD] = parsearCampo(linea, "theta2=");
  theta[TI] = parsearCampo(linea, "theta3=");
  theta[TD] = parsearCampo(linea, "theta4=");
  leerNavegacion(linea);
  for (uint8_t p = 0; p < NUM_PATAS; p++)         // angulo lineal si hay tabla
    if (theta[p] > VALOR_NO_DISPONIBLE + 1.0)     // el filtro trabaja en grados del sensor; al final, grados reales
      theta[p] = realDesdeSensor(p, filtrarLectura(p, linealizar(p, theta[p])));
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
      if (bufferSerial.length() > 260) bufferSerial = "";   // linea corrupta
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
  float a = realDesdeSensor(p, THETA_W0[p] + GRADOS_POR_WRITE[p] * WRITE_MIN);
  float b = realDesdeSensor(p, THETA_W0[p] + GRADOS_POR_WRITE[p] * WRITE_MAX);
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
  if (clave == "ganancias") {
    if (v != "aire" && v != "marcha") { mensajeWifi("usar ganancias=aire o ganancias=marcha"); return; }
    cargarGanancias(v == "marcha");
    calcularCoeficientes();
    reiniciarControlador();
    String g = "";
    for (uint8_t q = 0; q < NUM_PATAS; q++)
      g += String(q ? " | " : "") + String(q + 1) + ": kp=" + String(KP[q], 2) + " ki=" +
           String(KI[q], 1) + " kd=" + String(KD[q], 3) + " banda=" + String(BANDA_I[q], 1) +
           " imax=" + String(IMAX[q], 0);
    mensajeWifi("OK ganancias de " + v + " -> " + g);
    return;
  }
  if (clave == "modo") {
    estadoAuto = 2;
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
  // tabla de angulo real: real1=v220,v230,...,v310 (10 valores, crecientes)
  if (clave.length() == 5 && clave.startsWith("real")) {
    int p = clave.charAt(4) - '1';
    if (p < 0 || p >= NUM_PATAS) { mensajeWifi("formato: real1=10 valores (angulo real con el sensor en 220, 230, ..., 310)"); return; }
    float vals[N_REAL];
    int n = 0, desde = 0;
    while (n < N_REAL) {
      int coma = v.indexOf(',', desde);
      String item = (coma == -1) ? v.substring(desde) : v.substring(desde, coma);
      item.trim();
      if (item.length() == 0) break;
      vals[n++] = item.toFloat();
      if (coma == -1) break;
      desde = coma + 1;
    }
    if (n != N_REAL) { mensajeWifi("tabla real con " + String(n) + " valores, se esperaban 10"); return; }
    for (int i = 1; i < N_REAL; i++)
      if (vals[i] <= vals[i - 1]) { mensajeWifi("tabla real no creciente en el valor " + String(i + 1) + ": no se carga"); return; }
    for (int i = 0; i < N_REAL; i++) realTab[p][i] = vals[i];
    reiniciarControlador();
    mensajeWifi("OK tabla de angulo real pata " + String(p + 1) + ": " + String(vals[0], 1) + " .. " +
                String(vals[N_REAL - 1], 1) + " (con el sensor en 270 -> " + String(vals[5], 1) + " reales)");
    return;
  }
  if (clave == "andar") {                   // tipo de paso: fases de las 4 patas
    v.toLowerCase();
    float f[NUM_PATAS];
    if      (v == "trote")     { f[0] = 0; f[1] = 180; f[2] = 180; f[3] = 0;   }
    else if (v == "ambladura") { f[0] = 0; f[1] = 180; f[2] = 0;   f[3] = 180; }
    else if (v == "salto")     { f[0] = 0; f[1] = 0;   f[2] = 180; f[3] = 180; }
    else if (v == "brinco")    { f[0] = 0; f[1] = 0;   f[2] = 0;   f[3] = 0;   }
    else { mensajeWifi("andar=trote | ambladura | salto | brinco"); return; }
    for (uint8_t p = 0; p < NUM_PATAS; p++) FASE_PATA[p] = f[p] * PI / 180.0;
    mensajeWifi("OK andar=" + v + ": fases FI,FD,TI,TD = " + String(f[0], 0) + "," + String(f[1], 0) + "," +
                String(f[2], 0) + "," + String(f[3], 0) + " grados (desfase=" + String(DESFASE, 0) +
                " y ddesf=" + String(DDESF, 0) + " se suman a las traseras)");
    return;
  }

  if (!esNumero(v)) { mensajeWifi("valor no numerico en: " + linea); return; }
  float val = v.toFloat();

  if (clave == "mapa") {
    mapaReal = (val != 0);
    reiniciarControlador();
    mensajeWifi(String("OK mapa=") + (mapaReal ? "1: ref y theta en grados REALES (tabla del 16-nov)"
                                               : "0: ref y theta en grados nominales del servo (como antes)"));
    return;
  }
  if (clave == "vel") {                     // velocidad de avance -> wref al encender la orbita
    VEL_AVANCE = constrain(val, 0.5, 12.0);
    wrefManual = false;
    float R = 5.75 + DREF + 6.0;            // cm, del centro del tubo al centro del robot
    float w = VEL_AVANCE / R * 180.0 / PI;
    mensajeWifi("OK vel=" + String(VEL_AVANCE, 1) + " cm/s: con dref=" + String(DREF, 1) + " (radio " + String(R, 1) +
                " cm) la orbita usara wref=" + String(SENTIDO_ORB * w, 2) + " grados/s; una vuelta en " + String(360.0 / w, 0) + " s");
    return;
  }
  if (clave == "control") {                 // juego de ganancias del lazo de distancia
    if (val != 0) { KPE = 0.30; KDE = 5.0; KIE = 0.03; WLIM = 5.0; }
    else          { KPE = 0.14; KDE = 2.0; KIE = 0.02; WLIM = 3.0; }
    mensajeWifi(String("OK control=") + (val != 0 ? "1 (ajustado con el modelo)" : "0 (el del 28-oct)") + ": " + resumenOrbita());
    return;
  }

  if (clave == "marcha") {
    estadoAuto = 2;                                // el usuario toma el control
    marchaActiva = (val != 0);
    orbitaActiva = false;
    if (marchaActiva && escalaMarcha <= 0.0) faseMarcha = 0.0;
    mensajeWifi(String("OK marcha ") + (marchaActiva ? "ENCENDIDA" : "deteniendo (rampa de 1 s)") +
                ": amp=" + String(AMP, 1) + " damp=" + String(DAMP, 1) + " frec=" + String(FREC, 2) +
                " centro=" + String(CENTRO, 1) + " sentido=" + String(SENTIDO) + " desfase=" + String(DESFASE, 0) + " ddesf=" + String(DDESF, 0) +
                " | modo=" + String(modoPID ? "pid" : "abierto"));
    return;
  }
  else if (clave == "amp")     { AMP = constrain(val, 0.0, 40.0); }
  else if (clave == "damp")    { DAMP = constrain(val, -30.0, 30.0); }
  else if (clave == "frec")    {
    FREC = constrain(val, 0.1, 5.0);
    // velocidad pico de la rodilla = 2*pi*f*A; el MG90S da ~600 °/s sin carga
    float vpico = 2.0 * PI * FREC * (AMP + fabs(DAMP) / 2.0);
    if (vpico > 450.0)
      mensajeWifi("AVISO: con amp=" + String(AMP, 0) + " a " + String(FREC, 1) + " Hz la rodilla tendria que girar " +
                  String(vpico, 0) + " °/s; el servo no alcanza y el paso sale mas corto. Bajar amp.");
  }
  else if (clave == "centro")  { CENTRO = val; }
  else if (clave == "sentido") { SENTIDO = (val < 0) ? -1 : 1; }
  else if (clave == "phi")     { phiDeg = val; }
  else if (clave == "sesgo")   {                     // volver a medir el sesgo (robot QUIETO 2 s)
    sumaSesgo = 0.0; nSesgo = 0; tInicioSesgo = millis();
    mensajeWifi("OK midiendo el sesgo del giroscopio 2 s: no mover el robot");
    return;
  }
  else if (clave == "desfase") { DESFASE = constrain(val, -180.0, 180.0); }
  else if (clave == "ddesf")   { DDESF = constrain(val, -360.0, 360.0); }
  else if (clave == "orbita") {
    estadoAuto = 2;
    orbitaActiva = (val != 0);
    if (orbitaActiva) {
      phiInicioOrbita = phiDeg;
      distFilt = -1.0; dDistFilt = 0.0;
      iW = 0.0; iE = 0.0; tSinLaser = 0.0;
      epsInicio = 0.0; epsAhora = 0.0; epsInicioListo = false; anguloTubo = 0.0;
      wFilt = hayGiro ? psiDotDeg : 0.0;
      if (ACTUADOR == 1) { DESFASE = DESFASE_ORBITA; DAMP = 0.0; }
      if (!wrefManual)                               // giro que corresponde a vel y dref (horario)
        WREF = SENTIDO_ORB * constrain(VEL_AVANCE / (5.75 + DREF + 6.0) * 180.0 / PI, 1.0, 30.0);
      aplicarActuador(baseActuador(WREF));
      if (escalaMarcha <= 0.0) faseMarcha = 0.0;
      marchaActiva = true;
    } else {
      marchaActiva = false;
    }
    mensajeWifi(String("OK orbita ") + (orbitaActiva ? "ENCENDIDA" : "detenida") +
                ": " + resumenOrbita() + " desfase=" + String(DESFASE, 0) +
                " amp=" + String(AMP, 1) + " frec=" + String(FREC, 2) +
                " dist_ahora=" + String(distanciaCm, 1) + " | modo=" + String(modoPID ? "pid" : "abierto"));
    return;
  }
  else if (clave == "dref")    { DREF = constrain(val, 5.0, 40.0); }
  else if (clave == "actuador") { ACTUADOR = (val != 0) ? 1 : 0; mensajeWifi("OK " + resumenOrbita()); return; }
  else if (clave == "u0")      { U0[ACTUADOR] = val; }
  else if (clave == "umin")    { UMIN[ACTUADOR] = val; }
  else if (clave == "umax")    { UMAX[ACTUADOR] = val; }
  else if (clave == "wref")    { WREF = constrain(val, -30.0, 30.0); wrefManual = true; }
  // kuff con signo: negativo si un ddesf negativo hace girar en sentido positivo
  // (antihorario). kuff=0 apaga la prealimentacion (u0 fijo).
  else if (clave == "kuff")    { KUFF = constrain(val, -0.2, 0.2); }
  else if (clave == "orbsentido") { SENTIDO_ORB = (val >= 0) ? 1.0 : -1.0; wrefManual = false; }
  else if (clave == "desforb") { DESFASE_ORBITA = constrain(val, -180.0, 180.0); }
  else if (clave == "kpw")     { KPW = constrain(val, 0.0, 50.0); }
  else if (clave == "kiw")     { KIW = constrain(val, 0.0, 100.0); }
  else if (clave == "kpe")     { KPE = constrain(val, 0.0, 5.0); }
  else if (clave == "kde")     { KDE = constrain(val, 0.0, 20.0); }
  else if (clave == "kie")     { KIE = constrain(val, 0.0, 1.0); }
  else if (clave == "wlim")    { WLIM = constrain(val, 0.0, 15.0); }
  else if (clave == "dmax")    { DMAX_ORB = constrain(val, 10.0, 60.0); }
  else if (clave == "phiobj")  { PHI_OBJ = constrain(val, 0.0, 720.0); }
  else if (clave == "escgiro") { ESCALA_GIRO = constrain(val, 0.8, 1.2); }
  else if (clave == "lasera")  { LASER_A = constrain(val, 0.5, 1.5); }
  else if (clave == "laserb")  { LASER_B = constrain(val, -5.0, 5.0); }
  else if (clave == "epscorr") { USAR_EPS = (val != 0); }
  else if (clave.length() == 5 && clave.startsWith("fase") && clave.charAt(4) >= '1' && clave.charAt(4) <= '4') {
    int p = clave.charAt(4) - '1';
    FASE_PATA[p] = constrain(val, -360.0, 360.0) * PI / 180.0;
    mensajeWifi("OK fases FI,FD,TI,TD = " + String(FASE_PATA[0] * 180.0 / PI, 0) + "," + String(FASE_PATA[1] * 180.0 / PI, 0) +
                "," + String(FASE_PATA[2] * 180.0 / PI, 0) + "," + String(FASE_PATA[3] * 180.0 / PI, 0) + " grados");
    return;
  }
  else if (clave.length() == 4 && (clave.startsWith("cen") || clave.startsWith("esc")) &&
           clave.charAt(3) >= '1' && clave.charAt(3) <= '4') {
    int p = clave.charAt(3) - '1';
    if (clave.startsWith("cen")) CEN_PATA[p] = constrain(val, -20.0, 20.0);
    else                         ESC_PATA[p] = constrain(val, 0.0, 2.0);
    mensajeWifi("OK marcha por pata: cen=" + String(CEN_PATA[0], 1) + "," + String(CEN_PATA[1], 1) + "," +
                String(CEN_PATA[2], 1) + "," + String(CEN_PATA[3], 1) + "  esc=" + String(ESC_PATA[0], 2) +
                "," + String(ESC_PATA[1], 2) + "," + String(ESC_PATA[2], 2) + "," + String(ESC_PATA[3], 2));
    return;
  }
  else if (clave == "ref1") { referencia[FI] = val; avisarSiFueraDeRango(FI, val); }
  else if (clave == "ref2") { referencia[FD] = val; avisarSiFueraDeRango(FD, val); }
  else if (clave == "ref3") { referencia[TI] = val; avisarSiFueraDeRango(TI, val); }
  else if (clave == "ref4") { referencia[TD] = val; avisarSiFueraDeRango(TD, val); }
  else if (clave == "ref")  {
    for (uint8_t p = 0; p < NUM_PATAS; p++) { referencia[p] = val; avisarSiFueraDeRango(p, val); }
  }
  else if (clave.startsWith("kp") || clave.startsWith("ki") || clave.startsWith("kd") ||
           clave.startsWith("banda") || clave.startsWith("imax")) {
    // kp=, ki=, kd=, banda= -> las 4 patas;  kp1..kp4=, ki1.., kd1.., banda1.. -> una pata
    String base = clave;
    int p = -1;
    char ult = clave.charAt(clave.length() - 1);
    if (ult >= '1' && ult <= '4') { p = ult - '1'; base = clave.substring(0, clave.length() - 1); }
    float* arr = (base == "kp") ? KP : (base == "ki") ? KI : (base == "kd") ? KD :
                 (base == "banda") ? BANDA_I : (base == "imax") ? IMAX : nullptr;
    if (arr == nullptr) { mensajeWifi("comando no reconocido: " + linea); return; }
    for (uint8_t q = 0; q < NUM_PATAS; q++) if (p < 0 || q == p) arr[q] = val;
    calcularCoeficientes();
    reiniciarControlador();
    String g = "";
    for (uint8_t q = 0; q < NUM_PATAS; q++)
      g += String(q ? " | " : "") + String(q + 1) + ": kp=" + String(KP[q], 2) + " ki=" +
           String(KI[q], 1) + " kd=" + String(KD[q], 3) + " banda=" + String(BANDA_I[q], 1) + " imax=" + String(IMAX[q], 0);
    mensajeWifi("OK " + clave + "=" + String(val, 3) + "  -> " + g);
    return;
  }
  else { mensajeWifi("comando no reconocido: " + linea); return; }
  // para la marcha se informa el valor que quedo de verdad (puede venir limitado)
  if      (clave == "amp")  val = AMP;
  else if (clave == "damp") val = DAMP;
  else if (clave == "frec") val = FREC;
  else if (clave == "sentido") val = SENTIDO;
  mensajeWifi("OK " + clave + "=" + String(val, 3) +
              ((clave == "frec" || clave == "amp" || clave == "damp") ? String(" (limites: amp 0..40, damp -30..30, frec 0.1..5)") : String("")));
}

// ---------------------------------------------------------------
// Diagnostico cada 2 s: por WiFi (siempre) y por el monitor serie (si DEBUG_SERIAL)
// El monitor serie debe estar a 115200 baudios.
// ---------------------------------------------------------------
#define DEBUG_SERIAL true
unsigned long ultimoDiagnostico = 0, bytesAnt = 0, lineasAnt = 0;

String bufTele;
unsigned long tUltimoEnvioTele = 0;
unsigned long teleDescartada = 0;
unsigned long teleMaxUs = 0;      // peor tiempo de una escritura WiFi (diagnostico)

// Tiempo que se lleva cada parte del loop (14-oct: el lazo iba a 17-25 ciclos/s
// aun con la telemetria sin bloquear). 0 wifi, 1 serial, 2 diagnostico, 3 control,
// 4 telemetria, 5 fuera del loop (WiFi del sistema entre una vuelta y otra).
const char *NOMBRE_SECCION[7] = {"wifi", "serial", "diag", "control", "tele", "sistema", "servos"};
unsigned long tSeccionUs[7] = {0}, tSeccionMaxUs[7] = {0};
unsigned long vueltasLoop = 0;
float vccMin = 99.0, vccMax = 0.0;   // V, en la ventana de 2 s del diagnostico
void sumarSeccion(uint8_t i, unsigned long d) { tSeccionUs[i] += d; if (d > tSeccionMaxUs[i]) tSeccionMaxUs[i] = d; }

void diagnostico() {
  unsigned long ahora = millis();
  if (ahora - ultimoDiagnostico < 2000) return;
  float dt = (ahora - ultimoDiagnostico) / 1000.0;
  ultimoDiagnostico = ahora;
  float bps = (bytesSensores - bytesAnt) / dt;
  float lps = (lineasSensores - lineasAnt) / dt;
  bytesAnt = bytesSensores; lineasAnt = lineasSensores;
  static unsigned long ciclosAnt = 0;
  float cps = (ciclosControl - ciclosAnt) / dt;
  ciclosAnt = ciclosControl;

  String estado;
  if (bps < 1)                         estado = "NO LLEGA NADA de la ESP-Sensores (revisar cable TX->RX, GND comun y que este encendida)";
  else if (lps < 1)                    estado = "llegan bytes pero no lineas completas: BAUDIOS DISTINTOS (esta placa usa " + String(BAUD_SENSORES) + ")";
  else if (millis() - ultimoDatoRecibido > TIMEOUT_DATOS_MS) estado = "llegan lineas pero sin theta: revisar formato (ver ultima linea)";
  else if (lps < 45)                   estado = "OK pero lento: " + String(lps, 0) + " lineas/s (el PID necesita 50)";
  else                                 estado = "OK";

  String s = "estado sensores: " + estado + " | " + String(bps, 0) + " B/s, " + String(lps, 0) +
             " lineas/s | control " + String(cps, 0) + " ciclos/s (deben ser 50)" +
             " | WiFi max " + String(teleMaxUs / 1000.0, 1) + " ms, paquetes descartados " + String(teleDescartada) +
             " | modo=" + String(modoPID ? "pid" : "abierto") + " mapa=" + String(mapaReal ? 1 : 0) +
             " | theta=" + String(theta[FI], 1) + "," + String(theta[FD], 1) + "," +
             String(theta[TI], 1) + "," + String(theta[TD], 1) +
             " | write=" + String(writeActual[FI]) + "," + String(writeActual[FD]) + "," +
             String(writeActual[TI]) + "," + String(writeActual[TD]);
  mensajeWifi(s);
  teleMaxUs = 0;                                   // peor tiempo de los ultimos 2 s
  String tt = "tiempos en 2 s (total ms / peor ms):";
  for (uint8_t i = 0; i < 7; i++) {
    tt += " "; tt += NOMBRE_SECCION[i]; tt += " "; tt += String(tSeccionUs[i] / 1000.0, 0);
    tt += "/"; tt += String(tSeccionMaxUs[i] / 1000.0, 1);
    tSeccionUs[i] = 0; tSeccionMaxUs[i] = 0;
  }
  tt += " | vueltas del loop " + String(vueltasLoop / dt, 0) + "/s";
  tt += " | VCC ESP min " + String(vccMin, 2) + " V, max " + String(vccMax, 2) + " V (normal ~3.3)";
  vccMin = 99.0; vccMax = 0.0;
  vueltasLoop = 0;
  mensajeWifi(tt);
  mensajeWifi("lineas danadas descartadas (suma de verificacion): " + String(lineasMalSuma));
  mensajeWifi(estadoOrbita());
  String sinSensor = "";
  for (uint8_t p = 0; p < NUM_PATAS; p++) if (sensorFalla[p]) sinSensor += String(" ") + NOMBRE_PATA[p];
  mensajeWifi("patas en lazo abierto por sensor malo:" + (sinSensor.length() ? sinSensor : String(" ninguna")));
  mensajeWifi("lecturas descartadas por el filtro (FI,FD,TI,TD): " + String(lecturasDescartadas[FI]) + "," +
              String(lecturasDescartadas[FD]) + "," + String(lecturasDescartadas[TI]) + "," +
              String(lecturasDescartadas[TD]));
  mensajeWifi("giroscopio: fuente=" + String(usaYawDot ? "yaw_dot (vertical)" : "psi_dot (eje z)") +
              " | inclinacion MPU=" + String(incMpu, 1) + " grados | sesgo=" + String(sesgoGiro, 2) +
              " grados/s | escala=" + String(ESCALA_GIRO, 4) +
              " | giro=" + String(psiDotDeg, 1) + " grados/s | phi=" + String(phiDeg, 1) +
              " | laser: dist=" + String(distanciaCm, 1) + " cm, reinicios=" + String(reiniciosLaser) +
              " | MPU reiniciado " + String(reiniciosMpuSens) + " veces, bus I2C recuperado " + String(recupBusSens) +
              " | ESP-Sensores reiniciada " + String(reiniciosEspSensores) + " veces" +
              ((millis() - tUltimoGiro > 500) ? String(" | SIN DATOS DEL GIROSCOPIO") : String("")));
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
  float integralNueva = integral[p];
  if (fabs(r - y) >= BANDA_I[p])                 // zona muerta: cerca de la referencia no integra
    integralNueva += c_i[p] * (e + e_prev[p]);
  integralNueva = constrain(integralNueva, -IMAX[p], IMAX[p]);
  derivada[p] = c1 * derivada[p] - c2[p] * (y - y_prev[p]);

  // 3) feedforward (la referencia) + correccion del PID
  float thetaCmd = r + KP[p] * e + integralNueva + derivada[p];
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
// cliente.print() en la ESP8266 espera el ACK de cada paquete TCP y frenaba el lazo
// a ~17 ciclos/s (9-oct). Ahora las lineas se juntan y se mandan cada 100 ms en un
// solo paquete, y solo si cabe en el buffer de envio; si no cabe, se descarta.
void enviarTelemetria(unsigned long t, const bool *valida, const float *error) {
  if (!cliente || !cliente.connected()) { bufTele = ""; return; }
  String s;
  s.reserve(160);
  s += t;
  for (uint8_t p = 0; p < NUM_PATAS; p++) {
    s += ','; s += String(referencia[p], 2);
    s += ','; s += valida[p] ? String(theta[p], 2) : String("NA");
    s += ','; s += valida[p] ? String(error[p], 2) : String("NA");
    s += ','; s += writeActual[p];
  }
  // navegacion: velocidad de giro (°/s), angulo girado (°), distancia (cm)
  s += ','; s += hayGiro ? String(psiDotDeg, 2) : String("NA");
  s += ','; s += hayGiro ? String(phiDeg, 2) : String("NA");
  s += ','; s += (distanciaCm >= 0) ? String(distanciaCm, 1) : String("NA");
  s += ','; s += String(DAMP, 2);                    // campo 21 (9-oct): damp aplicado
  s += ','; s += String(DDESF, 1);                   // campo 22 (21-oct): ddesf aplicado
  s += "\r\n";
  bufTele += s;
  if (millis() - tUltimoEnvioTele < 100 && bufTele.length() < 900) return;
  tUltimoEnvioTele = millis();
  if (cliente.availableForWrite() >= bufTele.length()) {
    unsigned long t0 = micros();
    cliente.write((const uint8_t *)bufTele.c_str(), bufTele.length());
    unsigned long dt = micros() - t0;
    if (dt > teleMaxUs) teleMaxUs = dt;
  } else {
    teleDescartada++;
  }
  bufTele = "";
}

void setup() {
  Serial.setRxBufferSize(1024);   // ~7 lineas: si el loop se atrasa no se pierden datos
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
#if defined(ARDUINO_ESP8266_MAJOR) && (ARDUINO_ESP8266_MAJOR >= 3)
  enablePhaseLockedWaveform();   // Servo.write sin esperar al siguiente periodo
#endif

  cargarGanancias(true);           // juego de ganancias de la marcha
  calcularCoeficientes();

  WiFi.mode(WIFI_AP);
  // 22-oct: potencia de transmision 14 dBm (de 20.5): la PC esta cerca y cada envio
  // pide menos corriente a la bateria que comparten los servos.
  WiFi.setOutputPower(14.0);
  WiFi.softAP(SSID_AP, PASSWORD_AP);
  servidor.begin();

  delay(300);
  Serial.println();
  Serial.println("[dbg] ESP principal v2 lista. WiFi RobotPID en 192.168.4.1:80");
  ultimoCicloControl = millis();
  tInicioAuto = millis();
  if (!ARRANQUE_AUTOMATICO) estadoAuto = 2;
  ultimoDatoRecibido = 0;          // sin datos hasta que llegue la primera linea
}

// 14-oct: la parte de control tardaba ~35 ms por ciclo con el robot quieto. Era
// Servo.write(): en el nucleo 3.x de la ESP8266 el generador "PWM" (el de defecto)
// aplica cada cambio en el siguiente periodo de 20 ms del servo, y un segundo write
// espera a que se aplique el anterior (core_esp8266_waveform_pwm.cpp, waveformToChange).
// Por eso: (1) en setup se activa el modo "phase locked", que aplica el cambio de
// inmediato, y (2) solo se escribe cuando el valor cambia. El tiempo de las escrituras
// sale en 'estado' (seccion servos).
int ultimoWriteServo[NUM_PATAS] = {-1, -1, -1, -1};
void escribirServo(uint8_t p, int w) {
  if (w == ultimoWriteServo[p]) return;
  unsigned long t0 = micros();
  servoTibia[p].write(w);
  ultimoWriteServo[p] = w;
  sumarSeccion(6, micros() - t0);
}

void loop() {
  static unsigned long tFinLoop = 0;
  unsigned long tA = micros();
  if (tFinLoop) sumarSeccion(5, tA - tFinLoop);
  vueltasLoop++;
  loopInterno();
  tFinLoop = micros();
}

void loopInterno() {
  unsigned long tB = micros();
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
    mensajeWifi("Ultimo reinicio de la ESP principal: " + ESP.getResetReason() +
                " (Brownout/Power On a mitad de prueba = se cayo el voltaje)");
    mensajeWifi("ESP principal v2 conectada. Comandos: ref=, ref1..4=, kp=, ki=, kd= (o kp1..4= por pata), banda1..4=, modo=pid|abierto, marcha=1|0, amp=, damp=, frec=, sentido=, centro=, cal1..4=, tab1..4=, mapa=1|0, real1..4=, escgiro=, lasera=, laserb=, vel=, kuff=, orbsentido=, control=1|0, epscorr=, andar=trote|ambladura|salto|brinco, fase1..4=");
    mensajeWifi(String("Sensores caracterizados el 16-nov: mapa=") + (mapaReal ? "1" : "0") + " (ref y theta en grados " +
                (mapaReal ? "REALES" : "nominales") + "), escgiro=" + String(ESCALA_GIRO, 4) +
                ", laser a=" + String(LASER_A, 4) + " b=" + String(LASER_B, 3));
  }
  revisarComandoWifi();
  unsigned long tC = micros(); sumarSeccion(0, tC - tB);

  // 2) datos de la ESP-Sensores (sin bloquear) y diagnostico
  revisarSerialSensores();
  unsigned long tD = micros(); sumarSeccion(1, tD - tC);
  diagnostico();
  unsigned long tE = micros(); sumarSeccion(2, tE - tD);
  bool enlaceActivo = (millis() - ultimoDatoRecibido) < TIMEOUT_DATOS_MS;

  // 3) ciclo de control a periodo fijo
  unsigned long ahora = millis();
  if (ahora - ultimoCicloControl < PERIODO_CONTROL_MS) return;
  static unsigned long tCicloAnterior = 0;
  if (tCicloAnterior) dtCiclo = constrain((ahora - tCicloAnterior) / 1000.0, 0.005, 0.1);
  tCicloAnterior = ahora;
  ciclosControl++;
  ultimoCicloControl += PERIODO_CONTROL_MS;          // sin deriva acumulada
  if (ahora - ultimoCicloControl > PERIODO_CONTROL_MS) ultimoCicloControl = ahora;

  // arranque automatico: espera, camina DURACION_MARCHA_S y se detiene
  if (estadoAuto == 0 && millis() - tInicioAuto > ESPERA_ARRANQUE_MS) {
    bool hayDatos = (millis() - ultimoDatoRecibido) < TIMEOUT_DATOS_MS;
    modoPID = hayDatos;                              // sin sensores: lazo abierto
    reiniciarControlador();
    marchaActiva = true;
    faseMarcha = 0.0;
    estadoAuto = 1;
    tInicioAuto = millis();
    mensajeWifi(String("Arranque automatico: camina ") + String(DURACION_MARCHA_S, 0) + " s en modo " +
                (modoPID ? "pid (ganancias de marcha)" : "abierto (no llegan datos de los sensores)"));
  } else if (estadoAuto == 1 && modoPID && (millis() - ultimoDatoRecibido) > 500) {
    modoPID = false;                                 // se cayeron los sensores: sigue en lazo abierto
    mensajeWifi("Arranque automatico: sin datos de los sensores, sigo en lazo abierto");
  }
  if (estadoAuto == 1 && millis() - tInicioAuto > (unsigned long)(DURACION_MARCHA_S * 1000)) {
    marchaActiva = false;                            // rampa de parada de 1 s
    estadoAuto = 2;
    mensajeWifi("Arranque automatico: fin de la marcha");
  }
  pasoOrbita();                                      // ajusta DAMP con el laser (si orbita=1)
  pasoMarcha();                                      // referencias del trote (si esta activo)

  bool valida[NUM_PATAS];
  float error[NUM_PATAS];

  for (uint8_t p = 0; p < NUM_PATAS; p++) {
    valida[p] = enlaceActivo && (theta[p] > VALOR_NO_DISPONIBLE + 1.0);
    error[p] = valida[p] ? referencia[p] - theta[p] : 0;

    if (!modoPID || sensorFalla[p]) {
      // lazo abierto: solo la calibracion (sirve para verificarla); tambien una pata
      // cuyo sensor esta fallando, para que el PID no la lleve al tope
      writeActual[p] = constrain(writeDesdeTheta(p, referencia[p]), WRITE_MIN, WRITE_MAX);
      escribirServo(p, writeActual[p]);
      inicializado[p] = false;
    } else if (valida[p]) {
      writeActual[p] = pasoControl(p, referencia[p], theta[p]);
      escribirServo(p, writeActual[p]);
    } else {
      // sin medicion: mantener el ultimo write y reiniciar al volver el dato
      inicializado[p] = false;
    }
  }

  // 4) telemetria a Python
  // 20-oct: con la marcha detenida no se integran giros menores a 1.5 °/s (deriva del
  // sesgo en reposo); los giros a mano y la marcha (que no se detiene) si cuentan.
  bool quieto = !marchaActiva && escalaMarcha <= 0.0 && fabs(psiDotDeg) < 1.5;
  if (hayGiro && !midiendoSesgo() && !quieto) phiDeg += psiDotDeg * dtCiclo;   // integrar el giro
  // telemetria cada 2 ciclos (25 por segundo): menos carga de WiFi, el lazo no se atrasa
  static uint8_t cuentaVcc = 0;
  if (++cuentaVcc >= 5) {                          // 10 veces por segundo
    cuentaVcc = 0;
    float v = ESP.getVcc() / 1000.0;
    if (v < vccMin) vccMin = v;
    if (v > vccMax) vccMax = v;
  }
  unsigned long tF = micros(); sumarSeccion(3, tF - tE);
  static uint8_t cuentaTele = 0;
  if (++cuentaTele >= 2) { cuentaTele = 0; enviarTelemetria(ahora, valida, error); }
  sumarSeccion(4, micros() - tF);
}
