# Robot cuadrúpedo con control inteligente

Proyecto del curso **Control Inteligente** (Prof. Jorge Andrés Puerto Acosta).
Equipo: Fabian Segura, Brandon Bernal, Juan Beltran.

Robot araña ACEBOTT/QD020 con ESP8266 y 8 servos MG90S. Cada pata tiene una
articulación activa (rodilla/tibia); la coxa queda fija a 90°. El objetivo
final es que el robot rodee un cilindro hasta un ángulo dado (0–360°),
manteniendo distancia y velocidad.

## Estructura

| Carpeta | Contenido |
|---|---|
| `firmware/esp_principal_pid_4patas_v2/` | ESP principal: PID de las 4 rodillas a 50 Hz, servos, WiFi (AP `RobotPID`, 192.168.4.1:80) |
| `firmware/esp_sensores_pata_v2/` | ESP-Sensores: 4× AS5600 (mux TCA9548A), MPU6050, VL53L0X; envía una línea cada 20 ms |
| `firmware/prueba_iman_as5600/` | Prueba del imán de un AS5600 (campo, AGC, magnitud) |
| `pc/` | Programa de la PC (`pc_pid_4patas.py`) y la calibración guardada (`calibracion_patas.json`) |
| `modelado/` | Diseño y simulación del PID de una pata |
| `docs/` | Cronograma |
| `resultados/` | Gráficas y CSV de pruebas que vale la pena conservar |

Cada `.ino` está en una carpeta con su mismo nombre, como lo exige el IDE de Arduino.

## Conexiones

- ESP-Sensores TX → RX de la ESP principal, GND común, 115200 baudios.
- Servos de rodilla (FI, FD, TI, TD) = D8, D4, D5, D0.
- Canales del mux para los AS5600 (FI, FD, TI, TD) = 0, 7, 1, 2.

## Uso

```
pip install -r pc/requirements.txt
cd pc
python pc_pid_4patas.py
```

La PC debe estar conectada a la red WiFi `RobotPID` (clave `12345678`).
Comandos principales: `ref=`, `ref1..4=`, `modo=pid|abierto`, `kp=`/`ki=`/`kd=`/`banda=`
(o `kp1..4=` etc. por pata), `escalon`, `calibrar`, `cero`, `linealizar`, `barrido N`.

## Estado (1-oct-2026)

- PID de rodillas **al aire**: funciona con ganancias por pata.

  | Pata | kp | ki | kd | banda |
  |---|---|---|---|---|
  | FI | 0.5 | 6 | 0 | 1 |
  | FD | 0.8 | 12 | 0.008 | 0.5 |
  | TI | 0.4 | 5 | 0 | 1 |
  | TD | 0.8 | 12 | 0.008 | 0.5 |

- **En el piso** el lazo se vuelve inestable (lecturas imposibles del AS5600 con
  carga y patas acopladas a través del cuerpo). Pendiente: pruebas en piso con
  `modo=abierto` y con PID suave sin integral (`kp=0.2 ki=0 kd=0 banda=0`).
- El montaje de los AS5600 (imán pegado al cuerpo del servo, fuera del eje) no
  se puede cambiar: la lectura se corrige por software con tablas (`linealizar`).

## Cómo guardar cambios

```
git add -A
git commit -m "que cambio y por que"
git push
```

Los CSV y PNG que genera el programa en cada corrida no se suben solos; los
que sirvan como evidencia se copian a `resultados/` con un nombre descriptivo.
