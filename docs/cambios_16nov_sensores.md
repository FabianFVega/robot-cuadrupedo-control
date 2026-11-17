# Cambios del 16-nov: sensores caracterizados y peso de 395.2 g

Fuente: `docs/Caracterizacion_de_sensores.xlsx`. Solo cambia la **ESP principal**
(`firmware/esp_principal_pid_4patas_v2`). La ESP de sensores y el programa de la PC
quedan como estaban (versión del 28-oct).

## 1. Qué decía la caracterización y qué se cambió

| Sensor | Lo que se midió | Cambio en el firmware | Error después |
|---|---|---|---|
| Giroscopio | lee 89.3 / 177.0 / 265.1 / 354.3 en 90 / 180 / 270 / 360° (lectura = 0.9838·real) | `ESCALA_GIRO = 1.0165` (`escgiro=`) | menos de 1° de media |
| Láser | lee 24.2 / 19.9 / 15.5 / 10.6 / 5.9 en 25 / 20 / 15 / 10 / 5 cm | `LASER_A = 0.9721`, `LASER_B = −1.047` (`lasera=`, `laserb=`) | ±0.3 cm |
| Rodillas | con `ref=270` quedan en FI 268.0, FD 273.5, TI 271.0, TD 275.0; `amp=20` recorre ±18.5, ±17.0, ±18.8, ±17.5 | tabla de ángulo real por pata (`mapa=1`, `real1..4=`) | la repetibilidad del transportador (0 a 1.6°) |

Con `mapa=1` (por defecto) `ref=` y `theta` son grados reales del transportador, en
`modo=pid` y en `modo=abierto`: las 4 patas quedan verticales en 270 y recorren lo
mismo. `mapa=0` vuelve a los grados nominales del servo.

## 2. Control de la órbita

- **Parada** solo con el giroscopio corregido. La corrección `eps` queda apagada
  (`epscorr=1` la enciende): en la simulación empeora el ángulo.
- **`vel=`** (cm/s, por defecto 4): al encender la órbita, `wref = vel / radio` para el
  `dref` pedido. Antes era −8.5 fijo, que solo sirve para 15 cm.
- **`kuff=`** (por defecto 0.0478 °/s por grado de ddesf): el ddesf de partida es
  `wref / kuff`. `kuff=0` vuelve al `u0` fijo.
- **`control=1`**: lazo de distancia ajustado con el modelo (kpe 0.3, kde 5, kie 0.03,
  wlim 5). `control=0` es el del 28-oct y es el que arranca.

## 3. Modelo (`modelado/`)

- `caracterizacion_sensores.py`: lee el Excel y saca las constantes de arriba.
- `orbita_sensores_sim.py`: órbita con el giroscopio y el láser caracterizados.
  Resultados en `resultados/orbita_sensores_sim.txt` y `.png`.

Lo que muestra la simulación (60 corridas por caso, con V ±10 %, KU ±15 % y la salida
con ±1.5 cm y ±5° de error):

| | 28-oct | 16-nov | 16-nov + `control=1` |
|---|---|---|---|
| Error del ángulo a 360° (media, s) | −5.3 (12.3) | −3.8 (6.9) | −0.8 (6.1) |
| Error RMS de distancia, dref 15 | 4.2 cm | 2.8 cm | 2.2 cm |
| Error RMS de distancia, dref 25 | 11.8 cm | 4.5 cm | 3.5 cm |
| Vueltas fallidas (de 240) | 55 | 26 | 18 |

Con los sensores corregidos, el error que queda **solo por sensores** es de ±1.2° a 90°
y ±3.2° a 360°. Lo demás viene de cómo se pone el robot en la salida (el rumbo pesa
más que la distancia) y de que V y KU cambian de una corrida a otra.

V = 4 cm/s y KU = 0.0478 son del 25-oct. Hay que volver a medirlos.

## 4. Pruebas

Batería cargada, `modo=abierto`, `frec=3`, `amp=20`.

**Soporte**
1. `estado`: debe decir `mapa=1`, `escala=1.0165` y 50 líneas/s.
2. `ref=270`: las 4 patas verticales. Si una no, `cenN=±x`.
3. Láser contra una regla a 10, 15 y 20 cm: error menor a 0.5 cm.
4. Giro a mano de 360°: `phi` entre 355 y 365.

**Piso: medir la marcha**
- M1: `desfase=-45`, `ddesf=0`, `phi=0`, `marcha=1`, 20 s, `marcha=0`. V = cm / 20.
- M2: igual con `ddesf=-90`. KU = phi / 20 / (−90).
- M3: igual con `ddesf=-180`. KU = phi / 20 / (−180).
- Cargar `vel=V` y `kuff=KU`.

**Piso: órbita** (tubo a la derecha; girar el robot hasta que el láser marque la
distancia mínima, así queda tangente)
- `dref=15`, `phiobj=90`, `phi=0`, `orbita=1`. Tres veces por ángulo: 90, 180, 270, 360.
- Repetir con `control=1`.
- Luego `dref=10`, `20` y `25`.
