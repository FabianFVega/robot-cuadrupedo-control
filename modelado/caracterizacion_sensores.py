# -*- coding: utf-8 -*-
"""
Correcciones de los sensores a partir de docs/Caracterizacion_de_sensores.xlsx (16-nov).

Lee las hojas del Excel y calcula lo que usa el firmware de la ESP principal:
  1. Giroscopio: escala del angulo integrado (ESCALA_GIRO).
  2. Laser: recta de calibracion nueva (LASER_A, LASER_B), compuesta con la que
     ya tenia el firmware cuando se tomaron las lecturas (0.891, 0.50).
  3. AS5600: por pata, la tabla "angulo que marca el sensor -> angulo real
     (transportador)" de 220 a 310 grados (realTab en el firmware).
Tambien deja en resultados/ las graficas y un .txt con las constantes.

Uso:  python modelado/caracterizacion_sensores.py
"""
import os
import numpy as np
import openpyxl
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXCEL = os.path.join(RAIZ, "docs", "Caracterizacion_de_sensores.xlsx")
SALIDA = os.path.join(RAIZ, "resultados")
# calibracion del laser que tenia el firmware al tomar las lecturas del Excel
LASER_A_ANT, LASER_B_ANT = 0.891, 0.50

lineas = []
def out(s=""):
    print(s)
    lineas.append(s)

wb = openpyxl.load_workbook(EXCEL, data_only=True)

# ------------------------------------------------------------------ giroscopio
ws = wb["Giroscopio"]
reales = np.array([ws.cell(4, c).value for c in range(2, 6)], float)        # 90..360
lect = np.array([[ws.cell(r, c).value for c in range(2, 6)] for r in range(5, 20)], float)
x = np.tile(reales, lect.shape[0]); y = lect.ravel()
m, b = np.polyfit(x, y, 1)
m0 = float(np.sum(x * y) / np.sum(x * x))            # recta por el origen
ESCALA_GIRO = 1.0 / m0
out("=== Giroscopio MPU6050 (giros a mano, 4 angulos x 15 pruebas) ===")
out(f"lectura = {m:.4f}*real + {b:.2f}   |  por el origen: lectura = {m0:.4f}*real")
out(f"ESCALA_GIRO = 1/{m0:.4f} = {ESCALA_GIRO:.4f}   (el giro integrado se multiplica por esto)")
out("real   media leida   s      error antes   error corregido")
for j, r in enumerate(reales):
    med, s = lect[:, j].mean(), lect[:, j].std(ddof=1)
    out(f"{r:4.0f}   {med:8.2f}    {s:5.2f}   {med - r:+7.2f}      {med * ESCALA_GIRO - r:+7.2f}")
s_giro = lect.std(axis=0, ddof=1)
# dispersion de las pruebas 9..15 (las mas repetibles) para el modelo
s_giro_fino = lect[8:, :].std(axis=0, ddof=1)
out(f"dispersion s (grados) en 90/180/270/360: {np.round(s_giro, 2)}  (pruebas 9-15: {np.round(s_giro_fino, 2)})")

# ------------------------------------------------------------------ laser
ws = wb["Sensor Láser"]
d_real = np.array([ws.cell(1, c).value for c in range(5, 10)], float)       # 25..5
d_lect = np.array([[ws.cell(r, c).value for c in range(5, 10)] for r in range(2, 17)], float)
x = np.tile(d_real, d_lect.shape[0]); y = d_lect.ravel()
ml, bl = np.polyfit(x, y, 1)
# lectura = A_ant*crudo + B_ant  y  real = (lectura - bl)/ml  ->  real = A*crudo + B
LASER_A = LASER_A_ANT / ml
LASER_B = (LASER_B_ANT - bl) / ml
out()
out("=== Laser VL53L0X (5 distancias x 15 pruebas) ===")
out(f"lectura = {ml:.4f}*real + {bl:.2f} cm  (lectura ya calibrada con {LASER_A_ANT}, {LASER_B_ANT})")
out(f"LASER_A = {LASER_A:.4f}   LASER_B = {LASER_B:.3f}   (real = A*crudo_cm + B)")
out("real   media leida   s      error antes   error corregido")
s_laser = d_lect.std(axis=0, ddof=1)
for j, r in enumerate(d_real):
    med = d_lect[:, j].mean()
    out(f"{r:4.0f}   {med:8.2f}    {s_laser[j]:5.2f}   {med - r:+7.2f}      {(med - bl) / ml - r:+7.2f}")
# ruido en funcion de la distancia (para el modelo): s = s0 + s1*d
s1, s0 = np.polyfit(d_real, s_laser, 1)
out(f"ruido del laser: s(d) ~ {s0:.3f} + {s1:.4f}*d cm")

# ------------------------------------------------------------------ AS5600
HOJAS = [("FI", "FI"), ("FD", "FD"), ("TI", "Sensores Hall TI"), ("TD", "TD")]
out()
out("=== AS5600 por pata: angulo del sensor (= ref en modo pid) -> angulo real ===")
tablas = {}
for nombre, hoja in HOJAS:
    ws = wb[hoja]
    ref = np.array([ws.cell(r, 13).value for r in range(2, 12)], float)
    real = np.array([[ws.cell(r, c).value for c in range(14, 24)] for r in range(2, 12)], float)
    med = real.mean(axis=1)
    tablas[nombre] = (ref, med, real.std(axis=1, ddof=1))
    assert np.all(np.diff(med) > 0), nombre + ": la tabla no es creciente"
    out(f"{nombre}: " + ", ".join(f"{v:.1f}" for v in med))
    i250, i270, i290 = 3, 5, 7
    out(f"    con ref=270 la pata queda en {med[i270]:.1f} (corrimiento {med[i270] - 270:+.1f});"
        f" amp=20 pedida -> recorre {med[i250]:.1f}..{med[i290]:.1f} = +-{(med[i290] - med[i250]) / 2:.1f}")
out()
out("Para el firmware (realTab, sensor 220..310 de 10 en 10):")
for nombre, _ in HOJAS:
    out("  {" + ", ".join(f"{v:.1f}" for v in tablas[nombre][1]) + "},   // " + nombre)

# ------------------------------------------------------------------ graficas
os.makedirs(SALIDA, exist_ok=True)
fig, ax = plt.subplots(1, 3, figsize=(15, 4.6))
ax[0].errorbar(reales, lect.mean(axis=0) - reales, yerr=s_giro, fmt="o-", capsize=4, label="antes")
ax[0].errorbar(reales, lect.mean(axis=0) * ESCALA_GIRO - reales, yerr=s_giro * ESCALA_GIRO, fmt="s-", capsize=4,
               label=f"corregido (x{ESCALA_GIRO:.4f})")
ax[0].axhline(0, color="k", lw=0.8)
ax[0].set(title="Giroscopio: error del angulo", xlabel="angulo real (°)", ylabel="lectura − real (°)")
ax[0].legend(); ax[0].grid(alpha=0.3)
ax[1].errorbar(d_real, d_lect.mean(axis=0) - d_real, yerr=s_laser, fmt="o-", capsize=4, label="antes")
ax[1].errorbar(d_real, (d_lect.mean(axis=0) - bl) / ml - d_real, yerr=s_laser / ml, fmt="s-", capsize=4, label="corregido")
ax[1].axhline(0, color="k", lw=0.8)
ax[1].set(title="Laser: error de la distancia", xlabel="distancia real (cm)", ylabel="lectura − real (cm)")
ax[1].legend(); ax[1].grid(alpha=0.3)
for nombre, _ in HOJAS:
    ref, med, _s = tablas[nombre]
    ax[2].plot(ref, med - ref, "o-", label=nombre)
ax[2].axhline(0, color="k", lw=0.8)
ax[2].set(title="Rodillas: angulo real − angulo del sensor", xlabel="angulo del sensor = ref (°)", ylabel="real − sensor (°)")
ax[2].legend(); ax[2].grid(alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(SALIDA, "caracterizacion_sensores.png"), dpi=130)
with open(os.path.join(SALIDA, "caracterizacion_sensores.txt"), "w", encoding="utf-8") as f:
    f.write("\n".join(lineas) + "\n")
print("\nGuardado: resultados/caracterizacion_sensores.png y .txt")
