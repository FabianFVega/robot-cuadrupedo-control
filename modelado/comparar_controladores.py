# -*- coding: utf-8 -*-
"""
Comparacion de los controladores de la orbita en simulacion (21-nov, presentacion):
rn=0 PID en cascada, rn=1 perceptron, rn=2 MLP, rn=3 las dos, con control=1.
Usa el simulador de modelado/orbita_sensores_sim.py (sensores caracterizados,
robot distinto en cada corrida) y los pesos de modelado/pesos_redes.json.
Tambien prueba el caso con la planta invertida (kuff negativo, como la marcha del
20-nov con damp) para verificar que el signo se corrige.

Uso:  python modelado/comparar_controladores.py
"""
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import orbita_sensores_sim as S

N = 30
MODOS = [("rn=0 PID", 0, "tanh"), ("rn=1 perceptron", 1, "tanh"), ("rn=2 MLP", 2, "tanh"), ("rn=3 perceptron+MLP", 3, "tanh")]
base = dict(S.FW_NUEVO)
lineas = []
def out(t=""):
    print(t); lineas.append(t)

out(f"=== Controladores de la orbita, simulacion ({N} corridas por caso, control=1) ===")
out("modo                 | error RMS distancia 10/15/20/25 cm | vueltas fallidas | parada a 360: media (s)")
res = {}
for nom, rn, act in MODOS:
    fw = dict(base, rn=rn, rnact=act)
    drms, fall = [], 0
    for dref in (10.0, 15.0, 20.0, 25.0):
        q = S.lote(fw, 360, dref, n=N, semilla=21)
        drms.append(q["d_rms"].mean()); fall += int(np.sum((q["d_max"] > dref + 15) | (q["d_min"] < 1.0)))
    a = S.lote(fw, 360, 15.0, n=N, semilla=22)["angulo"]
    res[nom] = (drms, fall, a)
    out(f"{nom:20s} | " + " / ".join(f"{x:4.2f}" for x in drms) + f"          | {fall:3d}/{4 * N}       | {a.mean() - 360:+5.1f} ({a.std(ddof=1):4.1f})")

out()
out("Planta invertida (kuff negativo, limites 0..240): comprobacion del signo")
inv = dict(base, signo_planta=-1.0, kuff=-S.KU, umin=0.0, umax=240.0)
for nom, rn, act in MODOS:
    q = S.lote(dict(inv, rn=rn, rnact=act), 360, 15.0, n=10, semilla=5)
    out(f"  {nom:20s} error RMS a 15 cm = {q['d_rms'].mean():4.2f} cm, parada {q['angulo'].mean() - 360:+5.1f} grados")
out()
out("Perceptron con las tres activaciones (rn=1, dref 15)")
for act in ("sigmoide", "tanh", "lineal"):
    q = S.lote(dict(base, rn=1, rnact=act), 360, 15.0, n=N, semilla=21)
    out(f"  rnact={act:8s} error RMS = {q['d_rms'].mean():4.2f} cm, parada {q['angulo'].mean() - 360:+5.1f} ({q['angulo'].std(ddof=1):4.1f})")

os.makedirs(os.path.join(S.RAIZ, "resultados"), exist_ok=True)
fig, ax = plt.subplots(1, 2, figsize=(13, 4.6))
noms = list(res); x = np.arange(4)
for i, (nom, c) in enumerate(zip(noms, ("0.6", "tab:blue", "tab:orange", "tab:green"))):
    ax[0].bar(x + (i - 1.5) * 0.2, res[nom][0], 0.2, color=c, label=nom)
ax[0].set(xticks=x, xticklabels=["10", "15", "20", "25"], xlabel="dref (cm)", ylabel="error RMS de distancia (cm)",
          title="Distancia al tubo en una vuelta de 360°")
ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3, axis="y")
for i, (nom, c) in enumerate(zip(noms, ("0.6", "tab:blue", "tab:orange", "tab:green"))):
    a = res[nom][2] - 360
    ax[1].errorbar(i, a.mean(), yerr=a.std(ddof=1), fmt="o", color=c, capsize=5)
ax[1].axhline(0, color="k", lw=0.8)
ax[1].set(xticks=range(4), xticklabels=[n.split(" ", 1)[1] for n in noms], ylabel="recorrido − 360 (°)",
          title="Error del ángulo de parada (media ± s)")
ax[1].grid(alpha=0.3, axis="y")
fig.tight_layout(); fig.savefig(os.path.join(S.RAIZ, "resultados", "comparacion_controladores.png"), dpi=130)
with open(os.path.join(S.RAIZ, "resultados", "comparacion_controladores.txt"), "w", encoding="utf-8") as f:
    f.write("\n".join(lineas) + "\n")
print("Guardado: resultados/comparacion_controladores.png y .txt")
