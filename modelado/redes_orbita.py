# -*- coding: utf-8 -*-
"""
Redes neuronales para la orbita (21-nov, presentacion).

  1. Perceptron (lazo interno de GIRO). Entradas: e_w (error de giro, grados/s) y
     de_w (cambio del error en un ciclo de 20 ms). Salida: u_hat en [0, 1] y
         du = 8 * (2*u_hat - 1)        (unidades del actuador por ciclo)
     que se suma al actuador (forma incremental). Se entrena con la REGLA DELTA para
     imitar el incremento del PI del firmware:  du_PI = KIW*e_w*Ts + KPW*de_w.
     Tres activaciones: sigmoide, tanh y lineal (con la lineal es exactamente el PI).

  2. MLP 2-8-1 (lazo externo de DISTANCIA). Entradas: e_d (cm, limitado a +-10) y
     de_d (cm/s). Salida: corr (grados/s) = WLIM * tanh(salida). Capa oculta de 8
     neuronas tanh. Se entrena por RETROPROPAGACION contra la regla experta
         corr = WLIM * tanh((KPE*e + KDE*de) / WLIM)
     (el PD del firmware con saturacion suave). Un juego de pesos para control=1 y
     otro para control=0.

Escribe:
  firmware/esp_principal_pid_4patas_v2/pesos_redes.h   (lo incluye el firmware)
  firmware/esp_principal_octubre/pesos_redes.h          (la version de octubre)
  modelado/pesos_redes.json                              (lo usa la simulacion)
  resultados/redes_entrenamiento.png y redes_superficies.png

Uso:  python modelado/redes_orbita.py
"""
import os, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TS = 0.02
KPW, KIW = 4.0, 6.0
EW_N, DEW_N = 10.0, 1.0          # normalizacion del perceptron
ED_N, DED_N = 10.0, 3.0          # normalizacion de la MLP
DU_MAX = 8.0
JUEGOS = {"control1": dict(kpe=0.30, kde=5.0, wlim=5.0),
          "control0": dict(kpe=0.14, kde=2.0, wlim=3.0)}
rng = np.random.default_rng(7)


# ------------------------------------------------------------------ perceptron
def datos_perceptron(n):
    e = rng.uniform(-10, 10, n)
    de = rng.normal(0, 0.35, n)
    du = KIW * e * TS + KPW * de
    y = np.clip((du / DU_MAX + 1) / 2, 0.001, 0.999)
    X = np.column_stack([e / EW_N, de / DEW_N])
    return X, y, du


def act(v, tipo):
    if tipo == "sigmoide": return 1 / (1 + np.exp(-v))
    if tipo == "tanh": return (np.tanh(v) + 1) / 2
    return (v + 1) / 2                                  # lineal: u = (v + 1)/2


def dact(v, tipo):
    if tipo == "sigmoide": s = 1 / (1 + np.exp(-v)); return s * (1 - s)
    if tipo == "tanh": return (1 - np.tanh(v) ** 2) / 2
    return 0.5 * np.ones_like(v)


def entrenar_perceptron(tipo, X, y, Xv, yv, epocas=60, eta=0.5):
    w = rng.normal(0, 0.1, 3)                           # [sesgo, w_e, w_de]
    hist = []
    for ep in range(epocas):
        for i in rng.permutation(len(X)):              # regla delta, muestra por muestra
            v = w[0] + X[i] @ w[1:]
            err = y[i] - act(v, tipo)
            g = eta * err * dact(np.array([v]), tipo)[0]
            w[0] += g; w[1:] += g * X[i]
        vv = w[0] + Xv @ w[1:]
        hist.append(np.mean((yv - act(vv, tipo)) ** 2))
    return w, np.array(hist)


# ------------------------------------------------------------------ MLP 2-8-1
def regla(e, de, j):
    return j["wlim"] * np.tanh((j["kpe"] * e + j["kde"] * de) / j["wlim"])


def datos_mlp(n, j):
    e = rng.uniform(-10, 10, n)
    de = rng.uniform(-3, 3, n)
    X = np.column_stack([e / ED_N, de / DED_N])
    t = regla(e, de, j) / j["wlim"]                      # objetivo en [-1, 1]
    return X, t


def entrenar_mlp(X, t, Xv, tv, oculta=8, epocas=3000, eta=0.05, lote=64):
    W1 = rng.normal(0, 0.8, (oculta, 2)); b1 = np.zeros(oculta)
    W2 = rng.normal(0, 0.5, oculta); b2 = 0.0
    hist = []
    for ep in range(epocas):
        idx = rng.permutation(len(X))
        for k in range(0, len(X), lote):
            b = idx[k:k + lote]
            h = np.tanh(X[b] @ W1.T + b1)
            o = np.tanh(h @ W2 + b2)
            d_o = (o - t[b]) * (1 - o ** 2)              # retropropagacion
            gW2 = h.T @ d_o / len(b); gb2 = d_o.mean()
            d_h = np.outer(d_o, W2) * (1 - h ** 2)
            gW1 = d_h.T @ X[b] / len(b); gb1 = d_h.mean(axis=0)
            W2 -= eta * gW2; b2 -= eta * gb2; W1 -= eta * gW1; b1 -= eta * gb1
        if ep % 20 == 0 or ep == epocas - 1:
            ov = np.tanh(np.tanh(Xv @ W1.T + b1) @ W2 + b2)
            hist.append((ep, np.mean((tv - ov) ** 2)))
    return dict(W1=W1, b1=b1, W2=W2, b2=b2), np.array(hist)


def mlp(red, X):
    return np.tanh(np.tanh(X @ red["W1"].T + red["b1"]) @ red["W2"] + red["b2"])


# ------------------------------------------------------------------ salida .h
def arr(v):
    return "{" + ", ".join(f"{x:.7f}f" for x in np.ravel(v)) + "}"


def escribir_h(ruta, per, redes):
    os.makedirs(os.path.dirname(ruta), exist_ok=True)
    L = ["// Generado por modelado/redes_orbita.py -- no editar a mano",
         "#pragma once",
         f"#define RN_EW_N {EW_N:.3f}f", f"#define RN_DEW_N {DEW_N:.3f}f",
         f"#define RN_ED_N {ED_N:.3f}f", f"#define RN_DED_N {DED_N:.3f}f",
         f"#define RN_DU_MAX {DU_MAX:.3f}f",
         "// perceptron: [sesgo, w_e, w_de] por activacion (0 sigmoide, 1 tanh, 2 lineal)",
         "const float PER_W[3][3] = {" + ", ".join(arr(per[t]) for t in ("sigmoide", "tanh", "lineal")) + "};",
         "// MLP 2-8-1: [0] = control=0, [1] = control=1"]
    L.append("const float MLP_W1[2][8][2] = {" + ", ".join(
        "{" + ", ".join(arr(r) for r in redes[j]["W1"]) + "}" for j in ("control0", "control1")) + "};")
    L.append("const float MLP_B1[2][8] = {" + ", ".join(arr(redes[j]["b1"]) for j in ("control0", "control1")) + "};")
    L.append("const float MLP_W2[2][8] = {" + ", ".join(arr(redes[j]["W2"]) for j in ("control0", "control1")) + "};")
    L.append("const float MLP_B2[2] = {" + ", ".join(f"{redes[j]['b2']:.7f}f" for j in ("control0", "control1")) + "};")
    with open(ruta, "w") as f:
        f.write("\n".join(L) + "\n")


if __name__ == "__main__":
    # --- perceptron
    X, y, _ = datos_perceptron(1500)
    Xv, yv, duv = datos_perceptron(600)
    per, hper = {}, {}
    for tipo in ("sigmoide", "tanh", "lineal"):
        per[tipo], hper[tipo] = entrenar_perceptron(tipo, X, y, Xv, yv)
        vv = per[tipo][0] + Xv @ per[tipo][1:]
        du_red = DU_MAX * (2 * act(vv, tipo) - 1)
        print(f"perceptron {tipo:8s}: w = {np.round(per[tipo], 4)}  MSE val = {hper[tipo][-1]:.2e}  "
              f"error RMS en du = {np.sqrt(np.mean((du_red - duv) ** 2)):.4f}")
    # --- MLP
    redes, hmlp = {}, {}
    for j, g in JUEGOS.items():
        Xm, t = datos_mlp(3000, g); Xmv, tv = datos_mlp(800, g)
        redes[j], hmlp[j] = entrenar_mlp(Xm, t, Xmv, tv)
        err = (mlp(redes[j], Xmv) - tv) * g["wlim"]
        print(f"MLP {j}: MSE val = {hmlp[j][-1, 1]:.2e}  error RMS en corr = {np.sqrt(np.mean(err ** 2)):.3f} grados/s"
              f"  (max {np.max(np.abs(err)):.3f})")
    # --- salidas
    for carpeta in ("esp_principal_pid_4patas_v2", "esp_principal_octubre"):
        escribir_h(os.path.join(RAIZ, "firmware", carpeta, "pesos_redes.h"), per, redes)
    with open(os.path.join(RAIZ, "modelado", "pesos_redes.json"), "w") as f:
        json.dump(dict(norm=dict(EW_N=EW_N, DEW_N=DEW_N, ED_N=ED_N, DED_N=DED_N, DU_MAX=DU_MAX),
                       perceptron={k: v.tolist() for k, v in per.items()},
                       mlp={j: {k: np.asarray(v).tolist() for k, v in r.items()} for j, r in redes.items()},
                       juegos=JUEGOS), f, indent=1)
    # --- graficas
    os.makedirs(os.path.join(RAIZ, "resultados"), exist_ok=True)
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
    for tipo, c in zip(("sigmoide", "tanh", "lineal"), ("0.55", "tab:blue", "0.25")):
        ax[0].semilogy(np.arange(1, len(hper[tipo]) + 1), hper[tipo], color=c, label=tipo)
    ax[0].set(title="Perceptron (giro): regla delta", xlabel="epoca", ylabel="MSE de validacion")
    ax[0].legend(); ax[0].grid(alpha=0.3)
    for j, c in zip(JUEGOS, ("tab:blue", "0.5")):
        ax[1].semilogy(hmlp[j][:, 0], hmlp[j][:, 1], color=c, label=j)
    ax[1].set(title="MLP 2-8-1 (distancia): retropropagacion", xlabel="epoca", ylabel="MSE de validacion")
    ax[1].legend(); ax[1].grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(RAIZ, "resultados", "redes_entrenamiento.png"), dpi=130)
    g = JUEGOS["control1"]
    E, D = np.meshgrid(np.linspace(-10, 10, 41), np.linspace(-3, 3, 41))
    Xg = np.column_stack([E.ravel() / ED_N, D.ravel() / DED_N])
    Z = (mlp(redes["control1"], Xg) * g["wlim"]).reshape(E.shape)
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.6))
    for a, (Zp, tit) in zip(ax, ((regla(E, D, g), "Regla experta (PD saturado), control=1"), (Z, "MLP 2-8-1 entrenada"))):
        cs = a.contourf(E, D, Zp, levels=np.linspace(-5, 5, 21), cmap="RdBu_r")
        a.set(title=tit, xlabel="error de distancia e (cm)", ylabel="de/dt (cm/s)")
    fig.colorbar(cs, ax=ax, label="corr (grados/s)")
    fig.savefig(os.path.join(RAIZ, "resultados", "redes_superficies.png"), dpi=130)
    print("Escritos pesos_redes.h, pesos_redes.json y las graficas en resultados/")
