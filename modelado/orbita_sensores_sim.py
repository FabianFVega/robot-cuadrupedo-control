# -*- coding: utf-8 -*-
"""
Orbita alrededor del tubo con los SENSORES CARACTERIZADOS (16-nov).

Modelo:
  - Robot: punto con rumbo (uniciclo). Avanza a V cm/s y gira a una velocidad
    que depende de ddesf (w = KU*ddesf, con una constante de tiempo), mas el
    vaiven del trote a 3 Hz. El tubo (11.5 cm) queda a su derecha: gira horario.
    Cada corrida cambia V, KU, el giro propio del robot y la posicion de salida
    (el robot real no repite): asi se ve si el control aguanta.
  - Giroscopio: lectura = G*w_real. G = 0.9838 (Excel) con una dispersion por
    prueba, mas un sesgo residual y ruido.
  - Laser: en el costado derecho, a 6 cm del centro, mirando de lado. Mide la
    distancia al tubo A LO LARGO DEL HAZ (si el robot no va tangente lee de mas, y
    con mas de ~12 grados de error de rumbo el haz ya no toca el tubo). La lectura
    cruda es la inversa de la recta nueva y el ruido crece con la distancia.
  - Control: el de la ESP principal (pasoOrbita) a 50 Hz: lazo interno de giro
    (PI sobre ddesf, con o sin prealimentacion) y lazo externo de distancia.

Los parametros de MARCHA (V y KU) son los del 25-oct. Con el servo nuevo, el
mapa de angulo real y los 395.2 g hay que volver a medirlos (README, pruebas
M1..M3) y cambiarlos aqui y en el firmware (vel= y kuff=).

Uso:  python modelado/orbita_sensores_sim.py
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SALIDA = os.path.join(RAIZ, "resultados")

# ------------------------------------------------------------------ robot
MASA = 0.3952            # kg (16-nov, con la bateria extra)
R_TUBO = 5.75            # cm
MEDIO_ANCHO = 6.0        # cm, del centro del robot al laser
# --- marcha (25-oct; VOLVER A MEDIR: pruebas M1..M3) ---
V = 4.0                  # cm/s con amp=20, frec=3
KU = 8.6 / 180.0         # (grados/s) por grado de ddesf: ddesf=-180 -> -8.6 grados/s
TAU_W = 0.4              # s, lo que tarda el giro en seguir a ddesf
VAIVEN_W = 25.0          # grados/s, vaiven del rumbo a 3 Hz (pico)
VAIVEN_D = 0.3           # cm, vaiven lateral del cuerpo a 3 Hz (pico)
FREC = 3.0
# --- lo que cambia de una corrida a otra (desviacion estandar) ---
S_V = 0.10               # 10 % en la velocidad de avance
S_KU = 0.15              # 15 % en lo que gira por cada grado de ddesf
S_GIRO_PROPIO = 1.0      # grados/s, giro del robot con ddesf=0 (se va a un lado)
S_D0 = 1.5               # cm, error al ponerlo a la distancia de salida
S_PSI0 = 5.0             # grados, error de rumbo al ponerlo

# ------------------------------------------------------------------ sensores (Excel)
G_GIRO = 0.9838          # lectura/real del angulo integrado
S_ESCALA = 0.005         # dispersion de la escala entre pruebas (pruebas 9-15: ~0.45 %)
S_SESGO = 0.05           # grados/s, sesgo residual (SUPUESTO: no sale del Excel)
RUIDO_GIRO = 1.0         # grados/s por muestra
A_REAL, B_REAL = 0.9721, -1.047      # real = A*crudo + B  (recta nueva)
def ruido_laser(d):                  # cm (Excel: 0.09 a 5 cm ... 0.60 a 25 cm)
    return max(0.05, -0.111 + 0.0240 * d)
FONDO = 80.0             # cm, lo que lee el laser cuando el haz no toca el tubo

# ------------------------------------------------------------------ firmware
TS = 0.02
RAMPA_S = 1.0
IELIM, DMAX = 4.0, 35.0
UMIN, UMAX = -240.0, 0.0

# firmware del 28-oct: sensores sin corregir, wref fijo y ddesf de partida fijo (u0)
FW_ANTES = dict(nombre="28-oct", escala=1.0, la=0.891, lb=0.50, vel=None, wref=-8.5, u0=-180.0,
                kuff=0.0, kpw=4.0, kiw=6.0, kpe=0.14, kde=2.0, kie=0.02, wlim=3.0, eps="no")
# firmware del 16-nov tal como arranca: sensores corregidos, wref = v/R segun dref (vel=) y
# prealimentacion: ddesf parte de (giro pedido / KU) y el PI solo corrige lo que falte
FW_SENSORES = dict(FW_ANTES, nombre="16-nov", escala=1.0165, la=0.9721, lb=-1.047, vel=V, kuff=KU)
# lo mismo con el lazo de distancia mas firme (comando control=1; elegido entre varios
# juegos de ganancias con 60 corridas por caso)
FW_NUEVO = dict(FW_SENSORES, nombre="16-nov + control=1", kpe=0.3, kde=5.0, kie=0.03, wlim=5.0)
# para aislar el efecto de los sensores: el control del 16-nov con los sensores sin corregir
FW_SIN_CORREGIR = dict(FW_SENSORES, nombre="sensores sin corregir", escala=1.0, la=0.891, lb=0.50)


def simular(fw, phi_obj, dref, rng, registrar=False, nominal=False):
    """Una orbita completa. Devuelve un dict con lo que paso de verdad."""
    k = 0.0 if nominal else 1.0
    v_run = V * (1.0 + k * S_V * rng.standard_normal())
    ku_run = KU * (1.0 + k * S_KU * rng.standard_normal())
    w_propio = k * S_GIRO_PROPIO * rng.standard_normal()
    d0 = dref + k * S_D0 * rng.standard_normal()
    Rc = R_TUBO + d0 + MEDIO_ANCHO
    x, y = Rc, 0.0                              # tubo a la derecha, avanza horario
    psi0_err = np.deg2rad(k * S_PSI0 * rng.standard_normal())
    psi = -np.pi / 2 + psi0_err
    w_act = 0.0                                 # grados/s
    g_run = G_GIRO * (1.0 + S_ESCALA * rng.standard_normal())
    sesgo = S_SESGO * rng.standard_normal()
    fase0 = rng.uniform(0, 2 * np.pi)
    wref = fw["wref"] if fw["vel"] is None else \
        -fw["vel"] / (R_TUBO + dref + MEDIO_ANCHO) * 180.0 / np.pi
    sg = 1.0 if wref >= 0 else -1.0
    u0 = fw["u0"]
    # estado del control
    phi = 0.0; wfilt = 0.0; iW = 0.0; iE = 0.0
    dist_f = -1.0; ddist_f = 0.0; t_sin = 0.0
    eps = 0.0; eps0 = 0.0; eps0_listo = (fw["eps"] == "cero")
    u = u0 if fw["kuff"] <= 0 else float(np.clip(wref / fw["kuff"], UMIN, UMAX))
    marcha = True; escala_m = 0.0
    ang_prev = np.arctan2(y, x); ang_rec = 0.0
    lect_laser = -1.0; t_laser = 0.0
    t = 0.0; dd = []; sin_laser = 0; n = 0
    reg = []
    while True:
        # --- rampa de la marcha
        obj = 1.0 if marcha else 0.0
        if escala_m < obj: escala_m = min(obj, escala_m + TS / RAMPA_S)
        elif escala_m > obj: escala_m = max(obj, escala_m - TS / RAMPA_S)
        if not marcha and escala_m <= 0.0:
            break
        # --- robot (paso de 20 ms)
        w_cmd = ku_run * u + w_propio
        w_act += TS / TAU_W * (w_cmd - w_act)
        w_real = escala_m * (w_act + VAIVEN_W * np.sin(2 * np.pi * FREC * t + fase0))
        psi += np.deg2rad(w_real) * TS
        x += escala_m * v_run * np.cos(psi) * TS
        y += escala_m * v_run * np.sin(psi) * TS
        ang = np.arctan2(y, x)
        da = (ang - ang_prev + np.pi) % (2 * np.pi) - np.pi
        ang_rec += da; ang_prev = ang
        t += TS
        # --- distancia a lo largo del haz del laser (hacia la derecha del robot)
        rx, ry = np.cos(psi - np.pi / 2), np.sin(psi - np.pi / 2)
        lx, ly = x + MEDIO_ANCHO * rx, y + MEDIO_ANCHO * ry
        b = lx * rx + ly * ry
        disc = b * b - (lx * lx + ly * ly - R_TUBO ** 2)
        toca = disc > 0 and (-b - np.sqrt(disc)) > 0
        d_haz = (-b - np.sqrt(disc)) + VAIVEN_D * np.sin(2 * np.pi * FREC * t + fase0 + 1.0) if toca else FONDO
        d_lat = np.hypot(x, y) - R_TUBO - MEDIO_ANCHO          # distancia lateral verdadera
        if escala_m >= 1.0:
            dd.append(d_lat)
        # --- sensores
        w_med = g_run * w_real + sesgo + RUIDO_GIRO * rng.standard_normal()
        t_laser += TS
        if t_laser >= 0.05:                                   # medicion nueva cada 50 ms
            t_laser = 0.0
            if toca:
                crudo = (d_haz - B_REAL) / A_REAL + ruido_laser(min(d_haz, 30.0)) / 0.891 * rng.standard_normal()
                lect_laser = fw["la"] * crudo + fw["lb"]
            else:
                lect_laser = FONDO
        # --- firmware: pasoOrbita
        w_fw = w_med * fw["escala"]
        phi += w_fw * TS
        if marcha:
            n += 1
            wfilt += TS / (1.0 + TS) * (w_fw - wfilt)
            d = lect_laser
            laser_ok = (3.0 < d < DMAX) and (dist_f < 0 or abs(d - dist_f) < 8.0 or t_sin > 0.4)
            if laser_ok:
                t_sin = 0.0
                if dist_f < 0: dist_f = d; ddist_f = 0.0
                nuevo = dist_f + TS / (0.3 + TS) * (d - dist_f)
                der = (nuevo - dist_f) / TS
                ddist_f += TS / (0.8 + TS) * (der - ddist_f)
                dist_f = nuevo
                e = np.clip(dist_f - dref, -10.0, 10.0)
                iE = np.clip(iE + fw["kie"] * e * TS, -IELIM, IELIM)
                corr = np.clip(fw["kpe"] * e + fw["kde"] * ddist_f + iE, -fw["wlim"], fw["wlim"])
                v_est = abs(wfilt) * np.pi / 180.0 * (R_TUBO + dist_f + MEDIO_ANCHO)
                if v_est > 1.0:
                    eps = -sg * np.rad2deg(np.arcsin(np.clip(ddist_f / v_est, -0.5, 0.5)))
                if not eps0_listo and abs(phi) > 5.0:
                    eps0 = eps; eps0_listo = True
            else:
                t_sin += TS; sin_laser += 1
                corr = iE
            wobj = wref + sg * corr
            ew = wobj - wfilt
            base = u0 if fw["kuff"] <= 0 else float(np.clip(wobj / fw["kuff"], UMIN, UMAX))
            iW = np.clip(iW + fw["kiw"] * ew * TS, UMIN - base, UMAX - base)
            u = float(np.clip(base + iW + fw["kpw"] * ew, UMIN, UMAX))
            ang_tubo = phi - ((eps - eps0) if eps0_listo else 0.0)
            rec = ang_tubo if (fw["eps"] != "no" and laser_ok) else phi
            if abs(rec) + abs(wfilt) * RAMPA_S * 0.5 >= phi_obj:
                marcha = False
        if registrar:
            reg.append((t, x, y, d_lat, -np.rad2deg(ang_rec), -phi, u, lect_laser))
        if t > 300:
            break
    dd = np.array(dd) if dd else np.array([d0])
    return dict(par=(v_run, ku_run, w_propio, d0 - dref, np.rad2deg(psi0_err)),
                angulo=-np.rad2deg(ang_rec), d_media=dd.mean(), d_min=dd.min(), d_max=dd.max(),
                d_rms=float(np.sqrt(np.mean((dd - dref) ** 2))), sin_laser=sin_laser / max(n, 1),
                reg=np.array(reg))


def lote(fw, phi_obj, dref, n=60, semilla=1, nominal=False):
    rng = np.random.default_rng(semilla)
    r = [simular(fw, phi_obj, dref, rng, nominal=nominal) for _ in range(n)]
    return {k: np.array([q[k] for q in r]) for k in ("angulo", "d_media", "d_min", "d_max", "d_rms", "sin_laser")}


if __name__ == "__main__":
    lineas = []
    def out(s=""):
        print(s); lineas.append(s)

    FWS = (FW_ANTES, FW_SENSORES, FW_NUEVO)
    N = 60
    out("=== Orbita con los sensores caracterizados (16-nov) ===")
    out(f"Marcha nominal: V = {V} cm/s, giro = {KU:.4f} grados/s por grado de ddesf (25-oct, volver a medir)")
    for fw in FWS:
        out(f"  [{fw['nombre']}] escala={fw['escala']} laser={fw['la']},{fw['lb']} "
            f"{'vel=' + str(fw['vel']) if fw['vel'] else 'wref=' + str(fw['wref'])} kuff={fw['kuff']:.4f} "
            f"kpe={fw['kpe']} kde={fw['kde']} kie={fw['kie']} wlim={fw['wlim']}")
    out()
    out("A) SOLO LOS SENSORES: robot nominal, bien puesto en la salida y con el mismo control (16-nov);")
    out("   lo unico que cambia es si el firmware corrige o no el giroscopio y el laser")
    out("   Angulo recorrido alrededor del tubo con dref = 15 cm: error medio (s) en grados")
    out("   objetivo |  sin corregir   | sensores corregidos")
    for obj in (90, 180, 270, 360):
        a0 = lote(FW_SIN_CORREGIR, obj, 15.0, n=N, nominal=True)["angulo"]
        a1 = lote(FW_SENSORES, obj, 15.0, n=N, nominal=True)["angulo"]
        out(f"     {obj:4d}   |  {a0.mean() - obj:+5.1f} ({a0.std(ddof=1):3.1f})   |  {a1.mean() - obj:+5.1f} ({a1.std(ddof=1):3.1f})")
    out("   Distancia lateral real media en una vuelta de 360 (cm) y error contra dref")
    out("   dref |  sin corregir   | sensores corregidos")
    for dref in (10.0, 15.0, 20.0, 25.0):
        d0 = lote(FW_SIN_CORREGIR, 360, dref, n=20, nominal=True)["d_media"]
        d1 = lote(FW_SENSORES, 360, dref, n=20, nominal=True)["d_media"]
        out(f"    {dref:3.0f} |  {d0.mean():5.1f} ({d0.mean() - dref:+4.1f})   |  {d1.mean():5.1f} ({d1.mean() - dref:+4.1f})")
    out()
    out(f"B) ROBOT REAL (cada corrida distinta: V +-{S_V * 100:.0f} %, KU +-{S_KU * 100:.0f} %, giro propio +-{S_GIRO_PROPIO} grados/s,")
    out(f"   salida +-{S_D0} cm y +-{S_PSI0} grados de rumbo), {N} corridas por caso")
    out("   Angulo recorrido alrededor del tubo, dref = 15 cm: error medio (s) en grados")
    out("   objetivo | " + " | ".join(f"{fw['nombre']:>20s}" for fw in FWS))
    res_ang = {}
    for obj in (90, 180, 270, 360):
        res_ang[obj] = [lote(fw, obj, 15.0, n=N, semilla=22)["angulo"] for fw in FWS]
        out(f"     {obj:4d}   | " + " | ".join(f"{a.mean() - obj:+11.1f} ({a.std(ddof=1):4.1f})" + " " * 2 for a in res_ang[obj]))
    out("   Vuelta de 360: error RMS de la distancia (cm), % del tiempo sin laser y vueltas fallidas")
    out("   (fallida = se aleja mas de 15 cm de dref o toca el tubo)")
    out("   dref | " + " | ".join(f"{fw['nombre']:>24s}" for fw in FWS))
    res_d = {}
    for dref in (10.0, 15.0, 20.0, 25.0):
        res_d[dref] = [lote(fw, 360, dref, n=N, semilla=21) for fw in FWS]
        out(f"    {dref:3.0f} | " + " | ".join(
            f"{r['d_rms'].mean():5.1f} cm  {r['sin_laser'].mean() * 100:3.0f} %  {int(np.sum((r['d_max'] > dref + 15) | (r['d_min'] < 1.0))):2d}/{N}  "
            for r in res_d[dref]))
    out("   De donde sale la dispersion del angulo (16-nov + control=1, dref 15): quitando una causa a la vez")
    import sys
    yo = sys.modules[__name__]
    for nom, chg in (("todas las causas", {}), ("sin error de rumbo al ponerlo", {"S_PSI0": 0.0}),
                     ("sin error de distancia al ponerlo", {"S_D0": 0.0}), ("sin variar V ni KU", {"S_V": 0.0, "S_KU": 0.0}),
                     ("solo el error de los sensores", {"S_PSI0": 0.0, "S_D0": 0.0, "S_V": 0.0, "S_KU": 0.0, "S_GIRO_PROPIO": 0.0})):
        viejo = {k_: getattr(yo, k_) for k_ in chg}
        for k_, v_ in chg.items(): setattr(yo, k_, v_)
        a90 = lote(FW_NUEVO, 90, 15.0, n=N, semilla=22)["angulo"]; a360 = lote(FW_NUEVO, 360, 15.0, n=N, semilla=22)["angulo"]
        out(f"     {nom:34s} s(90) = {a90.std(ddof=1):4.1f}   s(360) = {a360.std(ddof=1):4.1f}")
        for k_, v_ in viejo.items(): setattr(yo, k_, v_)

    # --- carga de las patas con el peso nuevo (estimacion)
    L_TIBIA = 0.055      # m  SUPUESTO: medir la tibia (eje de la rodilla -> punta del pie)
    PAR_SERVO = 1.8      # kg*cm, MG90S a 4.8 V (catalogo). Baja con el voltaje.
    F = MASA * 9.81 / 2  # N por pata: en el trote apoyan 2
    out()
    out(f"C) Carga de las rodillas con {MASA * 1000:.1f} g (trote: apoyan 2 patas, {F:.2f} N cada una)")
    out(f"   SUPUESTOS: tibia de {L_TIBIA * 100:.1f} cm; MG90S con {PAR_SERVO} kg*cm de par maximo")
    for a in (10, 20, 30):
        tau = F * L_TIBIA * np.sin(np.deg2rad(a))
        out(f"   pata a {a:2d} grados de la vertical: {tau * 1000:5.1f} mN*m = {tau / 9.81 * 100:.2f} kg*cm"
            f"  ({tau / 9.81 * 100 / PAR_SERVO * 100:.0f} % del par maximo)")

    # --- graficas
    os.makedirs(SALIDA, exist_ok=True)
    colores = ("tab:red", "tab:orange", "tab:blue")
    fig, ax = plt.subplots(1, 3, figsize=(17, 5.2))
    for fw, col in zip(FWS, colores):
        r = simular(fw, 360, 15.0, np.random.default_rng(11), registrar=True)
        reg = r["reg"]
        ax[0].plot(reg[:, 1], reg[:, 2], color=col, lw=1.2,
                   label=f"{fw['nombre']}: {r['angulo']:.0f}°, {r['d_min']:.0f}–{r['d_max']:.0f} cm")
        ax[0].plot(reg[-1, 1], reg[-1, 2], "o", color=col)
    th = np.linspace(0, 2 * np.pi, 100)
    ax[0].fill(R_TUBO * np.cos(th), R_TUBO * np.sin(th), color="0.6")
    ax[0].plot((R_TUBO + 15 + MEDIO_ANCHO) * np.cos(th), (R_TUBO + 15 + MEDIO_ANCHO) * np.sin(th), "k:", lw=0.8)
    ax[0].set_aspect("equal"); ax[0].legend(loc="upper center", fontsize=8)
    ax[0].set(title="La misma salida, vuelta de 360° con dref = 15 cm", xlabel="cm", ylabel="cm")
    objs = list(res_ang); w = 9
    for k, (fw, col) in enumerate(zip(FWS, colores)):
        med = [res_ang[o][k].mean() - o for o in objs]
        s = [res_ang[o][k].std(ddof=1) for o in objs]
        ax[1].bar(np.array(objs) + (k - 1) * w, med, w, yerr=s, capsize=3, color=col, label=fw["nombre"])
    ax[1].axhline(0, color="k", lw=0.8)
    ax[1].set(title="Error del angulo de parada (media y s)", xlabel="angulo objetivo (°)",
              ylabel="recorrido − objetivo (°)", xticks=objs)
    ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3, axis="y")
    ds = list(res_d)
    for k, (fw, col) in enumerate(zip(FWS, colores)):
        ax[2].bar(np.array(ds) + (k - 1) * 1.2, [res_d[d][k]["d_rms"].mean() for d in ds], 1.2, color=col,
                  label=fw["nombre"])
    ax[2].set(title="Error RMS de la distancia al tubo (vuelta de 360°)", xlabel="dref (cm)", ylabel="cm", xticks=ds)
    ax[2].legend(fontsize=8); ax[2].grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(SALIDA, "orbita_sensores_sim.png"), dpi=130)
    with open(os.path.join(SALIDA, "orbita_sensores_sim.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lineas) + "\n")
    print("\nGuardado: resultados/orbita_sensores_sim.png y .txt")
