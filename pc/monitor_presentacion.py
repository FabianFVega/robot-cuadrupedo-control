# -*- coding: utf-8 -*-
"""
MONITOR DE LA PRESENTACION (21-nov) -- robot cuadrupedo con control inteligente

Una sola ventana para todas las pruebas, conectada a la ESP principal por WiFi:
  - Rodillas: referencia contra angulo medido de las 4 patas, y prueba de ESCALON
    (boton) que calcula sobrepaso, asentamiento y error final por pata y guarda la
    grafica. Sirve para comparar PI-D (pidtipo=pi-d) contra PID discreto (pidtipo=pid).
  - Orbita: giro medido contra objetivo, distancia contra dref (gris = el laser no ve
    el tubo), actuador y correcciones (aplicada, MLP y perceptron), avance del angulo.
  - Redes: el perceptron y la MLP 2-8-1 dibujados EN VIVO. Grosor = |peso| (rojo +,
    azul -); el color de cada neurona es su valor en ese instante. Las dos redes se
    calculan siempre: con rn=0 se ve "en sombra" lo que harian frente al PID.
  - Botones: rn=0..3, orbita, parar, marcha, sesgo, escalon, pidtipo; caja de comandos.
  - Guarda todo en resultados/monitor_<fecha>.csv y una tabla de las orbitas terminadas
    en resultados/orbitas_<fecha>.csv.

Necesita el firmware del 21-nov (manda las lineas @orb al recibir orbtele=1) y
pesos_redes.json (junto a este archivo o en ../modelado/).

Uso:
  1. Conectar la PC a la red WiFi RobotPID (clave 12345678).
  2. pip install numpy matplotlib
  3. python monitor_presentacion.py
Solo un programa puede estar conectado a la ESP: cerrar pc_pid_4patas.py antes.
"""
import os, sys, json, time, socket, threading, collections, datetime
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection

HOST, PUERTO = "192.168.4.1", 80
VENTANA_S = 30.0                       # segundos que muestran las graficas
AQUI = os.path.dirname(os.path.abspath(__file__))
CARPETA_RES = os.path.join(AQUI, "resultados")
os.makedirs(CARPETA_RES, exist_ok=True)
SELLO = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
PATAS = ["FI", "FD", "TI", "TD"]
RN_NOMBRE = {0: "PID", 1: "perceptron", 2: "MLP", 3: "perceptron + MLP"}
ACT_NOMBRE = {0: "sigmoide", 1: "tanh", 2: "lineal"}


# ------------------------------------------------------------------ pesos
def cargar_pesos():
    for r in (os.path.join(AQUI, "pesos_redes.json"), os.path.join(AQUI, "..", "modelado", "pesos_redes.json")):
        if os.path.exists(r):
            with open(r) as f:
                return json.load(f)
    print("AVISO: no encuentro pesos_redes.json: las redes no se dibujan")
    return None


PESOS = cargar_pesos()


# ------------------------------------------------------------------ enlace
class Enlace:
    def __init__(self):
        self.sock = None
        self.vivo = True
        self.lock = threading.Lock()
        n = 3000
        self.tele = collections.deque(maxlen=n)       # (t_pc, t_esp, ref[4], th[4], w[4], giro, phi, dist, damp, ddesf)
        self.orb = collections.deque(maxlen=n)        # dict por linea @orb
        self.mensajes = collections.deque(maxlen=9)
        self.diag = {}
        self.orbitas = []                             # resultados de orbitas terminadas
        self.ultimo_orb = None
        self.t0 = time.time()
        self.f_log = open(os.path.join(CARPETA_RES, f"monitor_{SELLO}.csv"), "w", encoding="utf-8")
        self.f_log.write("t_pc,linea\n")
        threading.Thread(target=self._hilo, daemon=True).start()

    def enviar(self, cmd):
        cmd = cmd.strip()
        if not cmd:
            return
        try:
            with self.lock:
                if self.sock:
                    self.sock.sendall((cmd + "\n").encode())
            print("  enviado:", cmd)
            self.mensajes.append("> " + cmd)
        except OSError as e:
            print("  no se pudo enviar:", e)

    def _hilo(self):
        while self.vivo:
            try:
                print(f"Conectando a {HOST}:{PUERTO} ...")
                s = socket.create_connection((HOST, PUERTO), timeout=5)
                s.settimeout(4)
                with self.lock:
                    self.sock = s
                print("Conectado.")
                self.mensajes.append("Conectado a la ESP")
                time.sleep(0.3)
                self.enviar("orbtele=1")
                buf = b""
                while self.vivo:
                    datos = s.recv(4096)
                    if not datos:
                        raise OSError("la ESP cerro la conexion")
                    buf += datos
                    while b"\n" in buf:
                        linea, buf = buf.split(b"\n", 1)
                        self._linea(linea.decode(errors="replace").strip())
            except OSError as e:
                with self.lock:
                    self.sock = None
                print(f"  sin conexion ({e}). ¿La PC esta en la red RobotPID? Reintento en 2 s")
                time.sleep(2)

    def _linea(self, l):
        if not l:
            return
        tpc = time.time() - self.t0
        self.f_log.write(f"{tpc:.3f},\"{l}\"\n")
        if l.startswith("#"):
            m = l[1:].strip()
            print("[ESP]", m)
            clave = m.split(":")[0][:30]
            self.diag[clave] = m
            if not m.startswith(("estado sensores", "tiempos", "lineas danadas", "giroscopio", "orbita:",
                                 "patas en lazo", "lecturas descartadas", "ultima linea")):
                self.mensajes.append(m[:150])
            if "orbita terminada" in m:
                self._orbita_terminada(m)
            return
        p = l.split(",")
        if p[0] == "@orb" and len(p) >= 23:
            try:
                v = [float(x) for x in p[1:]]
            except ValueError:
                return
            d = dict(t=tpc, activa=v[1], wf=v[2], wobj=v[3], ew=v[4], dew=v[5], dist=v[6], dref=v[7],
                     laser=v[8], ed=v[9], ded=v[10], u=v[11], corr=v[12], corr_mlp=v[13], du_per=v[14],
                     rn=int(v[15]), rnact=int(v[16]), rec=v[17], phiobj=v[18], act=int(v[19]),
                     juego=int(v[20]), us=v[21])
            self.orb.append(d); self.ultimo_orb = d
            return
        if len(p) in (17, 20, 21, 22) and p[0].isdigit():
            def f(x):
                try:
                    return float(x)
                except ValueError:
                    return np.nan
            v = [f(x) for x in p]
            ref = [v[1 + 4 * i] for i in range(4)]
            th = [v[2 + 4 * i] for i in range(4)]
            w = [v[4 + 4 * i] for i in range(4)]
            extra = v[17:] + [np.nan] * (5 - len(v[17:]))
            self.tele.append((tpc, ref, th, w, *extra))

    def _orbita_terminada(self, m):
        o = self.ultimo_orb or {}
        fila = dict(hora=datetime.datetime.now().strftime("%H:%M:%S"), rn=RN_NOMBRE.get(o.get("rn", -1), "?"),
                    activacion=ACT_NOMBRE.get(o.get("rnact", -1), "?"), phiobj=o.get("phiobj", np.nan),
                    mensaje=m)
        # distancia minima / maxima durante la ultima orbita
        ds = [d["dist"] for d in self.orb if d["activa"] and d["laser"] and d["t"] > (o.get("t", 0) - 600)]
        tramo = []
        for d in reversed(self.orb):
            if not d["activa"] and tramo:
                break
            if d["activa"]:
                tramo.append(d)
        dl = [d["dist"] for d in tramo if d["laser"]]
        fila["dist_min"] = min(dl) if dl else np.nan
        fila["dist_max"] = max(dl) if dl else np.nan
        fila["err_rms"] = float(np.sqrt(np.mean([(d["dist"] - d["dref"]) ** 2 for d in tramo if d["laser"]]))) if dl else np.nan
        fila["sin_laser_%"] = 100.0 * (1 - len(dl) / len(tramo)) if tramo else np.nan
        self.orbitas.append(fila)
        ruta = os.path.join(CARPETA_RES, f"orbitas_{SELLO}.csv")
        with open(ruta, "w", encoding="utf-8") as f:
            cols = list(fila)
            f.write(",".join(cols) + "\n")
            for r in self.orbitas:
                f.write(",".join(f"\"{r[c]}\"" if isinstance(r[c], str) else f"{r[c]:.2f}" for c in cols) + "\n")
        print("\n=== ORBITAS TERMINADAS ===")
        for r in self.orbitas:
            print(f"  {r['hora']}  {r['rn']:16s} phiobj={r['phiobj']:.0f}  dist {r['dist_min']:.1f}..{r['dist_max']:.1f} cm"
                  f"  RMS {r['err_rms']:.2f} cm  sin laser {r['sin_laser_%']:.0f} %")
        print(f"  (guardado en {ruta})\n")


# ------------------------------------------------------------------ prueba de escalon
def escalon(enlace, base=265.0, paso=10.0, t_espera=2.0, t_med=2.5):
    def correr():
        print(f"\n=== ESCALON {base:.0f} -> {base + paso:.0f} (debe estar en modo=pid, en el soporte) ===")
        enlace.enviar(f"ref={base:.0f}")
        time.sleep(t_espera)
        t_ini = time.time() - enlace.t0
        enlace.enviar(f"ref={base + paso:.0f}")
        time.sleep(t_med)
        datos = [d for d in list(enlace.tele) if d[0] >= t_ini - 0.2]
        enlace.enviar(f"ref={base:.0f}")
        if len(datos) < 10:
            print("  no llego telemetria suficiente")
            return
        t = np.array([d[0] for d in datos]) - t_ini
        fig, ax = plt.subplots(1, 4, figsize=(15, 3.6), sharey=True)
        tipo = enlace.diag.get("OK pidtipo", "")
        res = []
        for i, nom in enumerate(PATAS):
            th = np.array([d[2][i] for d in datos])
            ok = ~np.isnan(th)
            ax[i].plot(t, [base + paso if x >= 0 else base for x in t], "k--", lw=1, label="referencia")
            ax[i].plot(t[ok], th[ok], color="tab:blue", label="medido")
            ax[i].set(title=nom, xlabel="s"); ax[i].grid(alpha=0.3)
            tt, yy = t[ok & (t >= 0)], th[ok & (t >= 0)]
            if len(yy) < 5:
                res.append((nom, np.nan, np.nan, np.nan)); continue
            sob = max(0.0, (yy.max() - (base + paso)) / paso * 100)
            fin = np.mean(yy[-5:]) - (base + paso)
            fuera = np.where(np.abs(yy - (base + paso)) > 0.05 * paso)[0]
            ts = tt[fuera[-1]] if len(fuera) and fuera[-1] + 1 < len(tt) else (np.nan if len(fuera) else 0.0)
            res.append((nom, sob, ts, fin))
        ax[0].set_ylabel("angulo (grados)"); ax[0].legend(fontsize=8)
        fig.suptitle(f"Escalon de {paso:.0f} grados  {tipo[:80]}")
        ruta = os.path.join(CARPETA_RES, f"escalon_{datetime.datetime.now():%H%M%S}.png")
        fig.tight_layout(); fig.savefig(ruta, dpi=110); plt.close(fig)
        print("  pata | sobrepaso | asentamiento (5 %) | error final")
        for nom, sob, ts, fin in res:
            print(f"  {nom}   | {sob:6.1f} % | {ts:6.2f} s           | {fin:+5.2f} grados")
        print(f"  grafica: {ruta}\n")
        enlace.mensajes.append(f"escalon listo: {ruta}")
    threading.Thread(target=correr, daemon=True).start()


# ------------------------------------------------------------------ dibujo de las redes
def capas_red(ax, posiciones, pesos):
    """posiciones: lista de capas, cada una una lista de (x, y). pesos: lista de matrices (n_sig x n_ant)."""
    segs, anchos, colores = [], [], []
    for k, W in enumerate(pesos):
        for j, (xj, yj) in enumerate(posiciones[k + 1]):
            for i, (xi, yi) in enumerate(posiciones[k]):
                w = W[j][i]
                segs.append([(xi, yi), (xj, yj)])
                anchos.append(0.4 + 2.6 * min(abs(w), 2.0) / 2.0)
                colores.append((0.8, 0.2, 0.15, 0.55) if w > 0 else (0.15, 0.35, 0.8, 0.55))
    ax.add_collection(LineCollection(segs, linewidths=anchos, colors=colores, zorder=1))
    pts = [p for capa in posiciones for p in capa]
    sc = ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=230, c=np.zeros(len(pts)), cmap="RdBu_r",
                    vmin=-1, vmax=1, edgecolors="0.3", zorder=2)
    return sc


def main():
    enlace = Enlace()
    plt.rcParams.update({"font.size": 8})
    fig = plt.figure(figsize=(16, 9.2))
    fig.canvas.manager.set_window_title("Monitor de la presentacion - robot cuadrupedo")
    gs = fig.add_gridspec(4, 4, left=0.05, right=0.98, top=0.95, bottom=0.2, hspace=0.55, wspace=0.28)
    ax_p = [fig.add_subplot(gs[i // 2, i % 2]) for i in range(4)]
    ax_w = fig.add_subplot(gs[2, 0:2]); ax_d = fig.add_subplot(gs[3, 0:2])
    ax_u = fig.add_subplot(gs[0, 2:4]); ax_per = fig.add_subplot(gs[1:3, 2]); ax_mlp = fig.add_subplot(gs[1:3, 3])
    ax_txt = fig.add_subplot(gs[3, 2:4]); ax_txt.axis("off")
    lin_p = []
    for i, a in enumerate(ax_p):
        r, = a.plot([], [], "k--", lw=1, label="ref"); m, = a.plot([], [], color="tab:blue", lw=1.2, label="medido")
        a.set_title(f"Rodilla {PATAS[i]}"); a.grid(alpha=0.3); lin_p.append((r, m))
    ax_p[0].legend(loc="upper left", fontsize=7)
    lw_f, = ax_w.plot([], [], color="tab:blue", label="giro medido (filtrado)")
    lw_o, = ax_w.plot([], [], "k--", lw=1, label="giro objetivo")
    ax_w.set(title="Orbita: lazo interno de giro", ylabel="grados/s"); ax_w.grid(alpha=0.3); ax_w.legend(fontsize=7, loc="upper left")
    ld, = ax_d.plot([], [], color="tab:blue", label="distancia filtrada"); ldr, = ax_d.plot([], [], "k--", lw=1, label="dref")
    ax_d.set(title="Orbita: lazo externo de distancia (gris = sin tubo)", ylabel="cm", xlabel="s"); ax_d.grid(alpha=0.3)
    ax_d.legend(fontsize=7, loc="upper left")
    lu, = ax_u.plot([], [], color="0.3", label="actuador u (damp o ddesf)")
    ax_u2 = ax_u.twinx()
    lc, = ax_u2.plot([], [], color="tab:blue", label="corr aplicada")
    lcm, = ax_u2.plot([], [], color="tab:orange", lw=1, label="corr MLP")
    ldp, = ax_u2.plot([], [], color="tab:green", lw=0.8, label="du perceptron x10")
    ax_u.set(title="Actuador y correcciones"); ax_u.grid(alpha=0.3)
    ax_u2.set_ylabel("grados/s")
    ax_u.legend(handles=[lu, lc, lcm, ldp], fontsize=7, loc="upper left", ncol=2)

    # --- redes
    sc_per = sc_mlp = None
    if PESOS:
        pos_per = [[(0, 0.7), (0, 0.3)], [(1, 0.5)], [(2, 0.5)]]
        w_per = PESOS["perceptron"]["tanh"]
        sc_per = capas_red(ax_per, pos_per, [[w_per[1:]], [[1.0]]])
        for (x, y), t in zip([(0, 0.7), (0, 0.3), (1, 0.5), (2, 0.5)], ["e_w", "Δe_w", "v → û", "Δu"]):
            ax_per.text(x, y - 0.09, t, ha="center", fontsize=8)
        ax_per.set(xlim=(-0.4, 2.4), ylim=(0.1, 0.9)); ax_per.axis("off")
        pos_mlp = [[(0, 0.65), (0, 0.35)], [(1, 0.08 + 0.12 * i) for i in range(8)], [(2, 0.5)]]
        red = PESOS["mlp"]["control1"]
        sc_mlp = capas_red(ax_mlp, pos_mlp, [red["W1"], [red["W2"]]])
        for (x, y), t in zip([(0, 0.65), (0, 0.35), (2, 0.5)], ["e_d", "de_d", "corr"]):
            ax_mlp.text(x, y - 0.07, t, ha="center", fontsize=8)
        ax_mlp.set(xlim=(-0.4, 2.4), ylim=(-0.02, 1.02)); ax_mlp.axis("off")
    tit_per = ax_per.set_title("Perceptron (giro)"); tit_mlp = ax_mlp.set_title("MLP 2-8-1 (distancia)")
    txt = ax_txt.text(0, 1, "", va="top", family="monospace", fontsize=7.5)

    # --- controles
    caja = TextBox(fig.add_axes([0.05, 0.11, 0.36, 0.035]), "comando ", initial="")
    def al_enviar(t):
        enlace.enviar(t); caja.set_val("")
    caja.on_submit(al_enviar)
    botones = []
    def boton(x, y, w, nombre, accion):
        b = Button(fig.add_axes([x, y, w, 0.035]), nombre)
        b.on_clicked(lambda _e: accion()); botones.append(b)
    fila1 = [("rn=0 PID", "rn=0"), ("rn=1 perceptron", "rn=1"), ("rn=2 MLP", "rn=2"), ("rn=3 ambas", "rn=3"),
             ("orbita ▶", "orbita=1"), ("parar ■", "orbita=0"), ("marcha ▶", "marcha=1"), ("marcha ■", "marcha=0")]
    for i, (n, c) in enumerate(fila1):
        boton(0.45 + i * 0.066, 0.11, 0.062, n, lambda c=c: enlace.enviar(c))
    fila2 = [("phi=0", lambda: enlace.enviar("phi=0")), ("sesgo (quieto)", lambda: enlace.enviar("sesgo=1")),
             ("modo=abierto", lambda: enlace.enviar("modo=abierto")), ("modo=pid", lambda: enlace.enviar("modo=pid")),
             ("pidtipo=pi-d", lambda: enlace.enviar("pidtipo=pi-d")), ("pidtipo=pid", lambda: enlace.enviar("pidtipo=pid")),
             ("ESCALON", lambda: escalon(enlace)), ("ref=270", lambda: enlace.enviar("ref=270"))]
    for i, (n, f) in enumerate(fila2):
        boton(0.05 + i * 0.075, 0.05, 0.07, n, f)
    boton(0.66, 0.05, 0.07, "rnact=tanh", lambda: enlace.enviar("rnact=tanh"))
    boton(0.735, 0.05, 0.07, "rnact=lineal", lambda: enlace.enviar("rnact=lineal"))
    boton(0.81, 0.05, 0.07, "control=1", lambda: enlace.enviar("control=1"))
    boton(0.885, 0.05, 0.07, "control=0", lambda: enlace.enviar("control=0"))

    sombras = []

    def actualizar(_):
        ahora = time.time() - enlace.t0
        t0 = ahora - VENTANA_S
        tele = [d for d in list(enlace.tele) if d[0] >= t0]
        if tele:
            t = np.array([d[0] for d in tele])
            for i in range(4):
                lin_p[i][0].set_data(t, [d[1][i] for d in tele])
                lin_p[i][1].set_data(t, [d[2][i] for d in tele])
                ax_p[i].set_xlim(t0, ahora)
                vals = np.array([d[1][i] for d in tele] + [d[2][i] for d in tele], float)
                vals = vals[~np.isnan(vals)]
                if len(vals):
                    ax_p[i].set_ylim(vals.min() - 3, vals.max() + 3)
        orb = [d for d in list(enlace.orb) if d["t"] >= t0]
        for s in sombras:
            s.remove()
        sombras.clear()
        if orb:
            t = np.array([d["t"] for d in orb])
            lw_f.set_data(t, [d["wf"] for d in orb]); lw_o.set_data(t, [d["wobj"] for d in orb])
            dist = np.array([d["dist"] if d["dist"] > 0 else np.nan for d in orb])
            ld.set_data(t, dist); ldr.set_data(t, [d["dref"] for d in orb])
            lu.set_data(t, [d["u"] for d in orb]); lc.set_data(t, [d["corr"] for d in orb])
            lcm.set_data(t, [d["corr_mlp"] for d in orb]); ldp.set_data(t, [10 * d["du_per"] for d in orb])
            for a in (ax_w, ax_d, ax_u):
                a.set_xlim(t0, ahora)
            for a, ys in ((ax_w, [d["wf"] for d in orb] + [d["wobj"] for d in orb]),
                          (ax_d, [x for x in dist if not np.isnan(x)] + [d["dref"] for d in orb]),
                          (ax_u, [d["u"] for d in orb])):
                ys = [y for y in ys if np.isfinite(y)]
                if ys:
                    a.set_ylim(min(ys) - 2, max(ys) + 2)
            ys = [d["corr"] for d in orb] + [d["corr_mlp"] for d in orb] + [10 * d["du_per"] for d in orb]
            ax_u2.set_ylim(min(ys) - 1, max(ys) + 1)
            sin = np.array([not d["laser"] for d in orb])
            if sin.any():
                sombras.append(ax_d.fill_between(t, 0, 1, where=sin, color="0.85", transform=ax_d.get_xaxis_transform(), zorder=0))
            o = orb[-1]
            # redes en vivo
            if PESOS and sc_per is not None:
                act = ACT_NOMBRE.get(o["rnact"], "tanh")
                w = PESOS["perceptron"][act]; nm = PESOS["norm"]
                x1, x2 = o["ew"] / nm["EW_N"], o["dew"] / nm["DEW_N"]
                v = w[0] + w[1] * x1 + w[2] * x2
                sc_per.set_array(np.clip([x1, x2, np.tanh(v), o["du_per"] / nm["DU_MAX"]], -1, 1))
                juego = "control1" if o["juego"] else "control0"
                red = PESOS["mlp"][juego]
                xm = np.array([o["ed"] / nm["ED_N"], np.clip(o["ded"], -6, 6) / nm["DED_N"]])
                h = np.tanh(np.array(red["W1"]) @ xm + np.array(red["b1"]))
                out = np.tanh(h @ np.array(red["W2"]) + red["b2"])
                sc_mlp.set_array(np.clip(np.concatenate([xm, h, [out]]), -1, 1))
                en_uso_p = o["rn"] in (1, 3); en_uso_m = o["rn"] in (2, 3)
                tit_per.set_text(f"Perceptron ({act}) - {'EN USO' if en_uso_p else 'en sombra'}")
                tit_per.set_color("tab:blue" if en_uso_p else "0.45")
                tit_mlp.set_text(f"MLP 2-8-1 ({'control=1' if o['juego'] else 'control=0'}) - {'EN USO' if en_uso_m else 'en sombra'}")
                tit_mlp.set_color("tab:blue" if en_uso_m else "0.45")
        o = enlace.ultimo_orb or {}
        estado = [f"Conexion: {'OK' if enlace.sock else 'sin conexion'}    telemetria: {len(tele)} muestras en {VENTANA_S:.0f} s",
                  f"Controlador: rn={o.get('rn', '?')} {RN_NOMBRE.get(o.get('rn', -1), '')}   actuador: {'damp' if o.get('act', 0) == 0 else 'ddesf'}"
                  f"   redes: {o.get('us', 0):.0f} us",
                  f"Orbita: {'ACTIVA' if o.get('activa') else 'detenida'}   recorrido {o.get('rec', 0):.0f} de {o.get('phiobj', 0):.0f} grados"
                  f"   dist {o.get('dist', 0):.1f} cm (dref {o.get('dref', 0):.0f})",
                  "-" * 70] + list(enlace.mensajes)[-7:]
        if enlace.orbitas:
            r = enlace.orbitas[-1]
            estado.insert(3, f"Ultima orbita: {r['rn']}  RMS {r['err_rms']:.2f} cm  {r['mensaje'][:60]}")
        txt.set_text("\n".join(estado))
        return []

    anim = FuncAnimation(fig, actualizar, interval=150, cache_frame_data=False)
    fig._anim = anim
    plt.show()
    enlace.vivo = False


if __name__ == "__main__":
    main()
