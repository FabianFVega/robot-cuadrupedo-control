# -*- coding: utf-8 -*-
"""
PC <-> ESP8266 principal (PID de las 4 rodillas, version 2) por WiFi.

Que hace:
  - Se conecta al Access Point del robot (SSID RobotPID, IP 192.168.4.1:80).
  - Recibe la telemetria (17 campos por linea) y la grafica en vivo:
    referencia y theta de cada pata (FI, FD, TI, TD) y el write enviado.
  - Guarda todo en un CSV con marca de tiempo (datos_pid_AAAAMMDD_HHMMSS.csv).
  - Deja escribir comandos en la terminal y los manda a la ESP:
        ref=260        las 4 patas
        ref1=260       solo FI   (ref2 = FD, ref3 = TI, ref4 = TD)
        kp=0.8  ki=12  kd=0.008
        modo=pid  /  modo=abierto
        escalon        prueba de escalon automatica (ver abajo)
        calibrar       mueve una pata a la vez en lazo abierto y dice que sensor
                       mueve cada servo y la recta theta = a + b*write de cada pata
        calibrar aplicar   lo mismo y ademas manda la calibracion a la ESP (cal1..4=)
        linealizar     corrige por tabla la lectura deformada del AS5600 (iman fuera
                       del eje) y deja 270° = vertical en las 4 patas. Usar despues de 'cero'
        cero           con las 4 patas verticales, calcula OFFSET_AS5600 y SIGNO_AS5600
                       para que todas lean 270° en vertical (mismo sistema de angulos)
        barrido 2      barre el write de una pata (1=FI 2=FD 3=TI 4=TD) de 40 a 140 y
                       de vuelta, mostrando cuanto se mueve en cada paso
        salir
  - Prueba de escalon: lleva las patas a REF_BASE, espera, sube a
    REF_BASE + AMPLITUD, espera, baja otra vez, y calcula para cada pata
    el sobrepaso, el tiempo de asentamiento y el error final. Guarda la
    grafica del escalon en un PNG.

Uso:
  1. Conectar la PC a la red WiFi "RobotPID" (clave 12345678).
  2. pip install matplotlib numpy
  3. python pc_pid_4patas.py
     (opcional) python pc_pid_4patas.py --escalon   -> corre la prueba al iniciar
"""
import argparse
import csv
import json
import os
import socket
import sys
import threading
import time
from collections import deque
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.figure import Figure

# ------------------------------------------------------------------
# CONFIGURACION
# ------------------------------------------------------------------
IP_ESP = "192.168.4.1"
PUERTO = 80
VENTANA_S = 10.0            # segundos visibles en la grafica en vivo
PATAS = ["FI", "FD", "TI", "TD"]

# prueba de escalon
REF_BASE = 265.0            # grados
AMPLITUD = 10.0             # grados
ESPERA_S = 3.0              # tiempo en cada nivel
BANDA_ASENT = 0.5           # grados, banda para el tiempo de asentamiento

CAMPOS = ["t_ms"] + [f"{c}{i}" for i in range(1, 5)
                     for c in ("ref", "theta", "error", "write")]


# ------------------------------------------------------------------
# Conexion con la ESP
# ------------------------------------------------------------------
class EnlaceESP:
    def __init__(self, ip, puerto, archivo_csv):
        self.ip, self.puerto = ip, puerto
        self.sock = None
        self.activo = True
        self.lock = threading.Lock()
        n = int(VENTANA_S * 50 * 1.5)
        self.t = deque(maxlen=n)
        self.ref = [deque(maxlen=n) for _ in PATAS]
        self.theta = [deque(maxlen=n) for _ in PATAS]
        self.write = [deque(maxlen=n) for _ in PATAS]
        self.registro = []          # (t_s, fila) mientras corre una prueba
        self.grabando = False
        self.lineas = 0
        self.t0_ms = None
        self.ultimo_dato = 0.0      # hora de la PC de la ultima linea de telemetria
        self.cal = None             # [(theta_w0, grados_por_write)] por pata, tras 'calibrar aplicar'
        self.f_csv = open(archivo_csv, "w", newline="")
        self.csv = csv.writer(self.f_csv)
        self.csv.writerow(["t_pc_s"] + CAMPOS)

    def conectar(self):
        while self.activo:
            try:
                print(f"Conectando a {self.ip}:{self.puerto} ...")
                s = socket.create_connection((self.ip, self.puerto), timeout=5)
                s.settimeout(1.0)
                self.sock = s
                with self.lock:          # no mezclar datos viejos con los de la nueva conexion
                    self.t.clear()
                    for d in self.ref + self.theta + self.write:
                        d.clear()
                    self.t0_ms = None
                print("Conectado. Escribe comandos (ej: ref=260, kp=0.8, escalon, salir).")
                # la ESP pierde la calibracion al reiniciarse: se la volvemos a mandar
                time.sleep(0.3)
                if os.path.exists(ARCHIVO_CAL):
                    cargar_calibracion(self)
                return
            except OSError as e:
                print(f"  sin conexion ({e}). ¿La PC esta en la red RobotPID? Reintento en 2 s")
                time.sleep(2)

    def enviar(self, comando, registrar=True):
        comando = comando.strip()
        if not comando:
            return
        if "=" not in comando:                       # "ref 330" -> "ref=330"
            partes = comando.replace(":", " ").split()
            if len(partes) == 2:
                comando = f"{partes[0]}={partes[1]}"
        try:
            self.sock.sendall((comando + "\n").encode())
            print(f"  enviado: {comando}")
        except OSError as e:
            print(f"  no se pudo enviar '{comando}': {e}")
            return
        # recordar la calibracion (calibrar aplicar o cal1=a,b escrito a mano)
        clave, _, valor = comando.partition("=")
        if registrar and len(clave) == 4 and clave.startswith("cal") and clave[3] in "1234" and "," in valor:
            try:
                a, b = (float(x) for x in valor.split(",", 1))
            except ValueError:
                return
            if self.cal is None:
                self.cal = [(CAL_W0_DEFECTO, CAL_GPW_DEFECTO)] * 4
            self.cal = list(self.cal)
            self.cal[int(clave[3]) - 1] = (a, b)
            guardar_calibracion(self.cal)

    def vigilar(self):
        """Si la conexion esta abierta pero no llega telemetria, la cierra y reconecta."""
        while self.activo:
            time.sleep(1)
            if self.sock is None or self.ultimo_dato == 0.0:
                continue
            if time.time() - self.ultimo_dato > 4:
                print("\n[PC] No llega telemetria hace 4 s: reconectando "
                      "(si sigue igual, reiniciar la ESP principal con su boton).")
                self.ultimo_dato = time.time()
                try:
                    self.sock.close()     # leer() detecta el cierre y reconecta
                except OSError:
                    pass

    def _parsear(self, linea):
        partes = linea.split(",")
        if len(partes) != 17:
            return None
        vals = []
        for p in partes:
            p = p.strip()
            vals.append(float("nan") if p == "NA" else float(p))
        return vals

    def leer(self):
        buf = b""
        while self.activo:
            try:
                datos = self.sock.recv(4096)
                if not datos:
                    raise ConnectionError("la ESP cerro la conexion")
            except socket.timeout:
                continue
            except OSError as e:
                if not self.activo:
                    break
                print(f"\nConexion perdida ({e}). Reconectando...")
                self.conectar()
                buf = b""
                continue
            buf += datos
            while b"\n" in buf:
                linea, buf = buf.split(b"\n", 1)
                texto = linea.decode(errors="ignore").strip()
                if texto.startswith("#"):          # mensajes y diagnostico de la ESP
                    print(f"\n[ESP] {texto[1:].strip()}")
                    continue
                try:
                    vals = self._parsear(linea.decode(errors="ignore").strip())
                except ValueError:
                    vals = None
                if vals is None:
                    continue
                t_pc = time.time()
                self.ultimo_dato = t_pc
                self.csv.writerow([f"{t_pc:.3f}"] + linea.decode(errors="ignore").strip().split(","))
                self.lineas += 1
                if self.lineas % 50 == 0:
                    self.f_csv.flush()
                if self.t0_ms is None:
                    self.t0_ms = vals[0]
                t_s = (vals[0] - self.t0_ms) / 1000.0
                with self.lock:
                    self.t.append(t_s)
                    for i in range(4):
                        self.ref[i].append(vals[1 + 4 * i])
                        self.theta[i].append(vals[2 + 4 * i])
                        self.write[i].append(vals[4 + 4 * i])
                    if self.grabando:
                        self.registro.append((t_s, vals))

    def cerrar(self):
        self.activo = False
        try:
            self.sock.close()
        except Exception:
            pass
        self.f_csv.close()


# ------------------------------------------------------------------
# Prueba de escalon
# ------------------------------------------------------------------
def metricas(t, y, r_ini, r_fin, t_cambio):
    """Sobrepaso (%), tiempo de asentamiento (s) y error final (grados)."""
    m = (t >= t_cambio) & ~np.isnan(y)
    t, y = t[m] - t_cambio, y[m]
    if len(y) < 5:
        return None
    salto = r_fin - r_ini
    if salto > 0:
        sp = max(0.0, (y.max() - r_fin) / salto * 100)
    else:
        sp = max(0.0, (r_fin - y.min()) / -salto * 100)
    fuera = np.where(np.abs(y - r_fin) > BANDA_ASENT)[0]
    if len(fuera) == 0:
        ts = 0.0
    elif fuera[-1] == len(y) - 1:
        ts = float("nan")             # nunca entro a la banda
    else:
        ts = t[fuera[-1] + 1]
    ultimos = y[t > t[-1] - 0.5]
    ess = r_fin - np.mean(ultimos)
    return sp, ts, ess


ARCHIVO_CAL = "calibracion_patas.json"


def guardar_calibracion(cal, tablas=None):
    """Guarda la calibracion (y las tablas de linealizacion, si hay)."""
    if tablas is None:                       # conservar las tablas ya guardadas
        try:
            with open(ARCHIVO_CAL) as f:
                tablas = json.load(f).get("tablas")
        except (OSError, ValueError):
            tablas = None
    datos = {"fecha": f"{datetime.now():%Y-%m-%d %H:%M}",
             "patas": [{"pata": PATAS[p], "theta_w0": round(a, 3),
                        "grados_por_write": round(b, 5)}
                       for p, (a, b) in enumerate(cal)]}
    if tablas:
        datos["tablas"] = tablas
    with open(ARCHIVO_CAL, "w") as f:
        json.dump(datos, f, indent=2)


def cargar_calibracion(enlace):
    """Lee la calibracion guardada por 'calibrar aplicar' y se la manda a la ESP."""
    try:
        with open(ARCHIVO_CAL) as f:
            cal = json.load(f)
    except (OSError, ValueError):
        return False
    enlace.cal = [(c["theta_w0"], c["grados_por_write"]) for c in cal["patas"]]
    tablas = cal.get("tablas") or [None] * 4
    for p, (a, b) in enumerate(enlace.cal):
        enlace.enviar(f"cal{p + 1}={a:.2f},{b:.4f}", registrar=False)
        if tablas[p]:
            enlace.enviar(f"tab{p + 1}=" + ",".join(f"{x:.2f}" for x in tablas[p]), registrar=False)
    print(f"  Calibracion cargada de {ARCHIVO_CAL} ({cal.get('fecha', '')}) y enviada a la ESP.")
    return True


def prueba_escalon(enlace):
    """Escalon de AMPLITUD grados en cada pata, centrado en la mitad de SU rango.
    Cada AS5600 tiene su propio cero, asi que la referencia es distinta por pata."""
    motivo = problema_datos(enlace)
    if motivo:
        print(f"\n  {motivo}\n  Escalon cancelado.")
        return
    if enlace.cal is None and not cargar_calibracion(enlace):
        print("\n  Primero corre 'calibrar aplicar': sin la calibracion de cada pata la ESP")
        print("  no sabe convertir la referencia a write() y cada sensor tiene un cero distinto.")
        return
    base, alto = [], []
    for a, b in enlace.cal:
        centro = a + b * CAL_W_REPOSO                  # mitad del rango de write (40..140)
        base.append(round(centro - AMPLITUD / 2, 2))
        alto.append(round(centro + AMPLITUD / 2, 2))
    print(f"\n=== Prueba de escalon de {AMPLITUD}° en cada pata (modo pid) ===")
    for p in range(4):
        print(f"   {PATAS[p]}: {base[p]:.2f}° -> {alto[p]:.2f}° -> {base[p]:.2f}°")

    def mandar(refs):
        for p in range(4):
            enlace.enviar(f"ref{p + 1}={refs[p]:.2f}")

    enlace.enviar("modo=pid")
    mandar(base)
    time.sleep(ESPERA_S)
    with enlace.lock:
        enlace.registro = []
        enlace.grabando = True
    time.sleep(0.5)
    mandar(alto)
    time.sleep(ESPERA_S)
    mandar(base)
    time.sleep(ESPERA_S)
    with enlace.lock:
        enlace.grabando = False
        reg = list(enlace.registro)
    if len(reg) < 20:
        print("  No llegaron datos suficientes. ¿La ESP-Sensores esta enviando theta?")
        return

    t = np.array([r[0] for r in reg])
    vals = np.array([r[1] for r in reg])
    ref_all = vals[:, 1::4]
    # instante del cambio = primera muestra en que la ESP reporta la nueva referencia
    def instante(col, valor, desde=None):
        ok = np.abs(ref_all[:, col] - valor) < 1e-3
        if desde is not None:
            ok &= t > desde
        idx = np.where(ok)[0]
        return t[idx[0]] if len(idx) else None

    fig = Figure(figsize=(11, 7))                 # sin pyplot: se puede crear fuera del hilo principal
    ax = fig.subplots(2, 2, sharex=True)
    print(f"{'Pata':5s} | {'subida: SP %':>12s} {'t_s s':>7s} {'e_ss °':>7s} | "
          f"{'bajada: SP %':>12s} {'t_s s':>7s} {'e_ss °':>7s}")
    for i, nombre in enumerate(PATAS):
        y = vals[:, 2 + 4 * i]
        r = vals[:, 1 + 4 * i]
        t1 = instante(i, alto[i])
        t2 = instante(i, base[i], desde=t1) if t1 is not None else None
        filas = []
        if t1 is not None and t2 is not None:
            m1 = (t >= t1) & (t < t2)
            filas.append(metricas(t[m1], y[m1], base[i], alto[i], t1))
            filas.append(metricas(t[t >= t2], y[t >= t2], alto[i], base[i], t2))
        txt = []
        for f in filas:
            txt.append("   sin datos           " if f is None else f"{f[0]:12.1f} {f[1]:7.2f} {f[2]:+7.2f}")
        print(f"{nombre:5s} | " + " | ".join(txt))
        a = ax.flat[i]
        a.plot(t - t[0], r, "--", color="0.5", lw=1, label="referencia")
        a.plot(t - t[0], y, color="#1a4f8b", lw=1.8, label="θ medida (AS5600)")
        aw = a.twinx()                                  # write() enviado, eje derecho
        aw.plot(t - t[0], vals[:, 4 + 4 * i], color="#e07b39", lw=1, alpha=0.7)
        aw.set_ylim(30, 150)
        aw.set_ylabel("write()", color="#e07b39", fontsize=8)
        aw.tick_params(axis="y", labelcolor="#e07b39", labelsize=8)
        a.set_title(nombre)
        a.set_ylabel("θ (°)")
        a.grid(alpha=0.3)
        a.spines[["top", "right"]].set_visible(False)
    ax[0, 0].legend(fontsize=8, frameon=False)
    for a in ax[1]:
        a.set_xlabel("Tiempo (s)")
    fig.suptitle(f"Escalón de {AMPLITUD}° en cada pata (PID, Kp/Ki/Kd actuales)")
    fig.tight_layout()
    nombre_png = f"escalon_{datetime.now():%Y%m%d_%H%M%S}.png"
    fig.savefig(nombre_png, dpi=150)
    print(f"  (t_s = nan: no entro en la banda de ±{BANDA_ASENT}°)  Grafica: {nombre_png}\n")


# ------------------------------------------------------------------
# Calibracion automatica: que sensor mueve cada servo y con que recta
# ------------------------------------------------------------------
CAL_W0_DEFECTO, CAL_GPW_DEFECTO = 220.0, 0.6111   # calibracion que trae el .ino por defecto
CAL_WRITES = [50, 70, 90, 110, 130]               # writes que se prueban en cada pata
CAL_W_REPOSO = 90
CANAL_AS5600_ACTUAL = [0, 7, 1, 2]                # el de esp_sensores_pata_v2.ino (FI, FD, TI, TD)


def _theta_promedio(enlace, ventana=0.4):
    with enlace.lock:
        t = np.array(enlace.t)
        th = [np.array(d) for d in enlace.theta]
    if len(t) == 0:
        return [float("nan")] * 4
    m = t >= t[-1] - ventana
    return [float(np.nanmean(x[m])) if np.any(~np.isnan(x[m])) else float("nan") for x in th]


def problema_datos(enlace):
    """None si llegan datos frescos con las 4 theta; si no, el motivo."""
    if time.time() - enlace.ultimo_dato > 1.5:
        return ("No llega telemetria de la ESP principal. Revisar que la PC siga en la red "
                "RobotPID; si no se recupera, reiniciar la ESP principal y este programa.")
    th = _theta_promedio(enlace, ventana=0.5)
    faltan = [PATAS[p] for p in range(4) if np.isnan(th[p])]
    if faltan:
        return (f"Llega telemetria pero sin angulo de {', '.join(faltan)} (theta = NA): revisar que la "
                "ESP-Sensores tenga esp_sensores_pata_v2.ino, este encendida y conectada (TX->RX, GND).")
    return None


def _ref_para_write(w):
    # en modo abierto la ESP convierte ref -> write con la calibracion por defecto
    return CAL_W0_DEFECTO + CAL_GPW_DEFECTO * w


def calibrar(enlace, aplicar=False):
    """Mueve una pata a la vez en lazo abierto y ajusta theta = a + b*write."""
    print("\n=== Calibracion: se mueve UNA pata a la vez (modo abierto) ===")
    print("    Deja el robot en el soporte, con las patas libres.")
    motivo = problema_datos(enlace)
    if motivo:
        print(f"  {motivo}\n  Calibracion cancelada.")
        return
    for p in range(4):     # calibracion por defecto solo en la ESP (no borra la guardada)
        enlace.enviar(f"tab{p + 1}=off", registrar=False)
        enlace.enviar(f"cal{p + 1}={CAL_W0_DEFECTO},{CAL_GPW_DEFECTO}", registrar=False)
    enlace.enviar("modo=abierto")
    enlace.enviar(f"ref={_ref_para_write(CAL_W_REPOSO):.3f}")
    time.sleep(2.0)

    # subida y bajada: si a igual write la pata no vuelve al mismo angulo hay
    # juego mecanico, un tope o un iman que patina
    secuencia = CAL_WRITES + CAL_WRITES[-2::-1]
    resultados = []
    for p, nombre in enumerate(PATAS):
        print(f"  moviendo {nombre} ...")
        filas = []
        for w in secuencia:
            enlace.enviar(f"ref{p + 1}={_ref_para_write(w):.3f}")
            time.sleep(1.3)
            filas.append((w, _theta_promedio(enlace, ventana=0.5)))
        enlace.enviar(f"ref{p + 1}={_ref_para_write(CAL_W_REPOSO):.3f}")
        time.sleep(1.0)

        ws = np.array([f[0] for f in filas], dtype=float)
        th = np.array([f[1] for f in filas], dtype=float)          # (9, 4)
        if np.isnan(th[:, p]).sum() > 2:
            print(f"  Se perdieron los datos de {nombre} durante la prueba (theta = NA). "
                  f"Calibracion cancelada: revisar el enlace con la ESP-Sensores y repetir.")
            enlace.enviar(f"ref{p + 1}={_ref_para_write(CAL_W_REPOSO):.3f}")
            return
        th = np.rad2deg(np.unwrap(np.deg2rad(th), axis=0))          # por si cruza 0/360
        recorrido = np.nanmax(th, axis=0) - np.nanmin(th, axis=0)
        propio = th[:, p]
        # histeresis: diferencia subida-bajada en los writes intermedios
        n = len(CAL_WRITES)
        hist = float(np.nanmax(np.abs(propio[1:n - 1] - propio[::-1][1:n - 1]))) if n > 2 else 0.0
        if recorrido[p] < 5:
            otro = int(np.nanargmax(recorrido))
            resultados.append(dict(p=p, ok=False, cruce=otro if recorrido[otro] >= 5 else None,
                                   rec=recorrido))
            continue
        b, a = np.polyfit(ws, propio, 1)
        pred = a + b * ws
        r2 = 1 - np.sum((propio - pred) ** 2) / np.sum((propio - propio.mean()) ** 2)
        resultados.append(dict(p=p, ok=True, a=a, b=b, r2=r2, hist=hist, rec=recorrido))

    # ---- informe ----
    print(f"\n{'Pata':5s} {'recorrido':>9s} {'theta_w0':>9s} {'grados/write':>12s} {'R²':>6s} "
          f"{'histeresis':>10s}   otros sensores")
    for r in resultados:
        p, rec = r["p"], r["rec"]
        otros = ", ".join(f"{PATAS[j]} {rec[j]:.1f}°" for j in range(4) if j != p)
        if not r["ok"]:
            if r["cruce"] is not None:
                print(f"{PATAS[p]:5s} su sensor no se mueve; el que se mueve es theta{r['cruce'] + 1} "
                      f"({PATAS[r['cruce']]}) -> cables o canales cruzados")
            else:
                print(f"{PATAS[p]:5s} ningun sensor se movio mas de 5° -> servo sin alimentacion, "
                      f"pin equivocado, pata trabada o iman fuera del chip")
            continue
        print(f"{PATAS[p]:5s} {rec[p]:8.1f}° {r['a']:9.2f} {r['b']:12.4f} {r['r2']:6.3f} "
              f"{r['hist']:9.1f}°   {otros}")
        if abs(r["b"]) < 0.4:
            print(f"      ^ solo {abs(r['b']) * 100:.0f}° por 100 unidades de write (lo normal son ~60°): "
                  f"la pata choca con algo o el iman no gira con ella")
        if r["r2"] < 0.98 or r["hist"] > 3:
            print(f"      ^ poco lineal o con histeresis: tope, juego mecanico o iman que patina")
        if max(rec[j] for j in range(4) if j != p) > 3:
            print(f"      ^ otros sensores se movieron mas de 3° mientras solo se movia esta pata: "
                  f"el robot se movio, alguien lo toco o cae el voltaje. Repetir.")

    cruzado = any((not r["ok"]) and r["cruce"] is not None for r in resultados)
    mapa = [r["p"] if r["ok"] else r["cruce"] for r in resultados]
    if cruzado:
        print("\n!! Hay sensores cruzados: el servo de una pata mueve el sensor de otra.")
        if None not in mapa and sorted(mapa) == [0, 1, 2, 3]:
            nuevo = [CANAL_AS5600_ACTUAL[s] for s in mapa]
            print("   Corregir en esp_sensores_pata_v2.ino:")
            print(f"   const uint8_t CANAL_AS5600[NUM_PATAS] = {{{', '.join(map(str, nuevo))}}};")
            print("   y volver a correr 'calibrar' despues de cargarlo.")
        else:
            print("   No es una permutacion limpia (dos servos mueven el mismo sensor o alguno no mueve ninguno):")
            print("   revisar cableado de servos y sensores antes de seguir.")
        return

    if None in mapa:
        print("\nHay patas sin respuesta: revisarlas antes de cerrar el lazo.")
        return

    print("\nCalibracion para copiar en esp_principal_pid_4patas_v2.ino:")
    print("float THETA_W0[NUM_PATAS]         = {" +
          ", ".join(f"{r['a']:.2f}" for r in resultados) + "};")
    print("float GRADOS_POR_WRITE[NUM_PATAS] = {" +
          ", ".join(f"{r['b']:.4f}" for r in resultados) + "};")
    print("\nRango alcanzable por pata (write 40 a 140):")
    for r in resultados:
        lo, hi = sorted((r["a"] + r["b"] * 40, r["a"] + r["b"] * 140))
        print(f"   {PATAS[r['p']]}: {lo:6.1f}° a {hi:6.1f}°")
    if aplicar:
        guardar_calibracion([(r["a"], r["b"]) for r in resultados], tablas=[None] * 4)
        for r in resultados:
            enlace.enviar(f"cal{r['p'] + 1}={r['a']:.2f},{r['b']:.4f}")
        print("\nCalibracion aplicada en vivo. Sigue en modo abierto: prueba refs dentro del rango")
        print("de cada pata (ref1=, ref2=...) y, si cada una va donde debe, manda modo=pid.")
        print(f"Guardada en {ARCHIVO_CAL}: 'escalon' la vuelve a mandar si la ESP se reinicia.")
    else:
        print("\nPara aplicarla sin recompilar: comando 'calibrar aplicar'.")


def barrido(enlace, p):
    """Barre el write de UNA pata de 40 a 140 y de vuelta, de 10 en 10, e imprime
    cuanto cambia el angulo en cada paso. Sirve para ver en que parte del recorrido
    la pata se frena (tope, roce) o el sensor deja de seguirla (iman)."""
    nombre = PATAS[p]
    motivo = problema_datos(enlace)
    if motivo:
        print(f"\n  {motivo}\n  Barrido cancelado.")
        return
    print(f"\n=== Barrido de {nombre}: write 40 -> 140 -> 40 (modo abierto) ===")
    print("    Mira la pata mientras corre: si se detiene fisicamente, es un tope o un roce.")
    enlace.enviar(f"cal{p + 1}={CAL_W0_DEFECTO},{CAL_GPW_DEFECTO}", registrar=False)
    enlace.enviar("modo=abierto")
    ws = list(range(40, 141, 10)) + list(range(130, 39, -10))
    datos = []
    for w in ws:
        enlace.enviar(f"ref{p + 1}={_ref_para_write(w):.3f}")
        time.sleep(1.3)
        datos.append((w, _theta_promedio(enlace, ventana=0.5)[p]))
    enlace.enviar(f"ref{p + 1}={_ref_para_write(CAL_W_REPOSO):.3f}")

    print(f"\n{'write':>6s} {'theta':>8s} {'cambio':>8s}")
    anterior = None
    for i, (w, th) in enumerate(datos):
        if i == len(ws) // 2 + 1:
            print("   --- bajando ---")
        d = "" if anterior is None else f"{th - anterior:+8.2f}"
        marca = ""
        if anterior is not None and abs(th - anterior) < 1.5:
            marca = "  <- casi no se movio en este paso"
        print(f"{w:6d} {th:8.2f} {d:>8s}{marca}")
        anterior = th
    n = len(ws) // 2 + 1
    sube = {w: th for w, th in datos[:n]}
    baja = {w: th for w, th in datos[n - 1:]}
    hist = [abs(sube[w] - baja[w]) for w in sube if w in baja and w not in (40, 140)]
    rec = max(th for _, th in datos) - min(th for _, th in datos)
    print(f"\nRecorrido total: {rec:.1f}° (lo normal son ~60°).  "
          f"Diferencia subida/bajada maxima: {max(hist):.1f}°")
    print("Pasos de ~6° en todo el barrido = pata sana. Pasos que se achican en un extremo = tope o roce.")
    print("Pasos pequenos parejos en todo el barrido, con la pata girando completa a la vista = iman/sensor.")
    if enlace.cal is not None:
        a, b = enlace.cal[p]
        enlace.enviar(f"cal{p + 1}={a:.2f},{b:.4f}", registrar=False)
        print(f"(Se restauro la calibracion de {nombre}. Sigue en modo abierto.)")


# ------------------------------------------------------------------
# Cero comun: 270° = pata vertical en las 4 patas
# ------------------------------------------------------------------
# Las patas de cada lado son espejo: el mismo aumento de write mueve la pata
# izquierda en un sentido y la derecha en el contrario. Con estos signos, el
# angulo crece en el mismo sentido fisico en las 4 patas (el sentido en que se
# mueven las izquierdas cuando sube el write). Si ese sentido resulta ser
# "pie hacia atras", invertir los 4 signos.
SIGNO_POR_PATA = [1, -1, 1, -1]      # FI, FD, TI, TD


def cero(enlace):
    """Con las 4 patas verticales, calcula OFFSET_AS5600 para que todas lean 270°."""
    print("\n=== Cero comun (270° = pata vertical) ===")
    print("  Requisitos: la ESP-Sensores con OFFSET_AS5600 = {0,0,0,0} y SIGNO_AS5600 = {1,1,1,1}")
    print("  (angulo crudo) y las 4 patas VERTICALES hacia abajo. Para ponerlas verticales:")
    print("  modo=abierto y ajustar ref1..ref4 mirando la pata, o apagar los servos y ponerlas a mano.")
    time.sleep(1.0)
    motivo = problema_datos(enlace)
    if motivo:
        print(f"  {motivo}\n  Cero cancelado.")
        return
    crudo = _theta_promedio(enlace, ventana=1.0)
    if any(np.isnan(crudo)):
        print("  Falta la lectura de alguna pata (NA). Revisar sensores y repetir.")
        return
    offsets = []
    for p in range(4):
        s = SIGNO_POR_PATA[p]
        # theta = s * (crudo - OFFSET), llevado a [0, 360)  ->  queremos 270
        off = (crudo[p] - 270.0) if s == 1 else (crudo[p] + 270.0)
        offsets.append(off % 360.0)
    print(f"\n{'Pata':5s} {'crudo':>8s} {'signo':>6s} {'OFFSET':>8s}")
    for p in range(4):
        print(f"{PATAS[p]:5s} {crudo[p]:8.2f} {SIGNO_POR_PATA[p]:6d} {offsets[p]:8.2f}")
    print("\nCopiar en esp_sensores_pata_v2.ino y cargarlo:")
    print("float OFFSET_AS5600[NUM_PATAS] = {" + ", ".join(f"{o:.2f}" for o in offsets) + "};")
    print("int   SIGNO_AS5600[NUM_PATAS]  = {" + ", ".join(str(s) for s in SIGNO_POR_PATA) + "};")
    print("\nDespues: las 4 patas leeran ~270° en vertical. La calibracion anterior ya no sirve:")
    print("correr 'calibrar aplicar' de nuevo (en las patas derechas saldra grados/write negativo,")
    print("es correcto). Luego mirar: al subir ref en cualquier pata, ¿el pie va hacia adelante?")
    print("Si va hacia atras en las 4, cambiar SIGNO_POR_PATA (arriba en este archivo) a")
    print("[-1, 1, -1, 1]; si solo en algunas, cambiar el signo de esas. Luego repetir 'cero'.")


# ------------------------------------------------------------------
# Linealizacion por tabla (montaje con el iman fuera del eje de giro)
# ------------------------------------------------------------------
# Grados que gira realmente la pata por unidad de write. Supuesto: ~0.6.
# Para medirlo: modo=abierto, mandar write 40 y 140 (ref = 220 + 0.6111*w con la
# calibracion por defecto) y medir con transportador el angulo entre ambas
# posiciones; K_REAL = angulo / 100.
K_REAL = 0.6
TABLA_WRITES = list(range(40, 141, 10))      # igual que en la ESP: 11 puntos


def linealizar(enlace):
    """Barre cada pata (write 40..140 subiendo y bajando), arma la tabla
    lectura -> write y se la manda a la ESP junto con una calibracion lineal
    que deja 270° en la pata vertical (segun el 'cero' de la ESP-Sensores)."""
    motivo = problema_datos(enlace)
    if motivo:
        print(f"\n  {motivo}\n  Linealizacion cancelada.")
        return
    print("\n=== Linealizacion por tabla: cada pata, write 40 -> 140 -> 40 (modo abierto) ===")
    print("    Robot en el soporte, patas libres. Tarda ~2 minutos.")
    for p in range(4):
        enlace.enviar(f"tab{p + 1}=off", registrar=False)
        enlace.enviar(f"cal{p + 1}={CAL_W0_DEFECTO},{CAL_GPW_DEFECTO}", registrar=False)
    enlace.enviar("modo=abierto")
    enlace.enviar(f"ref={_ref_para_write(CAL_W_REPOSO):.3f}")
    time.sleep(2.0)

    secuencia = TABLA_WRITES + TABLA_WRITES[-2::-1]
    tablas, cal, informe = [], [], []
    for p, nombre in enumerate(PATAS):
        print(f"  midiendo {nombre} ...")
        lect = {w: [] for w in TABLA_WRITES}
        for w in secuencia:
            enlace.enviar(f"ref{p + 1}={_ref_para_write(w):.3f}", registrar=False)
            time.sleep(1.2)
            lect[w].append(_theta_promedio(enlace, ventana=0.5)[p])
        enlace.enviar(f"ref{p + 1}={_ref_para_write(CAL_W_REPOSO):.3f}", registrar=False)
        time.sleep(0.8)
        if any(np.isnan(v).any() for v in map(np.array, lect.values())):
            print(f"  Se perdieron datos de {nombre}. Linealizacion cancelada.")
            return
        # promedio subida/bajada (quita la histeresis), desenrollado 360->0
        sube = np.array([lect[w][0] for w in TABLA_WRITES])
        baja = np.array([lect[w][-1] for w in TABLA_WRITES])
        sube = np.rad2deg(np.unwrap(np.deg2rad(sube)))
        baja = sube + ((baja - sube + 180) % 360 - 180)
        t = (sube + baja) / 2
        hist = float(np.max(np.abs(sube - baja)))
        d = np.diff(t)
        monotona = np.all(d > 0.2) or np.all(d < -0.2)
        recorrido = abs(t[-1] - t[0])
        signo = 1.0 if t[-1] > t[0] else -1.0
        # write en que la pata esta vertical (lectura 270, segun el cero)
        x, y = (t, np.array(TABLA_WRITES, float)) if signo > 0 else (t[::-1], np.array(TABLA_WRITES[::-1], float))
        objetivo = 270.0 + 360.0 * round((t.mean() - 270.0) / 360.0)
        w_vert = float(np.interp(objetivo, x, y, left=np.nan, right=np.nan))
        gpw = signo * K_REAL
        if np.isnan(w_vert):
            w_vert = 90.0
            aviso_vert = "  (270° fuera del recorrido: se tomo write 90 como referencia)"
        else:
            aviso_vert = ""
        a = 270.0 - gpw * w_vert
        tablas.append([round(float(v), 2) for v in t])
        cal.append((a, gpw))
        informe.append((nombre, recorrido, hist, monotona, w_vert, a, gpw, aviso_vert, d))

    print(f"\n{'Pata':5s} {'recorrido sensor':>16s} {'histeresis':>10s} {'monotona':>9s} "
          f"{'write vertical':>14s}   rango lineal (write 40..140)")
    for i, (nombre, rec, hist, mono, wv, a, gpw, aviso, d) in enumerate(informe):
        lo, hi = sorted((a + gpw * 40, a + gpw * 140))
        print(f"{nombre:5s} {rec:15.1f}° {hist:9.1f}° {'si' if mono else 'NO':>9s} {wv:14.1f}   "
              f"{lo:6.1f}° a {hi:6.1f}°{aviso}")
        if not mono:
            print(f"      ^ la lectura no avanza siempre en el mismo sentido (pasos: "
                  f"{', '.join(f'{x:+.1f}' for x in d)}): esta pata NO se linealiza")
        elif rec < 15:
            print(f"      ^ el sensor ve solo {rec:.0f}°: la tabla funciona, pero la medicion sera ruidosa")
    ok = [mono for (_, _, _, mono, *_) in informe]

    for p in range(4):
        if ok[p]:
            enlace.enviar(f"tab{p + 1}=" + ",".join(f"{x:.2f}" for x in tablas[p]), registrar=False)
            enlace.enviar(f"cal{p + 1}={cal[p][0]:.2f},{cal[p][1]:.4f}", registrar=False)
    enlace.cal = [cal[p] if ok[p] else (enlace.cal[p] if enlace.cal else (CAL_W0_DEFECTO, CAL_GPW_DEFECTO))
                  for p in range(4)]
    guardar_calibracion(enlace.cal, tablas=[tablas[p] if ok[p] else None for p in range(4)])
    print(f"\nTablas y calibracion enviadas a la ESP y guardadas en {ARCHIVO_CAL}")
    print("(se reenvian solas al reconectar). Ahora en todas las patas: 270° = vertical y")
    print(f"{K_REAL}° por unidad de write. Prueba ref=270 (deben quedar verticales) y luego 'escalon'.")


# ------------------------------------------------------------------
# Entrada de comandos por terminal
# ------------------------------------------------------------------
def hilo_comandos(enlace):
    while enlace.activo:
        try:
            cmd = input("> ").strip()
        except EOFError:
            break
        if cmd == "salir":
            enlace.activo = False
            plt.close("all")
            break
        elif cmd == "escalon":
            threading.Thread(target=prueba_escalon, args=(enlace,), daemon=True).start()
        elif cmd == "linealizar":
            threading.Thread(target=linealizar, args=(enlace,), daemon=True).start()
        elif cmd == "cero":
            threading.Thread(target=cero, args=(enlace,), daemon=True).start()
        elif cmd.startswith("barrido"):
            partes = cmd.split()
            if len(partes) == 2 and partes[1] in "1234" and len(partes[1]) == 1:
                threading.Thread(target=barrido, args=(enlace, int(partes[1]) - 1),
                                 daemon=True).start()
            else:
                print("  uso: barrido 1   (1=FI, 2=FD, 3=TI, 4=TD)")
        elif cmd in ("calibrar", "calibrar aplicar"):
            threading.Thread(target=calibrar, args=(enlace, cmd.endswith("aplicar")),
                             daemon=True).start()
        elif cmd:
            enlace.enviar(cmd)


# ------------------------------------------------------------------
# Grafica en vivo
# ------------------------------------------------------------------
def grafica_en_vivo(enlace):
    fig = plt.figure(figsize=(11, 8.5))
    gs = fig.add_gridspec(3, 2, height_ratios=[2, 2, 1.4], hspace=0.45)
    ejes = [fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1]),
            fig.add_subplot(gs[1, 0]), fig.add_subplot(gs[1, 1])]
    lin_ref, lin_th = [], []
    for a, nombre in zip(ejes, PATAS):
        lr, = a.plot([], [], "--", color="0.5", lw=1, label="referencia")
        lt, = a.plot([], [], color="#1a4f8b", lw=1.6, label="θ medida")
        lin_ref.append(lr); lin_th.append(lt)
        a.set_title(nombre)
        a.set_ylabel("θ (°)")
        a.grid(alpha=0.3)
        a.spines[["top", "right"]].set_visible(False)
    ejes[0].legend(fontsize=8, frameon=False, loc="upper left")
    aw = fig.add_subplot(gs[2, :])
    colores = ["#1a4f8b", "#e07b39", "#2e8b57", "#8e44ad"]
    lin_w = [aw.plot([], [], color=c, lw=1.3, label=n)[0] for c, n in zip(colores, PATAS)]
    aw.set_ylabel("write()")
    aw.set_xlabel("Tiempo (s)")
    aw.legend(fontsize=8, frameon=False, ncol=4, loc="upper left")
    aw.grid(alpha=0.3)
    aw.spines[["top", "right"]].set_visible(False)
    fig.suptitle("PID de las rodillas — telemetría en vivo")

    def actualizar(_):
        with enlace.lock:
            t = np.array(enlace.t)
            refs = [np.array(d) for d in enlace.ref]
            ths = [np.array(d) for d in enlace.theta]
            ws = [np.array(d) for d in enlace.write]
        if len(t) < 2:
            return
        t_max = t[-1]
        m = t >= t_max - VENTANA_S
        for i, a in enumerate(ejes):
            lin_ref[i].set_data(t[m], refs[i][m])
            lin_th[i].set_data(t[m], ths[i][m])
            a.set_xlim(t_max - VENTANA_S, t_max + 0.1)
            datos = np.concatenate([refs[i][m], ths[i][m]])
            datos = datos[~np.isnan(datos)]
            if len(datos):
                a.set_ylim(datos.min() - 3, datos.max() + 3)
        for i in range(4):
            lin_w[i].set_data(t[m], ws[i][m])
        aw.set_xlim(t_max - VENTANA_S, t_max + 0.1)
        wall = np.concatenate([w[m] for w in ws])
        aw.set_ylim(np.nanmin(wall) - 5, np.nanmax(wall) + 5)

    anim = FuncAnimation(fig, actualizar, interval=100, cache_frame_data=False)
    plt.show()
    return anim


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default=IP_ESP)
    ap.add_argument("--puerto", type=int, default=PUERTO)
    ap.add_argument("--escalon", action="store_true", help="corre la prueba de escalon al iniciar")
    ap.add_argument("--sin-grafica", action="store_true", help="solo terminal y CSV")
    args = ap.parse_args()

    archivo = f"datos_pid_{datetime.now():%Y%m%d_%H%M%S}.csv"
    enlace = EnlaceESP(args.ip, args.puerto, archivo)
    enlace.conectar()
    print(f"Guardando datos en {archivo}")

    threading.Thread(target=enlace.leer, daemon=True).start()
    threading.Thread(target=enlace.vigilar, daemon=True).start()
    threading.Thread(target=hilo_comandos, args=(enlace,), daemon=True).start()
    if args.escalon:
        threading.Thread(target=prueba_escalon, args=(enlace,), daemon=True).start()

    try:
        if args.sin_grafica:
            while enlace.activo:
                time.sleep(0.5)
        else:
            grafica_en_vivo(enlace)
    except KeyboardInterrupt:
        pass
    finally:
        enlace.cerrar()
        print(f"\nListo. {enlace.lineas} lineas guardadas en {archivo}")


if __name__ == "__main__":
    main()
