# -*- coding: utf-8 -*-
"""
PID de la pata (rodilla) del robot araña — diseño, discretización y simulación.

Planta: servo MG90S + pata (sección 7 del documento de modelado)
    M θ'' + B_servo θ' + g S cos θ = K_servo (θ_ref − θ)
Lineal alrededor de θe = 275°:
    G_sp(s) = K_servo / (M s² + B_servo s + (K_servo − K))

Lazo: el AS5600 mide θ, el PID calcula θ_ref y el ESP8266 lo manda al servo
con write(). Estructura PI-D: P e I sobre el error, D sobre la medición
(un cambio brusco de referencia no produce un pico de derivada).

Por qué los gains no salen de una asignación de polos continua:
el servo solo recibe una orden nueva cada pulso PWM (Ts = 20 ms) y la
aplica en el pulso siguiente. Ese retardo (~30 ms en total con el ZOH)
es comparable con la constante de tiempo de la planta (33 ms), así que
un diseño continuo que lo ignora queda inestable u oscilatorio. Las
ganancias se ajustan sobre el modelo discreto CON retardo, buscando el
menor tiempo de asentamiento con sobrepaso ≤ 2 % (función buscar_ganancias).

Cuando tengan K_servo y B_servo identificados con el AS5600, cambien los
valores de PARÁMETROS, pongan BUSCAR = True y vuelvan a correr.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import signal

# ------------------------------------------------------------------
# PARÁMETROS DEL MODELO (estimados; reemplazar por los identificados)
# ------------------------------------------------------------------
M = 3.917e-6          # kg·m²   inercia efectiva Tibia + servo
S = 2.311e-4          # kg·m    primer momento de masa
g = 9.81
K_servo = 3.525e-3    # N·m/rad (datasheet, ζ = 1, t60 = 0.1 s)
B_servo = 2.350e-4    # N·m·s/rad
theta_e = np.deg2rad(275.0)
K = g * S * np.sin(theta_e)           # −2.258e-3 N·m/rad

# Hardware
Ts = 0.02             # s, periodo de control = periodo PWM del servo (50 Hz)
DELAY_SAMPLES = 1     # la orden calculada se aplica en el pulso PWM siguiente
TH_MIN, TH_MAX = np.deg2rad(220.0), np.deg2rad(330.0)   # rango del servo
WRITE_STEP = np.deg2rad(0.6111)       # 1 unidad de write() = 0.6111°
AS5600_STEP = np.deg2rad(360.0 / 4096)  # resolución 12 bits

# ------------------------------------------------------------------
# GANANCIAS DEL PID (resultado de buscar_ganancias con el modelo actual)
# ------------------------------------------------------------------
Kp = 0.75             # rad/rad
Ki = 19.0             # 1/s
Kd = 0.008            # s
Tf = Ts               # s, filtro de la derivada (un periodo de muestreo)
BUSCAR = False        # True → vuelve a sintonizar con los parámetros de arriba

a0 = K_servo - K
num_p, den_p = [K_servo], [M, B_servo, a0]


def coeficientes(Kp, Ki, Kd, Tf=Ts):
    """Coeficientes de la ecuación en diferencias (Tustin)."""
    c_i = Ki * Ts / 2
    c1 = (2 * Tf - Ts) / (2 * Tf + Ts)
    c2 = 2 * Kd / (2 * Tf + Ts)
    return c_i, c1, c2


# ------------------------------------------------------------------
# Simulación híbrida: planta NO lineal continua + PID discreto
# ------------------------------------------------------------------
def simulate(ref_fn, gains, t_end=1.2, mode="pid", dt=1e-4, quant=True,
             ks=K_servo, bs=B_servo, m=M):
    """mode 'pid': lazo externo con AS5600 · 'open': θ_ref = referencia."""
    Kp_, Ki_, Kd_ = gains
    c_i, c1, c2 = coeficientes(Kp_, Ki_, Kd_)
    wstep = WRITE_STEP if quant else 1e-12
    astep = AS5600_STEP if quant else 1e-12

    def rhs(x, u):
        th, w = x
        return np.array([w, (ks * (u - th) - bs * w - g * S * np.cos(th)) / m])

    th0 = theta_e
    u0 = th0 + g * S * np.cos(th0) / ks        # comando que sostiene 275°
    if mode == "open":
        # sin lazo externo se escribe directamente la referencia (275°):
        # la pata se queda unos 3° más abajo por el droop
        u0 = ref_fn(0.0)
        for _ in range(50):
            th0 = u0 - g * S * np.cos(th0) / ks
    x = np.array([th0, 0.0])
    n_ctrl = int(round(Ts / dt))
    I, e_prev, y_prev, D = u0, 0.0, th0, 0.0   # arranque sin salto
    queue = [u0] * DELAY_SAMPLES
    u_apl = u0
    out = []
    for k in range(int(round(t_end / dt))):
        t = k * dt
        if k % n_ctrl == 0:
            r = ref_fn(t)
            y = np.round(x[0] / astep) * astep             # lectura AS5600
            if mode == "pid":
                e = r - y
                I_new = I + c_i * (e + e_prev)
                D = c1 * D - c2 * (y - y_prev)
                u = Kp_ * e + I_new + D
                u_sat = np.clip(u, TH_MIN, TH_MAX)
                if u == u_sat:                            # anti-windup
                    I = I_new
                e_prev, y_prev = e, y
            else:
                u_sat = r
            u_q = TH_MIN + np.round((u_sat - TH_MIN) / wstep) * wstep  # write() entero
            queue.append(u_q)
            u_apl = queue.pop(0)                          # retardo de un pulso
        k1 = rhs(x, u_apl); k2 = rhs(x + dt / 2 * k1, u_apl)
        k3 = rhs(x + dt / 2 * k2, u_apl); k4 = rhs(x + dt * k3, u_apl)
        x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        if k % 10 == 0:
            out.append((t, x[0], u_apl, ref_fn(t)))
    t, th, u, r = np.array(out).T
    return t, np.rad2deg(th), np.rad2deg(u), np.rad2deg(r)


def metricas(t, y, t_step=0.1, y0=275.0, yf=285.0, banda=0.02):
    m = t >= t_step
    t, y = t[m] - t_step, y[m]
    span = yf - y0
    so = max(0.0, (y.max() - yf) / span * 100)
    fuera = np.where(np.abs(y - yf) > banda * abs(span))[0]
    ts = t[fuera[-1]] if len(fuera) else 0.0
    return so, ts, yf - y[-1]


def escalon(t, t_step=0.1, amp=10.0):
    return theta_e + (np.deg2rad(amp) if t >= t_step else 0.0)


def buscar_ganancias():
    """Menor t_s(2 %) con sobrepaso ≤ 2 %, sobre el modelo discreto con retardo."""
    mejor = None
    for kp in np.arange(0.2, 1.21, 0.05):
        for ki in np.arange(8, 30.1, 1.0):
            for kd in (0, 0.002, 0.004, 0.006, 0.008, 0.010, 0.012):
                t, th, _, _ = simulate(escalon, (kp, ki, kd), t_end=1.0,
                                       dt=5e-4, quant=False)
                so, ts, _ = metricas(t, th)
                if so <= 2 and (mejor is None or ts < mejor[0]):
                    mejor = (ts, so, kp, ki, kd)
    return mejor


def polos_discretos(Kp_, Ki_, Kd_, ks=K_servo, bs=B_servo, m=M):
    """Polos en z del lazo lineal: planta ZOH · retardo · PI-D Tustin."""
    a0_ = ks - K
    nz, dz, _ = signal.cont2discrete(([ks], [m, bs, a0_]), Ts, method="zoh")
    nz = np.trim_zeros(np.squeeze(nz), "f")
    dz = np.polymul(dz, [1, 0] + [0] * (DELAY_SAMPLES - 1))   # z^-1 retardo
    c_i, c1, c2 = coeficientes(Kp_, Ki_, Kd_)
    # U = [Kp + c_i (z+1)/(z-1)] E − c2 (z-1)/(z - c1) Y ,  E = R − Y
    # Lazo: Y = Gz U  →  char = den_pid·dz + num_lazo·nz
    Cpi_n = np.polyadd(np.polymul([Kp_], [1, -1]), np.polymul([c_i], [1, 1]))  # /(z-1)
    Cd_n = np.polymul([c2], [1, -1])                                           # /(z-c1)
    den_c = np.polymul([1, -1], [1, -c1])
    lazo_n = np.polyadd(np.polymul(Cpi_n, [1, -c1]), np.polymul(Cd_n, [1, -1]))
    char = np.polyadd(np.polymul(den_c, dz), np.polymul(lazo_n, nz))
    return np.roots(char)


if __name__ == "__main__":
    global_gains = (Kp, Ki, Kd)
    if BUSCAR:
        ts, so, Kp, Ki, Kd = buscar_ganancias()
        global_gains = (Kp, Ki, Kd)
        print(f"Búsqueda: Kp={Kp:.2f} Ki={Ki:.1f} Kd={Kd:.3f} (t_s={ts:.3f} s, SO={so:.1f} %)")

    print("=== Planta servo + pata ===")
    print(f"G_sp(s) = {K_servo:.4e} / ({M:.4e} s² + {B_servo:.4e} s + {a0:.4e})")
    print(f"Polos: {np.round(np.roots(den_p), 2)} · ganancia estática {K_servo / a0:.3f}")
    nz, dz, _ = signal.cont2discrete((num_p, den_p), Ts, method="zoh")
    print(f"G_sp(z), ZOH Ts = {Ts} s:  ({np.squeeze(nz)[1]:.5f} z + {np.squeeze(nz)[2]:.5f}) "
          f"/ (z² {dz[1]:+.5f} z {dz[2]:+.5f})")

    print("\n=== PID ===")
    print(f"Kp = {Kp}   Ki = {Ki} 1/s   Kd = {Kd} s   Tf = {Tf} s")
    pz = polos_discretos(*global_gains)
    print("Polos de lazo cerrado en z:", np.round(pz, 4))
    print("|z| máximo:", np.round(np.max(np.abs(pz)), 4), "(< 1 → estable)")
    ps = np.log(pz.astype(complex)) / Ts
    print("Equivalentes en s = ln(z)/Ts:", np.round(ps, 1))

    c_i, c1, c2 = coeficientes(*global_gains)
    print("\n=== Ecuación en diferencias para el ESP8266 (Ts = 20 ms, ángulos en rad) ===")
    print("e_k = r_k − y_k")
    print(f"I_k = I_(k−1) + {c_i:.4f}·(e_k + e_(k−1))")
    print(f"D_k = {c1:.4f}·D_(k−1) − {c2:.4f}·(y_k − y_(k−1))")
    print(f"u_k = {Kp}·e_k + I_k + D_k   → saturar a [220°, 330°], convertir a write()")

    # ---- Escalón de +10° ----
    tc = np.linspace(0, 1.1, 1101)
    t_open, th_open, u_open, _ = simulate(escalon, global_gains, mode="open")
    t_pid, th_pid, u_pid, _ = simulate(escalon, global_gains)
    t_pnq, th_pnq, _, _ = simulate(escalon, global_gains, quant=False)
    print("\n=== Escalón de +10° en la referencia (275° → 285°) ===")
    for nombre, (tt, yy), banda in [
        ("Solo el servo (sin lazo externo)", (t_open, th_open), 0.02),
        ("PID discreto, sin cuantización", (t_pnq, th_pnq), 0.02),
        ("PID discreto, con write() y AS5600", (t_pid, th_pid), 0.05),
    ]:
        so, ts, ess = metricas(tt, yy, banda=banda)
        print(f"{nombre:38s} SO={so:4.1f} %  t_s({int(banda*100)} %)={ts:5.3f} s  error final={ess:+.2f}°")
    tail = th_pid[t_pid > 0.8]
    print(f"Con cuantización, θ oscila entre {tail.min():.2f}° y {tail.max():.2f}° "
          f"(1 paso de write() = 0.61°)")

    # ---- Robustez: parámetros del servo ±30 % ----
    print("\n=== Robustez (K_servo, B_servo ±30 %, M ±20 %) ===")
    casos = []
    for fk in (0.7, 1.0, 1.3):
        for fb in (0.7, 1.0, 1.3):
            for fm in (0.8, 1.2):
                pz = polos_discretos(*global_gains, ks=K_servo * fk, bs=B_servo * fb, m=M * fm)
                tt, yy, _, _ = simulate(escalon, global_gains, quant=False,
                                        ks=K_servo * fk, bs=B_servo * fb, m=M * fm,
                                        t_end=1.0, dt=2e-4)
                so, ts, _ = metricas(tt, yy)
                casos.append((fk, fb, fm, np.max(np.abs(pz)), so, ts))
    peor_z = max(casos, key=lambda c: c[3])
    peor_so = max(casos, key=lambda c: c[4])
    peor_ts = max(casos, key=lambda c: c[5])
    print(f"|z| máximo en los 18 casos: {peor_z[3]:.3f} (K×{peor_z[0]}, B×{peor_z[1]}, M×{peor_z[2]})")
    print(f"Peor sobrepaso: {peor_so[4]:.1f} % (K×{peor_so[0]}, B×{peor_so[1]}, M×{peor_so[2]})")
    print(f"Peor t_s(2 %): {peor_ts[5]:.3f} s (K×{peor_ts[0]}, B×{peor_ts[1]}, M×{peor_ts[2]})")

    # ---- Gráfica ----
    fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True,
                           gridspec_kw={"height_ratios": [3, 2]})
    ax[0].axhline(285, color="0.6", lw=1, ls="--", label="Referencia 285°")
    ax[0].plot(t_open, th_open, color="#9c9c9c", lw=2, label=f"Solo el servo: se queda en {th_open[-1]:.0f}°")
    ax[0].plot(t_pnq, th_pnq, color="#2c7fb8", lw=1.5, ls="--", label="PID discreto (sin cuantización)")
    ax[0].plot(t_pid, th_pid, color="#1a4f8b", lw=2.2, label="PID discreto con write() y AS5600")
    ax[0].set_ylabel("θ de la rodilla (°)")
    ax[0].set_title("PID de la pata: escalón de +10° (Kp = %.2f, Ki = %.0f, Kd = %.3f, Ts = 20 ms)" % global_gains)
    ax[0].legend(loc="lower right", fontsize=9, frameon=False)
    ax[0].grid(alpha=0.3)
    ax[1].step(t_pid, u_pid, where="post", color="#1a4f8b", lw=1.8, label="θ_ref enviado por el PID")
    ax[1].step(t_open, u_open, where="post", color="#9c9c9c", lw=1.5, label="θ_ref sin lazo externo")
    ax[1].set_ylabel("Comando al servo (°)")
    ax[1].set_xlabel("Tiempo (s)")
    ax[1].legend(loc="lower right", fontsize=9, frameon=False)
    ax[1].grid(alpha=0.3)
    for a in ax:
        a.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig("pid_pata_escalon.png", dpi=150)
    print("\nGráfica: pid_pata_escalon.png")
