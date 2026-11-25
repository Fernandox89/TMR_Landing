#!/usr/bin/env python3
"""
Analiza un vuelo guardado por main.py (logs/AAAAMMDD_HHMMSS.csv).

Qué te dice:
  1. Ritmo real del ciclo (Hz).
  2. Secuencia de estados y cuánto duró cada uno.
  3. ESCALA Y SIGNO de la velocidad del Tello  -> navigator.vel_scale_* y vel_sign_*
     (requiere un vuelo de SEGUIMIENTO con la plataforma QUIETA).
  4. Ganancia rc -> cm/s del dron              -> navigator.feedforward_gain sugerido.
  5. Error de seguimiento y velocidad estimada de la plataforma.

Uso:
    python3 tools/analyze_log.py                 # analiza el log más reciente
    python3 tools/analyze_log.py logs/xxx.csv
    python3 tools/analyze_log.py --plot          # además grafica (requiere matplotlib)
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRACK_STATES = ("SEGUIMIENTO", "DESCENSO")


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    # Si el programa se cortó a la mitad de una escritura, la última fila puede venir rota
    def valid(r):
        try:
            float(r["t"]); float(r["dt"])
            return None not in r.values()
        except (TypeError, ValueError):
            return False
    rows = [r for r in rows if valid(r)]
    cols = {}
    for k in rows[0].keys():
        vals = [r[k] for r in rows]
        try:
            cols[k] = np.array([float(v) if v != "" else np.nan for v in vals])
        except ValueError:
            cols[k] = np.array(vals)
    return cols


def resample(t, y, step):
    """Re-muestrea a paso fijo (el Tello manda su estado a ~10 Hz y se repite entre ciclos)."""
    grid = np.arange(t[0], t[-1], step)
    return grid, np.interp(grid, t, y)


def section(title):
    print("\n" + "=" * 70 + f"\n{title}\n" + "=" * 70)


def loop_stats(c):
    section("1. RITMO DEL CICLO")
    dt = c["dt"][1:]
    dt = dt[(dt > 0) & (dt < 1.0)]
    print(f"dt medio = {dt.mean() * 1000:.1f} ms  ({1 / dt.mean():.1f} Hz)   p95 = {np.percentile(dt, 95) * 1000:.1f} ms")
    if 1 / dt.mean() < 15:
        print("  ! Ciclo lento. Cierra programas, baja loop_hz o desactiva record_video.")


def state_stats(c):
    section("2. ESTADOS")
    st, t = c["state"], c["t"]
    start = 0
    for i in range(1, len(st) + 1):
        if i == len(st) or st[i] != st[start]:
            print(f"  t={t[start]:7.2f}s  {st[start]:<12} {t[i - 1] - t[start]:6.2f}s")
            start = i


def velocity_fit(c, axis):
    """
    Con la plataforma quieta:  d(p)/dt = -k * vel_tello   ->  k = vel_sign * vel_scale
    """
    raw, vel, t = c[f"raw_{axis}"], c[f"vel_{axis}"], c["t"]
    ok = (c["flying"] == 1) & ~np.isnan(raw) & np.isin(c["state"], TRACK_STATES + ("BUSQUEDA",))
    if ok.sum() < 30:
        return None
    # Tramos continuos con ArUco visible
    idx = np.where(ok)[0]
    breaks = np.where(np.diff(idx) > 1)[0]
    segments = np.split(idx, breaks + 1)
    xs, ys = [], []
    for seg in segments:
        if len(seg) < 10:
            continue
        tg, p = resample(t[seg], raw[seg], 0.1)
        _, v = resample(t[seg], vel[seg], 0.1)
        if len(tg) < 5:
            continue
        # Derivada centrada de la posición medida
        dp = (p[2:] - p[:-2]) / 0.2
        xs.append(v[1:-1])
        ys.append(dp)
    if not xs:
        return None
    v = np.concatenate(xs)
    dp = np.concatenate(ys)
    moving = np.abs(v) > 0.5
    if moving.sum() < 10:
        return None
    v, dp = v[moving], dp[moving]
    k = -np.sum(dp * v) / np.sum(v * v)
    r = np.corrcoef(v, dp)[0, 1]
    return k, r, len(v)


def velocity_section(c):
    section("3. ESCALA Y SIGNO DE LA VELOCIDAD DEL TELLO (plataforma QUIETA)")
    fits = {}
    for axis in ("x", "y"):
        res = velocity_fit(c, axis)
        if res is None:
            print(f"  eje {axis}: pocos datos (vuela más tiempo siguiendo el ArUco y moviéndote).")
            continue
        k, r, n = res
        fits[axis] = k
        conf = "BUENA" if abs(r) > 0.7 else "REGULAR" if abs(r) > 0.4 else "MALA (no confiar)"
        print(f"  eje {axis}: k = {k:+.2f}  (correlación {abs(r):.2f}, {conf}, {n} muestras)")
        print(f"      -> navigator.vel_sign_{axis} = {1 if k >= 0 else -1}   "
              f"navigator.vel_scale_{axis} = {abs(k):.1f}")
    print("  Nota: si la plataforma se movía durante el vuelo, este ajuste NO es válido.")
    return fits


def rc_gain_section(c, fits):
    section("4. GANANCIA rc -> cm/s DEL DRON (para feedforward_gain)")
    for axis, cmd_col in (("x", "cmd_roll"), ("y", "cmd_pitch")):
        if axis not in fits:
            print(f"  eje {axis}: falta la escala de velocidad (sección 3).")
            continue
        ok = c["flying"] == 1
        if ok.sum() < 30:
            continue
        t = c["t"][ok]
        tg, cmd = resample(t, c[cmd_col][ok], 0.1)
        _, vel = resample(t, c[f"vel_{axis}"][ok], 0.1)
        w = fits[axis] * vel  # velocidad del dron en cm/s, eje de la cámara
        best = None
        for lag in range(0, 11):  # el dron tarda en responder: probar retrasos de 0 a 1 s
            a = cmd[: len(cmd) - lag] if lag else cmd
            b = w[lag:]
            if np.sum(a * a) < 1:
                continue
            g = np.sum(a * b) / np.sum(a * a)
            r = np.corrcoef(a, b)[0, 1] if np.std(a) > 0 and np.std(b) > 0 else 0
            if best is None or abs(r) > abs(best[1]):
                best = (g, r, lag)
        if best is None:
            print(f"  eje {axis}: sin comandos suficientes.")
            continue
        g, r, lag = best
        print(f"  eje {axis}: {abs(g):.2f} cm/s por unidad rc  (retraso {lag * 100} ms, correlación {abs(r):.2f})")
        if abs(g) > 0.05:
            print(f"      -> feedforward_gain sugerido ≈ {1 / abs(g):.2f}  (usa el promedio de ambos ejes)")


def tracking_section(c):
    section("5. SEGUIMIENTO")
    ok = np.isin(c["state"], TRACK_STATES)
    if ok.sum() == 0:
        print("  No hubo SEGUIMIENTO/DESCENSO en este vuelo.")
        return
    ex, ey = c["est_x"][ok], c["est_y"][ok]
    print(f"  error estimado RMS: x = {np.sqrt(np.mean(ex ** 2)):.1f} cm   y = {np.sqrt(np.mean(ey ** 2)):.1f} cm")
    vis = c["visible"][ok]
    print(f"  ArUco visible el {100 * np.mean(vis):.0f}% del tiempo")
    vtx, vty = c["vt_x"][ok], c["vt_y"][ok]
    print(f"  velocidad estimada de la plataforma: |x| p90 = {np.percentile(np.abs(vtx), 90):.1f} cm/s"
          f"   |y| p90 = {np.percentile(np.abs(vty), 90):.1f} cm/s")
    print("  (con la plataforma QUIETA debería ser < 5 cm/s; si es mayor, revisa la escala de la sección 3)")


def plot(c, path):
    import matplotlib.pyplot as plt
    t = c["t"]
    fig, ax = plt.subplots(4, 1, sharex=True, figsize=(12, 10))
    for i, axis in enumerate(("x", "y")):
        ax[i].plot(t, c[f"raw_{axis}"], ".", ms=2, label=f"PnP {axis}")
        ax[i].plot(t, c[f"est_{axis}"], label=f"Kalman {axis}")
        ax[i].plot(t, c[f"vt_{axis}"], label=f"V plataforma {axis} (cm/s)")
        ax[i].legend(loc="upper right"); ax[i].grid(True)
    ax[2].plot(t, c["cmd_roll"], label="roll"); ax[2].plot(t, c["cmd_pitch"], label="pitch")
    ax[2].plot(t, c["cmd_throttle"], label="throttle"); ax[2].legend(loc="upper right"); ax[2].grid(True)
    ax[3].plot(t, c["tof_cm"], label="ToF cm"); ax[3].plot(t, c["height_cm"], label="altura cm")
    ax[3].legend(loc="upper right"); ax[3].grid(True); ax[3].set_xlabel("t (s)")
    fig.suptitle(os.path.basename(path))
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="?")
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()

    path = args.csv
    if path is None:
        logs = sorted(glob.glob(os.path.join(ROOT, "logs", "*.csv")))
        if not logs:
            sys.exit("No hay logs en logs/. Vuela primero con main.py.")
        path = logs[-1]
    print(f"Analizando {path}")
    c = load(path)
    if len(c["t"]) < 10:
        sys.exit("Log demasiado corto.")

    loop_stats(c)
    state_stats(c)
    fits = velocity_section(c)
    rc_gain_section(c, fits)
    tracking_section(c)
    if args.plot:
        plot(c, path)


if __name__ == "__main__":
    main()
