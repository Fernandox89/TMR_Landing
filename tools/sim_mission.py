#!/usr/bin/env python3
"""
Simulador SIMPLE de la misión completa, sin dron.

Usa exactamente el mismo Navigator, StateMachine y compute_commands() que main.py,
pero en lugar del Tello hay un modelo físico sencillo:
  - La plataforma va y viene en línea recta (como ida_y_vuelta.py).
  - El dron responde a los comandos rc con un retraso (primer orden), con zona muerta,
    sube rápido y baja lento, y se hunde al moverse (valores medidos en Tello reales,
    tomados del modelo lab_healthy de Tello-Sim-Lab).
  - El video llega con latencia y la medición del ArUco tiene ruido.
  - El Tello reporta su velocidad en enteros (dm/s), a 10 Hz.
  - Debajo de cierta altura el ArUco deja de verse (punto ciego del espejo).

NO sustituye volar: sirve para detectar errores de lógica o de signos y para
probar cambios de parámetros antes de ir al laboratorio.

Uso:
    python3 tools/sim_mission.py                       # plataforma a 15 cm/s
    python3 tools/sim_mission.py --speed 25 --plot
    python3 tools/sim_mission.py --speed 0             # plataforma quieta
"""
import argparse
import math
import os
import random
import sys
from collections import deque

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from config.loader import load_settings
from logic.navigator import Navigator
from logic.state_machine import StateMachine, MisionState
from main import compute_commands, new_context


class SimClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def platform_velocity(t, speed, leg_s, pause_s):
    """Igual que ida_y_vuelta.py: leg_s adelante, pausa, leg_s atrás, pausa, y se repite."""
    cycle = 2 * (leg_s + pause_s)
    phase = t % cycle
    if phase < leg_s:
        return speed
    if phase < leg_s + pause_s:
        return 0.0
    if phase < 2 * leg_s + pause_s:
        return -speed
    return 0.0


def simulate(settings=None, platform_speed=15.0, platform_leg_s=3.0, platform_pause_s=1.0,
             platform_axis="y", platform_t0=0.0,
             duration=90.0, seed=0, rc_gain=1.45, response_tau=0.75, video_latency=0.17,
             rc_deadband=5.0, up_gain=0.4, down_gain=0.15, vertical_deadband=3.0, combo_sink=8.0,
             vision_noise_cm=1.0, start_offset=(25.0, -20.0), start_height=110.0,
             blind_height=30.0, navigator_overrides=None, true_vel_factor=1.0):
    """Corre una misión simulada. Regresa un diccionario con el resultado y la historia."""
    settings = settings or load_settings()
    rng = random.Random(seed)
    clock = SimClock()
    nav_cfg = dict(settings["navigator"], **(navigator_overrides or {}))
    nav = Navigator(**nav_cfg, clock=clock)
    fsm = StateMachine(**settings["fsm"], clock=clock)
    m = settings["mission"]

    dt = 1.0 / m["loop_hz"]
    k_x = nav_cfg["vel_sign_x"] * nav_cfg["vel_scale_x"]   # suponemos que la config es la correcta
    k_y = nav_cfg["vel_sign_y"] * nav_cfg["vel_scale_y"]

    # Estado físico. Posiciones en cm, en el marco de la cámara: p = plataforma - dron.
    plat = [0.0, 0.0]
    drone = [-start_offset[0], -start_offset[1]]
    w = [0.0, 0.0]          # velocidad del dron (cm/s)
    h = start_height        # altura sobre la plataforma (cm)
    vz = 0.0
    vel_raw = (0, 0)
    ctx = new_context()
    meas_queue = deque()    # mediciones en camino (latencia de video)
    history = []
    axis_i = 0 if platform_axis == "x" else 1

    fsm.set_state(MisionState.DESPEGUE)
    steps = int(duration / dt)
    for step in range(steps):
        clock.t = step * dt

        # --- Física de la plataforma: ida y vuelta
        plat[axis_i] += platform_velocity(clock.t + platform_t0, platform_speed,
                                          platform_leg_s, platform_pause_s) * dt
        p_true = (plat[0] - drone[0], plat[1] - drone[1])

        # --- Visión con latencia, ruido y campo de visión limitado
        fov_ok = abs(p_true[0]) < 0.55 * h and abs(p_true[1]) < 0.4 * h
        visible_now = fov_ok and h > blind_height
        meas = (p_true[0] + rng.gauss(0, vision_noise_cm), p_true[1] + rng.gauss(0, vision_noise_cm), h) \
            if visible_now else None
        meas_queue.append((clock.t, meas))
        pos_cm = None
        while meas_queue and meas_queue[0][0] <= clock.t - video_latency:
            pos_cm = meas_queue.popleft()[1]

        # --- Telemetría (velocidad entera a 10 Hz)
        if step % max(1, int(0.1 / dt)) == 0:
            # true_vel_factor != 1 simula que la escala configurada está mal
            vel_raw = (round(w[0] * true_vel_factor / k_x), round(w[1] * true_vel_factor / k_y))
        telem = {"tof_cm": h, "height_cm": h + 10, "battery": 80,
                 "vel_x": vel_raw[0], "vel_y": vel_raw[1]}

        # --- Mismo flujo que main.py
        nav.update(pos_cm, telem["vel_x"], telem["vel_y"], dt)
        error_xy = nav.error_xy if nav.has_track else None
        state = fsm.update(pos_cm is not None, pos_cm, telem["tof_cm"], error_xy=error_xy)

        if state == MisionState.ATERRIZAJE:
            err = (p_true[0] ** 2 + p_true[1] ** 2) ** 0.5
            return {"result": "landed", "t": clock.t, "landing_error_cm": err, "history": history}

        roll = pitch = throttle = 0
        if state != MisionState.ESPERA:
            roll, pitch, throttle = compute_commands(state, nav, telem, m, dt, ctx)

        # --- Física del dron: el comando que reduce p mueve al dron hacia la plataforma
        u = (nav_cfg["axis_sign_roll"] * roll, nav_cfg["axis_sign_pitch"] * pitch)
        def stick(rc, deadband, gain):
            return math.copysign(gain * (abs(rc) - deadband), rc) if abs(rc) > deadband else 0.0

        for i in range(2):
            target_w = -stick(u[i], rc_deadband, rc_gain)
            w[i] += (target_w - w[i]) * dt / response_tau
            drone[i] += w[i] * dt  # p = plataforma - dron, así que dp/dt = v_plat - w
        want_vz = stick(throttle, vertical_deadband, up_gain if throttle > 0 else down_gain)
        if abs(roll) > rc_deadband and abs(pitch) > rc_deadband:
            want_vz -= combo_sink  # el Tello se hunde al moverse adelante y de lado a la vez
        vz += (want_vz - vz) * dt / 0.6
        h = max(5.0, h + vz * dt)

        history.append({"t": clock.t, "state": state, "p_x": p_true[0], "p_y": p_true[1],
                        "est_x": nav.est_x, "est_y": nav.est_y,
                        "vt_x": nav.target_vel[0], "vt_y": nav.target_vel[1],
                        "h": h, "roll": roll, "pitch": pitch, "throttle": throttle,
                        "visible": pos_cm is not None})

    return {"result": "timeout", "t": duration, "landing_error_cm": None, "history": history}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=15.0, help="Velocidad de la plataforma cm/s")
    parser.add_argument("--leg", type=float, default=3.0, help="Segundos de cada tramo (como ida_y_vuelta.py)")
    parser.add_argument("--pause", type=float, default=1.0, help="Segundos de pausa entre tramos")
    parser.add_argument("--axis", choices=["x", "y"], default="y", help="Eje en el que se mueve la plataforma")
    parser.add_argument("--ff", type=float, default=None, help="Sobrescribe feedforward_gain")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()

    overrides = {"feedforward_gain": args.ff} if args.ff is not None else None
    res = simulate(platform_speed=args.speed, platform_leg_s=args.leg, platform_pause_s=args.pause,
                   platform_axis=args.axis, seed=args.seed, platform_t0=args.seed * 0.7,
                   navigator_overrides=overrides)

    states = []
    for row in res["history"]:
        if not states or states[-1][0] != row["state"]:
            states.append((row["state"], row["t"]))
    for s, t in states:
        print(f"  t={t:6.2f}s  {s}")
    if res["result"] == "landed":
        print(f"\nATERRIZÓ en t={res['t']:.1f}s con error de {res['landing_error_cm']:.1f} cm")
    else:
        print("\nNO aterrizó (timeout)")

    if args.plot:
        import matplotlib.pyplot as plt
        hist = res["history"]
        t = [r["t"] for r in hist]
        fig, ax = plt.subplots(3, 1, sharex=True, figsize=(11, 8))
        for key in ("p_x", "est_x", "p_y", "est_y"):
            ax[0].plot(t, [r[key] for r in hist], label=key)
        ax[0].set_ylabel("cm"); ax[0].legend(); ax[0].grid(True)
        for key in ("vt_x", "vt_y"):
            ax[1].plot(t, [r[key] for r in hist], label=key)
        ax[1].set_ylabel("cm/s"); ax[1].legend(); ax[1].grid(True)
        ax[2].plot(t, [r["h"] for r in hist], label="altura")
        ax[2].legend(); ax[2].grid(True); ax[2].set_xlabel("t (s)")
        plt.tight_layout(); plt.show()


if __name__ == "__main__":
    main()
