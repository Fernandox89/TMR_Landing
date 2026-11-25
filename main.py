"""
Misión completa: despegar, buscar el ArUco, seguir la plataforma, descender y aterrizar.

Uso:
    python main.py              # vuelo real
    python main.py --dry-run    # NO despega: mueve el dron con la mano y mira estados/comandos
    python main.py --solo-seguimiento   # vuela y sigue al ArUco pero NUNCA baja (calibración)

Teclas (con la ventana de video seleccionada):
    t        despegar (o en --dry-run, arrancar la misión sin motores)
    b        abortar descenso -> volver a BUSQUEDA
    l        aterrizar normal y volver a ESPERA
    ESPACIO  EMERGENCIA: corta motores al instante
    q / ESC  aterrizar y salir
    w/a/s/d  (solo con --solo-seguimiento) mover el punto objetivo 15 cm para generar
             movimiento y calibrar la velocidad con tools/analyze_log.py
"""
import argparse
import os
import sys
import time

import cv2

# Asegurar que Python encuentre los módulos del proyecto
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config.loader import load_settings
from vision.vision_system import VisionSystem
from control.tello_driver import TelloDriver
from logic.navigator import Navigator
from logic.state_machine import StateMachine, MisionState
from ui.telemetry_hud import TelemetryHUD
from utils.flight_logger import FlightLogger

KEY_ESC = 27


def new_context():
    """Memoria de la misión entre ciclos que no le corresponde a la FSM."""
    return {"pid_active": False,   # ¿Se usaron los PID el ciclo anterior? (para reiniciarlos al reactivarlos)
            "committed": False,    # ¿Ya entramos a la bajada final? (ya no se detiene por cambios de sentido)
            "gate_since": None,    # Desde cuándo estamos esperando ventana para la bajada final
            "alt_ref": None,       # Altura (ToF) a mantener cuando el descenso se pausa
            "gate_msg": "",        # Aviso para el HUD
            "search_disp": None,   # Desplazamiento (cm) desde que empezó a buscar quieto
            "search_t0": None}     # Cuándo empezó la búsqueda sin ArUco (para la espiral)


def hold_altitude(tof, target_cm, m):
    """
    Control P de altura con el ToF. Hace falta porque el Tello se HUNDE al moverse
    (5-8 cm/s medidos en el laboratorio): con throttle 0 no mantiene su altura.
    """
    err = target_cm - tof
    if abs(err) < m["alt_hold_deadband_cm"]:
        return 0
    lim = m["alt_hold_max_rc"]
    return int(max(-lim, min(lim, m["alt_hold_kp"] * err)))


def square_spiral(n):
    """Los primeros n puntos de una espiral cuadrada en unidades de cuadrícula: (0,0), (1,0), (1,1), (0,1), (-1,1)..."""
    pts, x, y, dx, dy, seg = [(0, 0)], 0, 0, 1, 0, 1
    while len(pts) < n:
        for _ in range(2):
            for _ in range(seg):
                x, y = x + dx, y + dy
                pts.append((x, y))
            dx, dy = -dy, dx
        seg += 1
    return pts[:n]


def search_target(ctx, nav, m):
    """
    Punto (cm, respecto a donde empezó la búsqueda) al que hay que ir ahora. Los primeros
    search_spiral_after_s se queda en el lugar; después recorre una espiral cuadrada con
    pasos de search_spiral_step_cm (más o menos lo que abarca la cámara a la altura de
    búsqueda), search_spiral_dwell_s en cada punto, hasta search_spiral_max_cm y vuelve a empezar.
    Cubre la deriva del despegue (cuando no hay control) y cualquier pérdida larga.
    """
    now = nav.clock()
    if ctx["search_t0"] is None:
        ctx["search_t0"] = now
    t = now - ctx["search_t0"] - m["search_spiral_after_s"]
    if t < 0:
        return 0.0, 0.0
    step = m["search_spiral_step_cm"]
    rings = max(1, int(m["search_spiral_max_cm"] // step))
    pts = square_spiral((2 * rings + 1) ** 2)
    i = int(t // m["search_spiral_dwell_s"]) % len(pts)
    return pts[i][0] * step, pts[i][1] * step


def hold_position(telem, nav, m, dt, ctx, target=(0.0, 0.0)):
    """
    Mantenerse en el lugar SIN ver el ArUco. Con rc 0 el Tello no se queda quieto: deriva
    sola (~5 cm/s, y algunos Tello ~12 cm/s siempre hacia el mismo lado). Integramos la
    velocidad que reporta el Tello para saber cuánto nos hemos movido y regresamos.
    vel_x = vgy (derecha), vel_y = vgx (adelante): no dependen de la cámara ni del espejo,
    así que roll y pitch se corrigen directo, sin los signos de la visión.
    """
    vx = telem["vel_x"] * abs(nav.vel_gain_x)   # cm/s a la derecha
    vy = telem["vel_y"] * abs(nav.vel_gain_y)   # cm/s hacia adelante
    if ctx["search_disp"] is None:
        ctx["search_disp"] = [0.0, 0.0]
    d = ctx["search_disp"]
    d[0] += vx * dt
    d[1] += vy * dt
    lim = m["search_hold_max_rc"]

    def axis(disp, vel):
        u = -(m["search_hold_kp"] * disp + m["search_hold_kv"] * vel)
        if abs(u) < 1:
            return 0
        u += nav.rc_deadband if u > 0 else -nav.rc_deadband
        return int(max(-lim, min(lim, u)))
    return axis(d[0] - target[0], vx), axis(d[1] - target[1], vy)


def compute_commands(state, nav, telem, m, dt, ctx):
    """
    Traduce el estado de la misión a comandos rc (roll, pitch, throttle).
    ctx es la memoria entre ciclos (ver new_context).
    """
    roll = pitch = throttle = 0
    use_pid = False
    holding = False   # ¿El throttle viene del control de altura?
    tof = telem["tof_cm"]
    ctx["gate_msg"] = ""

    if state == MisionState.DESPEGUE:
        # Estabilizándose después del despegue: que la deriva no lo aleje
        roll, pitch = hold_position(telem, nav, m, dt, ctx)

    elif state == MisionState.BUSQUEDA:
        if nav.has_track and nav.time_since_seen() < m["search_predict_s"]:
            # Lo acabamos de perder: seguir hacia donde el Kalman cree que va la plataforma
            use_pid = True
            throttle, holding = hold_altitude(tof, m["track_tof_cm"], m), True
        else:
            # Lo perdimos de verdad: subir para ver más área (con el ToF: la altura 'h'
            # del Tello no es confiable), quedarse en el lugar un momento, porque la
            # plataforma va y viene y puede volver a pasar, y si no aparece, espiral.
            throttle = min(hold_altitude(tof, m["search_tof_cm"], m), m["search_climb_rc"])
            holding = True
            roll, pitch = hold_position(telem, nav, m, dt, ctx, search_target(ctx, nav, m))

    elif state == MisionState.SEGUIMIENTO:
        use_pid = True
        throttle, holding = hold_altitude(tof, m["track_tof_cm"], m), True

    elif state == MisionState.DESCENSO:
        use_pid = True
        ex, ey = nav.error_xy
        # Solo bajamos si estamos bien centrados; si no, mantenemos altura y corregimos
        if max(abs(ex), abs(ey)) < m["descent_gate_cm"]:
            rate = m["descent_rc_slow"] if tof < m["descent_slow_tof_cm"] else m["descent_rc_fast"]
            throttle = -rate
            ctx["alt_ref"] = tof
            # Antes de la bajada final (que termina a ciegas) revisar que la plataforma
            # no vaya a cambiar de sentido a la mitad.
            if m["reversal_gate"] and tof <= m["commit_tof_cm"] and not ctx["committed"]:
                now = nav.clock()
                if ctx["gate_since"] is None:
                    ctx["gate_since"] = now
                waited = now - ctx["gate_since"]
                safe = nav.reversal.safe_window(m["final_phase_s"], m["reversal_settle_s"],
                                                require_period=m["require_period_if_moving"])
                if safe or waited > m["max_gate_wait_s"]:
                    ctx["committed"] = True
                else:
                    throttle, holding = hold_altitude(tof, m["commit_tof_cm"], m), True
                    ctx["gate_msg"] = ("APRENDIENDO RITMO DE LA PLATAFORMA" if nav.reversal.period is None
                                       else "ESPERANDO CAMBIO DE SENTIDO") + f" ({waited:.0f}s)"

        else:
            # Descentrados: pausar la bajada manteniendo la altura a la que íbamos
            if ctx["alt_ref"] is None:
                ctx["alt_ref"] = tof
            throttle, holding = hold_altitude(tof, ctx["alt_ref"], m), True

    elif state == MisionState.PUNTO_CIEGO:
        # Sin visión: confiamos en la predicción del Kalman (posición + velocidad de la plataforma)
        use_pid = True
        throttle = -m["blind_descent_rc"]

    if state not in (MisionState.DESPEGUE, MisionState.BUSQUEDA) or use_pid:
        ctx["search_disp"] = None
        ctx["search_t0"] = None
    if state not in (MisionState.DESCENSO, MisionState.PUNTO_CIEGO):
        ctx["committed"] = False
        ctx["gate_since"] = None
        ctx["alt_ref"] = None
    if use_pid:
        if not ctx["pid_active"]:
            nav.reset_controllers()
        roll, pitch = nav.compute_xy_commands(dt)
    ctx["pid_active"] = use_pid

    # Moverse adelante y de lado a la vez hunde al Tello (~8 cm/s medidos). Si estamos
    # manteniendo altura, lo compensamos antes de que el ToF lo note.
    if holding and abs(roll) > nav.rc_deadband and abs(pitch) > nav.rc_deadband:
        throttle = min(100, throttle + m["sink_comp_rc"])

    return roll, pitch, throttle


def final_touchdown(driver, mode):
    """Toque final sobre la plataforma."""
    if mode == "emergency":
        print("[MISION] Toque final: cortando motores.")
        driver.emergency()
    else:
        print("[MISION] Toque final: aterrizaje normal.")
        driver.set_velocity(0, 0, 0, 0)
        driver.land()


def main():
    parser = argparse.ArgumentParser(description="Aterrizaje autónomo en plataforma móvil (TMR)")
    parser.add_argument("--dry-run", action="store_true", help="No despega; prueba con el dron en la mano")
    parser.add_argument("--solo-seguimiento", action="store_true",
                        help="Sigue al ArUco pero nunca desciende (para calibrar y ajustar PID)")
    parser.add_argument("--config", default=None, help="Ruta a un settings.json alternativo")
    args = parser.parse_args()

    settings = load_settings(args.config)
    m = settings["mission"]
    dry_run = args.dry_run or m["dry_run"]

    print("--- TMR LANDING: MISIÓN COMPLETA ---" + ("  [DRY-RUN: SIN MOTORES]" if dry_run else "")
          + ("  [SOLO SEGUIMIENTO]" if args.solo_seguimiento else ""))

    driver = TelloDriver()
    vision = VisionSystem(**settings["vision"])
    navigator = Navigator(**settings["navigator"])
    fsm = StateMachine(**settings["fsm"])
    if args.solo_seguimiento:
        fsm.centered_to_descend_s = float("inf")   # nunca pasa a DESCENSO
    base_setpoint = (navigator.setpoint_x, navigator.setpoint_y)
    hud = TelemetryHUD()
    logger = FlightLogger(**settings["logging"])

    period = 1.0 / m["loop_hz"]
    ctx = new_context()
    mission_done = False
    roll = pitch = throttle = 0

    try:
        driver.connect()
        print("[INFO] Esperando a que inicialice el video...")
        time.sleep(3)
        print(__doc__)

        prev_loop = time.time()
        while True:
            loop_start = time.time()
            dt = loop_start - prev_loop
            prev_loop = loop_start

            # 1. VISIÓN
            frame_limpio, ids, pos_cm = vision.update(driver.get_frame())
            is_visible = pos_cm is not None

            # 2. SENSORES
            telem = driver.get_telemetry()

            # 3. ESTIMACIÓN (Kalman) - corre siempre, aunque estemos en tierra
            navigator.update(pos_cm, telem["vel_x"], telem["vel_y"], dt)

            # 4. DECISIÓN (FSM) con el error estimado, no el crudo
            error_xy = navigator.error_xy if navigator.has_track else None
            state = fsm.update(is_visible, pos_cm, telem["tof_cm"], error_xy=error_xy)

            # 5. SEGURIDAD: batería baja en vuelo
            if driver.is_flying and telem["battery"] < m["min_battery_flight"]:
                print(f"[SEGURIDAD] Batería {telem['battery']}% -> aterrizando.")
                driver.land()
                fsm.set_state(MisionState.ESPERA)
                state = fsm.state

            # 6. ACCIÓN
            if state == MisionState.ATERRIZAJE:
                if not mission_done:
                    if driver.is_flying:
                        final_touchdown(driver, m["final_mode"])
                    mission_done = True
                    print("[MISION] Completada. Presiona 'q' para salir.")
                roll = pitch = throttle = 0
                ctx = new_context()
            elif state == MisionState.ESPERA:
                roll = pitch = throttle = 0
                ctx = new_context()
                driver.hover()
            else:
                roll, pitch, throttle = compute_commands(state, navigator, telem, m, dt, ctx)
                driver.set_velocity(roll, pitch, throttle, 0)

            # 7. INTERFAZ Y LOG
            ex, ey = navigator.error_xy
            vtx, vty = navigator.target_vel
            extra = [
                f"EST err x={ex:+.0f} y={ey:+.0f} cm  Vplat x={vtx:+.0f} y={vty:+.0f} cm/s",
                f"CMD roll={roll:+d} pitch={pitch:+d} thr={throttle:+d}  {1.0 / max(dt, 1e-3):.0f} Hz",
            ]
            period_s = navigator.reversal.period
            if period_s is not None:
                extra.append(f"Cambio de sentido cada {period_s:.1f}s, proximo en "
                             f"{navigator.reversal.time_to_next_reversal():+.1f}s")
            if ctx["gate_msg"]:
                extra.append(ctx["gate_msg"])
            if dry_run:
                extra.append("DRY-RUN: motores desactivados")
            if args.solo_seguimiento:
                extra.append(f"SOLO SEGUIMIENTO  setpoint x={navigator.setpoint_x:+.0f} y={navigator.setpoint_y:+.0f}")
            if frame_limpio is not None:
                frame_hud = hud.draw(frame_limpio, telem, mission_state=state, target_pos=pos_cm,
                                     extra_lines=extra)
                cv2.imshow("TMR Landing", frame_hud)
                logger.write_frame(frame_hud)

            logger.log(dt=dt, state=state, visible=int(is_visible),
                       raw_x=pos_cm[0] if is_visible else None,
                       raw_y=pos_cm[1] if is_visible else None,
                       raw_z=pos_cm[2] if is_visible else None,
                       est_x=navigator.est_x, est_y=navigator.est_y, vt_x=vtx, vt_y=vty,
                       vel_x=telem["vel_x"], vel_y=telem["vel_y"],
                       cmd_roll=roll, cmd_pitch=pitch, cmd_throttle=throttle,
                       tof_cm=telem["tof_cm"], height_cm=telem["height_cm"],
                       battery=telem["battery"], flying=int(driver.is_flying))

            # 8. TECLADO
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), KEY_ESC):
                break
            elif key == ord(' '):
                print("[SEGURIDAD] EMERGENCIA: motores cortados.")
                driver.emergency()
                fsm.set_state(MisionState.ESPERA)
            elif key == ord('l'):
                driver.land()
                fsm.set_state(MisionState.ESPERA)
            elif args.solo_seguimiento and key in (ord('w'), ord('a'), ord('s'), ord('d')):
                # Mover el objetivo para que el dron se desplace sobre la plataforma quieta
                dx = {ord('a'): -15, ord('d'): 15}.get(key, 0)
                dy = {ord('w'): -15, ord('s'): 15}.get(key, 0)
                # Máximo 30 cm alrededor del setpoint original
                bx, by = base_setpoint
                navigator.setpoint_x = max(bx - 30, min(bx + 30, navigator.setpoint_x + dx))
                navigator.setpoint_y = max(by - 30, min(by + 30, navigator.setpoint_y + dy))
            elif key == ord('b') and state in (MisionState.DESCENSO, MisionState.PUNTO_CIEGO):
                fsm.set_state(MisionState.BUSQUEDA)
            elif key == ord('t') and state in (MisionState.ESPERA, MisionState.ATERRIZAJE) \
                    and not driver.is_flying:
                mission_done = False
                if dry_run:
                    fsm.set_state(MisionState.DESPEGUE)
                elif telem["battery"] < m["min_battery_takeoff"]:
                    print(f"[SEGURIDAD] Batería {telem['battery']}% < {m['min_battery_takeoff']}%. No despego.")
                else:
                    driver.takeoff(extra_height_cm=m["takeoff_extra_cm"])
                    fsm.set_state(MisionState.DESPEGUE)
                prev_loop = time.time()  # el despegue bloquea unos segundos; no contar ese dt

            # 9. RITMO FIJO DEL CICLO
            sleep = period - (time.time() - loop_start)
            if sleep > 0:
                time.sleep(sleep)
    finally:
        # Pase lo que pase (error, Ctrl+C, 'q'), el dron aterriza
        driver.shutdown()
        logger.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
