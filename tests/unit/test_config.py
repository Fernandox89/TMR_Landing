"""Verifica que settings.json carga y que cada sección coincide con los parámetros de su clase.
Así un error de dedo en el JSON se detecta aquí y no en el laboratorio."""
from config.loader import load_settings
from logic.navigator import Navigator
from logic.state_machine import StateMachine
from vision.vision_system import VisionSystem
from utils.flight_logger import FlightLogger

MISSION_KEYS = {
    "loop_hz", "takeoff_extra_cm", "min_battery_takeoff", "min_battery_flight",
    "search_predict_s", "search_tof_cm", "search_climb_rc",
    "search_hold_kp", "search_hold_kv", "search_hold_max_rc",
    "search_spiral_after_s", "search_spiral_step_cm", "search_spiral_dwell_s", "search_spiral_max_cm",
    "track_tof_cm", "alt_hold_kp", "alt_hold_max_rc", "alt_hold_deadband_cm", "sink_comp_rc",
    "descent_rc_fast", "descent_rc_slow", "descent_slow_tof_cm", "descent_gate_cm", "blind_descent_rc",
    "reversal_gate", "commit_tof_cm", "final_phase_s", "reversal_settle_s",
    "require_period_if_moving", "max_gate_wait_s", "final_mode", "dry_run",
}


def test_settings_build_every_module():
    s = load_settings()
    Navigator(**s["navigator"])
    StateMachine(**s["fsm"])
    VisionSystem(**s["vision"])
    FlightLogger(**dict(s["logging"], enabled=False))


def test_comments_are_stripped():
    s = load_settings()
    assert not any(k.startswith("_") for k in s["vision"])


def test_mission_keys_complete():
    m = load_settings()["mission"]
    assert MISSION_KEYS <= set(m), f"Faltan: {MISSION_KEYS - set(m)}"
    assert m["final_mode"] in ("emergency", "land")


def test_hold_heights_stay_out_of_blind_zone():
    """La altura a la que se espera ventana no puede caer en la zona de punto ciego:
    perder el ArUco ahí (p. ej. por un cambio de sentido) se confundiría con el punto ciego."""
    s = load_settings()
    m, f = s["mission"], s["fsm"]
    lowest_hold = m["commit_tof_cm"] - m["alt_hold_deadband_cm"]
    assert lowest_hold > f["blind_tof_cm"] + 5
    assert m["track_tof_cm"] > m["commit_tof_cm"]
    assert f["touchdown_tof_cm"] < f["blind_tof_cm"]
