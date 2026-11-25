"""Funciones auxiliares de main.py: altura, posición sin ArUco y espiral de búsqueda."""
from config.loader import load_settings
from logic.navigator import Navigator
from main import hold_altitude, hold_position, new_context, search_target, square_spiral
from conftest import FakeClock

M = load_settings()["mission"]


def test_square_spiral_covers_grid_without_repeating():
    pts = square_spiral(25)
    assert pts[0] == (0, 0)
    assert len(set(pts)) == 25
    assert all(abs(x) <= 2 and abs(y) <= 2 for x, y in pts)
    # cada paso es a un vecino
    assert all(abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1 for a, b in zip(pts, pts[1:]))


def test_hold_altitude_direction_and_deadband():
    assert hold_altitude(80, 110, M) > 0          # abajo del objetivo -> subir
    assert hold_altitude(140, 110, M) < 0         # arriba -> bajar
    assert hold_altitude(111, 110, M) == 0        # dentro de la zona muerta
    assert abs(hold_altitude(0, 1000, M)) <= M["alt_hold_max_rc"]


def test_hold_position_brakes_drift():
    nav = Navigator(**load_settings()["navigator"])
    ctx = new_context()
    # derivando a la derecha (vgy=+1 dm/s) y hacia adelante (vgx=+1) -> corregir izquierda y atrás
    roll, pitch = hold_position({"vel_x": 1, "vel_y": 1}, nav, M, 0.033, ctx)
    assert roll < 0 and pitch < 0
    # después de alejarse 50 cm a la izquierda, regresar a la derecha
    ctx["search_disp"] = [-50.0, 0.0]
    roll, _ = hold_position({"vel_x": 0, "vel_y": 0}, nav, M, 0.033, ctx)
    assert roll > 0


def test_search_target_waits_then_spirals():
    clock = FakeClock()
    nav = Navigator(**load_settings()["navigator"], clock=clock)
    ctx = new_context()
    assert search_target(ctx, nav, M) == (0.0, 0.0)
    clock.advance(M["search_spiral_after_s"] + M["search_spiral_dwell_s"] + 0.1)
    assert search_target(ctx, nav, M) == (M["search_spiral_step_cm"], 0)
