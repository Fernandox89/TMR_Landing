"""
Pruebas de la misión completa en el simulador (tools/sim_mission.py).
Usan el mismo Navigator, StateMachine y compute_commands que main.py,
así que detectan errores de signos o de lógica entre módulos.
"""
import contextlib
import io

import pytest
from tools.sim_mission import simulate


def run(**kw):
    with contextlib.redirect_stdout(io.StringIO()):
        return simulate(duration=90, **kw)


@pytest.mark.parametrize("seed", range(3))
def test_lands_on_static_platform(seed):
    r = run(platform_speed=0, seed=seed)
    assert r["result"] == "landed"
    assert r["landing_error_cm"] < 5


# Con el modelo medido la bajada final tarda ~4 s: con tramos de 3 s + 1 s de pausa el carro
# cambia de sentido cada 4 s y no hay ventana segura. Por eso se prueba con tramos de 5 y 10 s.
@pytest.mark.parametrize("speed,leg", [(10, 5.0), (20, 5.0), (20, 10.0)])
def test_lands_on_moving_platform(speed, leg):
    for seed in range(3):
        r = run(platform_speed=speed, platform_leg_s=leg, seed=seed, platform_t0=seed * 0.7)
        assert r["result"] == "landed"
        assert r["landing_error_cm"] < 10, f"seed={seed}"


def test_platform_moving_along_x_axis():
    r = run(platform_speed=15, platform_axis="x", platform_leg_s=5.0)
    assert r["result"] == "landed"
    assert r["landing_error_cm"] < 10


def test_feedforward_reduces_tracking_error():
    def tracking_rms(ff):
        r = run(platform_speed=20, platform_leg_s=5.0, navigator_overrides={"feedforward_gain": ff})
        # Solo el seguimiento ya estable (sin el acercamiento inicial)
        rows = [h for h in r["history"] if h["state"] in ("SEGUIMIENTO", "DESCENSO") and h["t"] > 4]
        return (sum(h["p_y"] ** 2 for h in rows) / len(rows)) ** 0.5
    assert tracking_rms(0.6) < tracking_rms(0.0)
