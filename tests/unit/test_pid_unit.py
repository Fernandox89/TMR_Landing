import pytest
from control.pid_controller import PIDController
from conftest import FakeClock


def make(**kw):
    args = dict(kp=2.0, ki=0.0, kd=0.0, min_out=-50, max_out=50)
    args.update(kw)
    return PIDController(**args, clock=FakeClock())


def test_proportional_sign():
    pid = make()
    # error = setpoint - valor = 0 - 10 = -10 -> salida -20
    assert pid.update(0, 10, dt=0.1) == -20


def test_output_clamped():
    assert make().update(0, 1000, dt=0.1) == -50


def test_no_derivative_kick_after_reset():
    pid = make(kp=0.0, kd=1.0)
    assert pid.update(0, 30, dt=0.1) == 0       # primera llamada: sin derivada
    assert pid.update(0, 31, dt=0.1) == -10     # -kd * (31-30)/0.1


def test_feedforward_added_and_clamped():
    pid = make(kp=0.0)
    assert pid.update(0, 0, dt=0.1, feedforward=12.0) == 12
    assert pid.update(0, 0, dt=0.1, feedforward=99.0) == 50


def test_integral_anti_windup():
    pid = make(kp=0.0, ki=10.0, max_out=20, min_out=-20)
    for _ in range(100):
        pid.update(0, -5, dt=0.1)               # error positivo constante -> satura
    # La integral no siguió creciendo: al invertir el error la salida baja rápido
    assert pid.integral <= 2.0 + 0.5 + 1e-9
    pid.reset()
    assert pid.integral == 0.0


def test_dt_from_clock():
    clock = FakeClock()
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, min_out=-100, max_out=100, clock=clock)
    clock.advance(0.1)
    pid.update(0, -10)
    assert pid.integral == pytest.approx(1.0)
