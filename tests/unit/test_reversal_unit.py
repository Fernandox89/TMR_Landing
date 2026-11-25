from logic.reversal_predictor import ReversalPredictor
from conftest import FakeClock

DT = 0.05


def run(pred, clock, v, seconds):
    for _ in range(int(seconds / DT)):
        clock.advance(DT)
        pred.update(v, 0.0)


def test_learns_period_and_predicts_next():
    clock = FakeClock()
    pred = ReversalPredictor(min_speed_cm_s=5, min_interval_s=1, confirm_s=0.2, clock=clock)
    for _ in range(3):
        run(pred, clock, 20, 3); run(pred, clock, 0, 1)
        run(pred, clock, -20, 3); run(pred, clock, 0, 1)
    assert abs(pred.period - 4.0) < 0.2
    t_next = pred.time_to_next_reversal()
    assert t_next is not None and t_next < 4.0


def test_static_platform_is_always_safe():
    clock = FakeClock()
    pred = ReversalPredictor(clock=clock)
    run(pred, clock, 2, 5)                       # ruido por debajo de min_speed
    assert not pred.platform_moving
    assert pred.safe_window(1.5, 0.8)


def test_short_spikes_are_ignored():
    clock = FakeClock()
    pred = ReversalPredictor(confirm_s=0.4, clock=clock)
    run(pred, clock, 30, 0.2)
    run(pred, clock, 0, 1)
    assert not pred.platform_moving


def test_moving_without_period_waits():
    clock = FakeClock()
    pred = ReversalPredictor(confirm_s=0.2, clock=clock)
    run(pred, clock, 20, 2)
    assert pred.platform_moving
    assert not pred.safe_window(1.5, 0.8, require_period=True)
    assert pred.safe_window(1.5, 0.8, require_period=False)


def test_not_safe_right_after_reversal():
    clock = FakeClock()
    pred = ReversalPredictor(confirm_s=0.2, clock=clock)
    run(pred, clock, 20, 2)
    run(pred, clock, -20, 0.5)
    assert pred.time_since_reversal() < 0.8
    assert not pred.safe_window(1.5, 0.8, require_period=False)


def test_one_reversal_gives_conservative_window():
    clock = FakeClock()
    pred = ReversalPredictor(min_speed_cm_s=5, min_interval_s=1, confirm_s=0.2, clock=clock)
    run(pred, clock, 10, 12)          # 12 s en un sentido (vista completa desde que se movió)
    run(pred, clock, -10, 1.5)        # primer cambio de sentido, ya pasó 1.5 s
    assert pred.period is None
    assert pred.min_leg is not None and abs(pred.min_leg - 11.8) < 0.3
    # Quedan ~10 s antes del siguiente cambio: una maniobra de 4 s cabe
    assert pred.safe_window(4.0, 0.8, require_period=True)
    run(pred, clock, -10, 8)          # ya casi toca el siguiente cambio
    assert not pred.safe_window(4.0, 0.8, require_period=True)
