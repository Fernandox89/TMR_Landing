from logic.state_machine import StateMachine, MisionState as S
from conftest import FakeClock

DT = 0.05


def make():
    clock = FakeClock()
    fsm = StateMachine(takeoff_settle_s=1.0, visible_to_track_s=0.5, lost_to_search_s=2.0,
                       center_radius_cm=10, centered_to_descend_s=1.0,
                       descent_abort_radius_cm=30, descent_abort_s=0.5, descent_lost_abort_s=1.5,
                       blind_tof_cm=45, blind_timeout_s=4.0, blind_force_land_tof_cm=35,
                       touchdown_tof_cm=20, clock=clock)
    return fsm, clock


def step(fsm, clock, seconds, visible=True, err=(0.0, 0.0), tof=100):
    for _ in range(int(round(seconds / DT))):
        clock.advance(DT)
        pos = (err[0], err[1], tof) if visible else None
        fsm.update(visible, pos, tof, error_xy=err)
    return fsm.state


def test_full_happy_path():
    fsm, clock = make()
    fsm.set_state(S.DESPEGUE)
    assert step(fsm, clock, 1.2, visible=False) == S.BUSQUEDA
    assert step(fsm, clock, 0.7) == S.SEGUIMIENTO
    assert step(fsm, clock, 1.2) == S.DESCENSO
    assert step(fsm, clock, 0.2, visible=False, tof=40) == S.PUNTO_CIEGO
    assert step(fsm, clock, 0.2, visible=False, tof=18) == S.ATERRIZAJE


def test_off_center_does_not_descend():
    fsm, clock = make()
    fsm.set_state(S.SEGUIMIENTO)
    assert step(fsm, clock, 3.0, err=(25, 0)) == S.SEGUIMIENTO


def test_descent_aborts_when_drifting():
    fsm, clock = make()
    fsm.set_state(S.DESCENSO)
    assert step(fsm, clock, 0.3, err=(40, 0)) == S.DESCENSO      # aún no pasa el tiempo
    assert step(fsm, clock, 0.4, err=(40, 0)) == S.SEGUIMIENTO


def test_lost_high_returns_to_search():
    fsm, clock = make()
    fsm.set_state(S.DESCENSO)
    assert step(fsm, clock, 2.0, visible=False, tof=90) == S.BUSQUEDA


def test_blind_timeout_low_forces_landing():
    fsm, clock = make()
    fsm.set_state(S.PUNTO_CIEGO)
    assert step(fsm, clock, 4.2, visible=False, tof=30) == S.ATERRIZAJE


def test_blind_timeout_high_aborts():
    fsm, clock = make()
    fsm.set_state(S.PUNTO_CIEGO)
    assert step(fsm, clock, 4.2, visible=False, tof=42) == S.BUSQUEDA


def test_legacy_signature_still_works():
    fsm, clock = make()
    fsm.set_state(S.SEGUIMIENTO)
    clock.advance(0.1)
    assert fsm.update(True, (0.0, 0.0, 100.0), 100) == S.SEGUIMIENTO
