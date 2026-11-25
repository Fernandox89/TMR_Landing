import pytest
from control.kalman import KalmanObjetivo

DT = 1 / 30


def test_estimates_platform_velocity_with_static_drone():
    kf = KalmanObjetivo(std_acc_target=40, std_pos_process=5, std_vision=1.0)
    kf.reset(p=0.0)
    p = 0.0
    for _ in range(150):                 # 5 s, plataforma a 20 cm/s, dron quieto
        p += 20 * DT
        kf.predict(0.0, DT)
        kf.update(p)
    assert kf.target_velocity == pytest.approx(20, abs=2)
    assert kf.position == pytest.approx(p, abs=1.5)


def test_drone_motion_does_not_look_like_platform_motion():
    kf = KalmanObjetivo()
    kf.reset(p=50.0)
    p = 50.0
    for _ in range(150):                 # plataforma quieta, dron avanzando a 15 cm/s
        p -= 15 * DT
        kf.predict(15.0, DT)
        kf.update(p)
    assert abs(kf.target_velocity) < 2


def test_prediction_without_vision_extrapolates():
    kf = KalmanObjetivo()
    kf.reset(p=0.0, v_obj=10.0)
    kf.P[:] = 0                          # certeza total para probar solo el modelo
    for _ in range(30):
        kf.predict(0.0, DT)
    assert kf.position == pytest.approx(10.0, abs=0.01)


def test_decay_and_clamp():
    kf = KalmanObjetivo()
    kf.reset(v_obj=100.0)
    kf.clamp_target_velocity(80)
    assert kf.target_velocity == 80
    kf.decay_target_velocity(0.5)
    assert kf.target_velocity == 40
