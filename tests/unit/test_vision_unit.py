"""Prueba la visión con imágenes sintéticas (sin dron)."""
import cv2
import numpy as np
import pytest

from vision.aruco_tracker import ArucoTracker
from vision.vision_system import VisionSystem


def synthetic_frame(marker_id=0, size_px=200, center=(480, 360), dictionary=cv2.aruco.DICT_6X6_100):
    d = cv2.aruco.getPredefinedDictionary(dictionary)
    marker = cv2.aruco.generateImageMarker(d, marker_id, size_px)
    frame = np.full((720, 960), 255, np.uint8)
    x0, y0 = center[0] - size_px // 2, center[1] - size_px // 2
    frame[y0:y0 + size_px, x0:x0 + size_px] = marker
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)


def test_centered_marker_distance():
    _, ids, pos = ArucoTracker(marker_size_cm=10.0).detect_and_estimate(synthetic_frame())
    assert pos is not None
    x, y, z = pos
    assert abs(x) < 0.5 and abs(y) < 0.5
    # Modelo pinhole: z = f * tamaño_real / tamaño_px = 920 * 10 / 200 = 46 cm
    assert z == pytest.approx(46, abs=1.5)


def test_offset_marker_sign():
    _, _, pos = ArucoTracker().detect_and_estimate(synthetic_frame(center=(680, 360)))
    assert pos[0] > 5                    # a la derecha en la imagen -> X positiva


def test_marker_id_filter():
    tracker = ArucoTracker(marker_id=3)
    assert tracker.detect_and_estimate(synthetic_frame(marker_id=0))[2] is None
    assert tracker.detect_and_estimate(synthetic_frame(marker_id=3))[2] is not None


def test_per_id_marker_size():
    tracker = ArucoTracker(marker_size_cm=10.0, marker_sizes_cm={"0": 20.0})
    z = tracker.detect_and_estimate(synthetic_frame())[2][2]
    assert z == pytest.approx(92, abs=3)


def test_vision_system_undoes_mirror():
    # Lo que ve la cámara a través del espejo es la imagen volteada; flip_code=1 la endereza.
    # VisionSystem además convierte RGB->BGR (formato de djitellopy), así que le damos RGB.
    # ArUco real a la DERECHA (x > 0):
    mirrored = cv2.flip(cv2.cvtColor(synthetic_frame(center=(680, 360)), cv2.COLOR_BGR2RGB), 1)
    _, _, pos = VisionSystem(flip_code=1).update(mirrored)
    assert pos is not None and pos[0] > 5

    # OJO: con flip_code=0 el ArUco TAMBIÉN se detecta (queda girado 180°, que es válido),
    # pero los signos de X e Y salen invertidos. Por eso flip_code y axis_sign van juntos.
    _, _, pos_wrong = VisionSystem(flip_code=0).update(mirrored)
    assert pos_wrong is not None and pos_wrong[0] < -5


def test_empty_frame():
    assert VisionSystem().update(None) == (None, None, None)
