"""
Seguimiento de esquinas por flujo óptico (vision/corner_tracker.py), con imágenes sintéticas:
un ArUco que se mueve un poco entre cuadros y, en el segundo, un reflejo blanco sobre su borde
inferior (como el de la tarjeta enmicada del laboratorio), que hace fallar al detector.
"""
import cv2
import numpy as np
import pytest

from conftest import FakeClock
from vision.aruco_tracker import ArucoTracker
from vision.corner_tracker import CornerTracker

DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
SIDE = 200  # px del lado negro del marcador en la imagen


def scene(center=(480, 360), scale=1.0, glare=False, marker=True):
    """Imagen gris 960x720: fondo con textura suave y el ArUco (con margen blanco) en center.
    Regresa (imagen, esquinas verdaderas del cuadro negro en orden del detector)."""
    rng = np.random.default_rng(0)
    img = cv2.GaussianBlur(rng.integers(90, 130, (720, 960)).astype(np.uint8), (0, 0), 3)
    s = SIDE * scale
    cx, cy = center
    corners = np.float32([[cx - s / 2, cy - s / 2], [cx + s / 2, cy - s / 2],
                          [cx + s / 2, cy + s / 2], [cx - s / 2, cy + s / 2]])
    if marker:
        m = cv2.aruco.generateImageMarker(DICT, 0, 400)
        card = cv2.copyMakeBorder(m, 50, 50, 50, 50, cv2.BORDER_CONSTANT, value=255)  # margen blanco
        src = np.float32([[50, 50], [450, 50], [450, 450], [50, 450]])
        H = cv2.getPerspectiveTransform(src, corners)
        warped = cv2.warpPerspective(card, H, (960, 720), borderValue=0)
        mask = cv2.warpPerspective(np.full_like(card, 255), H, (960, 720))
        img[mask > 0] = warped[mask > 0]
    if glare:  # mancha blanca sobre el borde negro inferior, como en la foto del laboratorio
        y, x = np.mgrid[0:720, 0:960]
        bx, by = cx, cy + 0.5 * s  # sobre la orilla exterior del borde negro
        blob = np.exp(-((x - bx) ** 2 + (y - by) ** 2) / (2 * (0.12 * s) ** 2))
        img = np.clip(img + blob * (255 - img.astype(float)) * 0.95, 0, 255).astype(np.uint8)
    return img, corners


def detect(gray):
    corners, ids, _ = cv2.aruco.ArucoDetector(DICT, cv2.aruco.DetectorParameters()).detectMarkers(gray)
    return None if ids is None else corners[0][0]


def test_scene_reproduces_the_problem():
    """Sin reflejo el detector lo encuentra; con el reflejo sobre el borde, no."""
    assert detect(scene()[0]) is not None
    assert detect(scene(center=(490, 366), glare=True)[0]) is None


def test_tracks_through_glare():
    clock = FakeClock()
    tr = CornerTracker(clock=clock)
    g0, _ = scene()
    tr.reset(g0, detect(g0))
    assert tr.active
    clock.advance(0.03)
    g1, truth = scene(center=(490, 366), glare=True)
    got = tr.track(g1)
    assert got is not None
    assert np.abs(got - truth).max() < 1.5        # las 4 esquinas, a menos de 1.5 px


def test_tracks_a_few_frames_with_scale_change():
    clock = FakeClock()
    tr = CornerTracker(clock=clock)
    g, _ = scene()
    tr.reset(g, detect(g))
    for k in range(1, 6):                          # se acerca: el marcador crece 3 % por cuadro
        clock.advance(0.03)
        g, truth = scene(center=(480 + 3 * k, 360 + 2 * k), scale=1 + 0.03 * k, glare=True)
        got = tr.track(g)
        assert got is not None and np.abs(got - truth).max() < 2.0, k


def test_does_not_invent_a_marker():
    clock = FakeClock()
    tr = CornerTracker(clock=clock)
    g, _ = scene()
    tr.reset(g, detect(g))
    clock.advance(0.03)
    assert tr.track(scene(marker=False)[0]) is None


def test_gives_up_after_max_track_s():
    clock = FakeClock()
    tr = CornerTracker(max_track_s=1.0, clock=clock)
    g, _ = scene()
    tr.reset(g, detect(g))
    clock.advance(1.2)
    assert tr.track(scene(center=(485, 362), glare=True)[0]) is None
    assert not tr.active


def test_aruco_tracker_uses_it_only_when_enabled():
    """Integrado en ArucoTracker: con reflejo da la posición (source "tracked") a ~1 cm de la
    verdadera; apagado, no da nada."""
    frames = [cv2.cvtColor(scene()[0], cv2.COLOR_GRAY2BGR),
              cv2.cvtColor(scene(center=(490, 366), glare=True)[0], cv2.COLOR_GRAY2BGR)]
    truth = ArucoTracker(marker_size_cm=10).detect_and_estimate(
        cv2.cvtColor(scene(center=(490, 366))[0], cv2.COLOR_GRAY2BGR))[2]

    on = ArucoTracker(marker_size_cm=10, corner_tracking={"enabled": True})
    assert on.detect_and_estimate(frames[0].copy())[2] is not None and on.last_source == "detected"
    pos = on.detect_and_estimate(frames[1].copy())[2]
    assert on.last_source == "tracked"
    assert pos == pytest.approx(truth, abs=1.0)

    off = ArucoTracker(marker_size_cm=10, corner_tracking={"enabled": False})
    off.detect_and_estimate(frames[0].copy())
    assert off.detect_and_estimate(frames[1].copy())[2] is None and off.last_source is None
