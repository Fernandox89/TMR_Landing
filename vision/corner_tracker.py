"""
Seguimiento de las esquinas de un marcador por flujo óptico, para los cuadros en los que el
detector falla (un reflejo sobre el borde, desenfoque, una ráfaga de humo).

No sabe nada de ArUco ni del dron: recibe imágenes en gris y 4 esquinas, y regresa 4 esquinas.

Cómo funciona:
  1. reset(): con una detección buena se guardan la imagen, las 4 esquinas y ~40 puntos con
     textura DENTRO del marcador (esquinas entre celdas blancas y negras).
  2. track(): en cada cuadro sin detección esos puntos se siguen con Lucas-Kanade
     (cv2.calcOpticalFlowPyrLK) y se verifican de ida y vuelta. Con los que sobreviven se
     calcula una homografía con RANSAC (descarta los puntos tapados por el reflejo) desde la
     REFERENCIA hasta el cuadro actual, y con ella se mueven las 4 esquinas originales.
  3. Se rechaza el resultado si: quedan pocos puntos, la forma no es razonable (no convexa,
     lados muy desiguales, área muy distinta), las esquinas salen de la imagen, o pasó más de
     max_track_s desde la última detección real.
"""
import time

import cv2
import numpy as np


class CornerTracker:
    def __init__(self, max_track_s=1.0, fb_max_px=1.0, min_points=8, max_features=40,
                 min_area_ratio=0.6, max_area_ratio=1.6, min_side_ratio=0.5, clock=time.time):
        """
        max_track_s   : segundos máximos siguiendo sin una detección real.
        fb_max_px     : error máximo de ida y vuelta (px) para aceptar un punto.
        min_points    : puntos buenos mínimos (después de RANSAC) para aceptar el resultado.
        max_features  : cuántos puntos buscar dentro del marcador en cada detección.
        min/max_area_ratio : el área del marcador seguido respecto a la de la última detección.
        min_side_ratio: lado más corto / lado más largo mínimo (rechaza formas deformadas).
        """
        self.max_track_s = max_track_s
        self.fb_max_px = fb_max_px
        self.min_points = min_points
        self.max_features = max_features
        self.min_area_ratio = min_area_ratio
        self.max_area_ratio = max_area_ratio
        self.min_side_ratio = min_side_ratio
        self.clock = clock
        self.clear()

    def clear(self):
        """Olvida la referencia (por ejemplo, si cambió el marcador)."""
        self.prev_gray = None
        self.prev_pts = None      # dónde están los puntos en prev_gray (N, 1, 2)
        self.ref_pts = None       # dónde estaban en la imagen de referencia (N, 1, 2)
        self.ref_corners = None   # las 4 esquinas en la imagen de referencia (4, 2)
        self.ref_area = None
        self.ref_time = None

    @property
    def active(self):
        return self.prev_pts is not None

    def reset(self, gray, corners, now=None):
        """Detección real: nueva referencia. corners: (4, 2) en el orden del detector."""
        corners = np.asarray(corners, np.float32).reshape(4, 2)
        pts = self._features(gray, corners)
        if pts is None or len(pts) < self.min_points:
            self.clear()  # marcador muy chico o sin textura: no hay qué seguir
            return
        self.prev_gray = gray
        self.prev_pts = pts
        self.ref_pts = pts.copy()
        self.ref_corners = corners
        self.ref_area = abs(cv2.contourArea(corners))
        self.ref_time = self.clock() if now is None else now

    def track(self, gray, now=None):
        """Sin detección: dónde quedaron las 4 esquinas, (4, 2), o None si no es confiable."""
        if not self.active:
            return None
        now = self.clock() if now is None else now
        if now - self.ref_time > self.max_track_s:
            self.clear()
            return None

        lk = dict(winSize=(21, 21), maxLevel=3,
                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        nxt, st, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_pts, None, **lk)
        if nxt is None:
            return None
        # Ida y vuelta: un punto bien seguido debe regresar a donde estaba
        back, st2, _ = cv2.calcOpticalFlowPyrLK(gray, self.prev_gray, nxt, None, **lk)
        fb = np.linalg.norm((back - self.prev_pts).reshape(-1, 2), axis=1)
        ok = (st.ravel() == 1) & (st2.ravel() == 1) & (fb < self.fb_max_px)
        if ok.sum() < self.min_points:
            return None

        # Homografía referencia -> cuadro actual; RANSAC descarta los puntos tapados o mal seguidos
        H, inliers = cv2.findHomography(self.ref_pts[ok], nxt[ok], cv2.RANSAC, 2.0)
        if H is None or inliers is None or inliers.sum() < self.min_points:
            return None
        corners = cv2.perspectiveTransform(self.ref_corners.reshape(1, 4, 2), H).reshape(4, 2)
        if not self._sane(corners, gray.shape):
            return None

        # Seguir de cuadro en cuadro solo con los puntos buenos (la referencia no cambia)
        good = ok.copy()
        good[ok] = inliers.ravel().astype(bool)
        self.prev_gray = gray
        self.prev_pts = nxt[good].reshape(-1, 1, 2)
        self.ref_pts = self.ref_pts[good].reshape(-1, 1, 2)
        return corners

    # ------------------------------------------------------------------ auxiliares
    def _features(self, gray, corners):
        """Puntos con textura dentro del marcador (un poco agrandado para incluir el borde)."""
        c = corners.mean(axis=0)
        grown = c + (corners - c) * 1.1
        h, w = gray.shape[:2]
        x0, y0 = np.floor(grown.min(axis=0)).astype(int)
        x1, y1 = np.ceil(grown.max(axis=0)).astype(int) + 1
        x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, w), min(y1, h)
        if x1 - x0 < 8 or y1 - y0 < 8:
            return None
        mask = np.zeros((y1 - y0, x1 - x0), np.uint8)
        cv2.fillConvexPoly(mask, np.round(grown - [x0, y0]).astype(np.int32), 255)
        side = np.sqrt(abs(cv2.contourArea(corners)))
        pts = cv2.goodFeaturesToTrack(gray[y0:y1, x0:x1], maxCorners=self.max_features,
                                      qualityLevel=0.05, minDistance=max(3, side / 12), mask=mask)
        if pts is None:
            return None
        return (pts + np.array([x0, y0], np.float32)).astype(np.float32)

    def _sane(self, corners, shape):
        """¿Las esquinas forman un cuadrilátero razonable y están dentro de la imagen?"""
        h, w = shape[:2]
        if (corners < -2).any() or (corners[:, 0] > w + 2).any() or (corners[:, 1] > h + 2).any():
            return False
        if not cv2.isContourConvex(corners.reshape(-1, 1, 2).astype(np.float32)):
            return False
        area = abs(cv2.contourArea(corners))
        if not (self.min_area_ratio * self.ref_area <= area <= self.max_area_ratio * self.ref_area):
            return False
        sides = np.linalg.norm(corners - np.roll(corners, 1, axis=0), axis=1)
        return sides.min() >= self.min_side_ratio * sides.max()
