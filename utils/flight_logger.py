import csv
import os
import time

import cv2

COLUMNS = [
    "t", "dt", "state", "visible",
    "raw_x", "raw_y", "raw_z",          # PnP crudo (cm)
    "est_x", "est_y",                   # Kalman: posición estimada (cm)
    "vt_x", "vt_y",                     # Kalman: velocidad estimada de la plataforma (cm/s)
    "vel_x", "vel_y",                   # Velocidad CRUDA del Tello (sus unidades)
    "cmd_roll", "cmd_pitch", "cmd_throttle",
    "tof_cm", "height_cm", "battery", "flying",
]


class FlightLogger:
    """
    Guarda un CSV por vuelo (y opcionalmente el video con HUD) en logs/AAAAMMDD_HHMMSS.*
    El CSV se analiza con tools/analyze_log.py.
    """

    def __init__(self, enabled=True, directory="logs", record_video=False, video_fps=20):
        self.enabled = enabled
        self.record_video = record_video
        self.video_fps = video_fps
        self.writer = None
        self.video = None
        self.t0 = time.time()
        if not enabled:
            return
        if not os.path.isabs(directory):
            directory = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), directory)
        os.makedirs(directory, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.base_path = os.path.join(directory, stamp)
        self.csv_file = open(self.base_path + ".csv", "w", newline="")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=COLUMNS)
        self.writer.writeheader()
        print(f"[LOG] Guardando vuelo en {self.base_path}.csv")

    def log(self, **row):
        if not self.writer:
            return
        row["t"] = round(time.time() - self.t0, 4)
        self.writer.writerow({k: _fmt(row.get(k, "")) for k in COLUMNS})

    def write_frame(self, frame):
        if not (self.enabled and self.record_video) or frame is None:
            return
        if self.video is None:
            h, w = frame.shape[:2]
            self.video = cv2.VideoWriter(self.base_path + ".mp4", cv2.VideoWriter_fourcc(*"mp4v"),
                                         self.video_fps, (w, h))
        self.video.write(frame)

    def close(self):
        if self.writer:
            self.csv_file.close()
            self.writer = None
        if self.video is not None:
            self.video.release()
            self.video = None


def _fmt(v):
    if isinstance(v, float):
        return round(v, 3)
    if v is None:
        return ""
    return v
