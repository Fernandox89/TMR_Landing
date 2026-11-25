import json
import os

DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "settings.json")


def _strip_comments(node):
    """Quita recursivamente las llaves que empiezan con '_' (se usan como comentarios en el JSON)."""
    if isinstance(node, dict):
        return {k: _strip_comments(v) for k, v in node.items() if not k.startswith("_")}
    if isinstance(node, list):
        return [_strip_comments(v) for v in node]
    return node


def load_settings(path=None):
    """Lee config/settings.json (o la ruta indicada) y regresa un diccionario sin comentarios."""
    path = path or DEFAULT_PATH
    with open(path, "r", encoding="utf-8") as f:
        return _strip_comments(json.load(f))


def save_camera_calibration(camera_matrix, dist_coeffs, path=None):
    """Escribe la calibracion de la camara en settings.json conservando el resto del archivo."""
    path = path or DEFAULT_PATH
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    raw["vision"]["camera_matrix"] = [[float(v) for v in row] for row in camera_matrix]
    raw["vision"]["dist_coeffs"] = [float(v) for v in dist_coeffs]
    with open(path, "w", encoding="utf-8") as f:
        json.dump(raw, f, indent=2, ensure_ascii=False)
        f.write("\n")
