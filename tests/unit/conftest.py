import os
import sys

# Permite importar los módulos del proyecto (vision, control, logic, ...) desde las pruebas
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))


class FakeClock:
    """Reloj controlable para probar lógica que depende del tiempo sin esperar de verdad."""

    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt
