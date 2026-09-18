"""
Sombra digital (Digital Shadow) para simulación en tiempo real paralela al vuelo real.
Alimentada por las mismas señales de control en cada ciclo para calcular el residuo normalizado.
"""

from typing import Dict, Tuple
from core.dynamics_model import FOPDTDynamicsModel

class DigitalShadow:
    """
    Simulación paralela no retroalimentada (flujo unidireccional real -> virtual).
    Calcula vgx_shadow y el residuo (vgx_real - vgx_shadow) en cada instante de muestreo.
    """

    def __init__(self, dynamics_model: FOPDTDynamicsModel):
        self.model = dynamics_model
        self.v_shadow = 0.0

    def reset(self, initial_velocity: float = 0.0) -> None:
        """Reinicia la sombra digital."""
        self.v_shadow = float(initial_velocity)
        self.model.reset(initial_velocity)

    def step(self, control_signal: float, vgx_real: float, dt: float) -> Tuple[float, float]:
        """
        Calcula un paso de la sombra digital y retorna (vgx_shadow, shadow_residual).
        """
        self.v_shadow = self.model.step(control_signal, dt)
        residual = float(vgx_real) - self.v_shadow
        return self.v_shadow, residual
