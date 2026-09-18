"""
Modelo matemático dinámico FOPDT (First Order Plus Dead Time).
Implementa la discretización por Retención de Orden Cero (ZOH) con buffer circular de retardo.
"""

import json
import numpy as np
from collections import deque
from typing import Dict, Any

class FOPDTDynamicsModel:
    """
    Modelo dinámico continuo de primer orden con tiempo muerto:
    V(s) / U(s) = K * e^(-L*s) / (tau * s + 1)

    Discretizado mediante ZOH:
    v[k] = a * v[k-1] + K * (1 - a) * u[k - N_d]
    donde a = exp(-Ts / tau), N_d = round(L / Ts)
    """

    def __init__(self, config_filepath: str):
        self.config_filepath = config_filepath
        self.K = 1.0
        self.L = 0.0
        self.tau = 1.0

        self.v_prev = 0.0
        self.delay_buffer = deque()
        self.load_params()

    def load_params(self) -> None:
        """Carga parámetros K, L, tau desde archivo de configuración JSON."""
        with open(self.config_filepath, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        self.K = float(cfg.get("K", 1.0))
        self.L = float(cfg.get("L", 0.0))
        self.tau = float(cfg.get("tau", 1.0))

    def reset(self, initial_velocity: float = 0.0) -> None:
        """Reinicia el estado del modelo dinámico."""
        self.v_prev = float(initial_velocity)
        self.delay_buffer.clear()

    def step(self, control_signal: float, dt: float) -> float:
        """
        Ejecuta un paso de simulación ZOH dado el comando de control actual y dt.
        """
        if dt <= 0.0:
            dt = 0.1

        tau_safe = max(0.01, self.tau)
        a = np.exp(-dt / tau_safe)

        # Determinar número de muestras de retardo N_d
        N_d = max(0, int(round(self.L / dt)))

        # Actualizar buffer circular
        self.delay_buffer.append(float(control_signal))

        if len(self.delay_buffer) <= N_d:
            u_delayed = 0.0
        else:
            # Obtener el comando enviado N_d muestras atrás
            u_delayed = self.delay_buffer[-1 - N_d]
            # Mantener tamaño del buffer acotado
            while len(self.delay_buffer) > (N_d + 10):
                self.delay_buffer.popleft()

        # Ecuación en diferencias ZOH
        v_k = a * self.v_prev + self.K * (1.0 - a) * u_delayed
        self.v_prev = v_k

        return float(v_k)
