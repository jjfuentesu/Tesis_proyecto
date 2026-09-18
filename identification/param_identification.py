"""
Módulo de identificación no lineal de parámetros dinámicos FOPDT (K, L, tau).
Genera señales de prueba (escalón y barrido senoidal) y ajusta el modelo mediante mínimos cuadrados.
"""

import json
import logging
import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from typing import Dict, Tuple, Any

from core.dynamics_model import FOPDTDynamicsModel

logger = logging.getLogger(__name__)

class ParameterIdentification:
    """
    Identificador de parámetros dinámicos K, L, tau del dron a partir de series temporales de calibración.
    """

    @staticmethod
    def generate_step_signal(duration_s: float = 10.0, dt: float = 0.1, step_amplitude: float = 30.0, step_time_s: float = 2.0) -> Tuple[np.ndarray, np.ndarray]:
        """Genera una señal de entrada tipo escalón."""
        t = np.arange(0, duration_s, dt)
        u = np.where(t >= step_time_s, step_amplitude, 0.0)
        return t, u

    @staticmethod
    def fit_fopdt(t: np.ndarray, u: np.ndarray, v_real: np.ndarray) -> Tuple[Dict[str, float], float]:
        """
        Ajusta K, L, tau minimizando el error cuadrático entre v_real y la salida del modelo FOPDT.
        Retorna (parámetros_ajustados, r2_score).
        """
        def residuals(params):
            K, L, tau = params
            if tau <= 0.001 or K < 0 or L < 0:
                return np.full_like(v_real, 1e6)

            # Simular FOPDT con los parámetros propuestos
            v_sim = np.zeros_like(v_real)
            v_prev = 0.0
            dt = t[1] - t[0] if len(t) > 1 else 0.1
            tau_safe = max(0.01, tau)
            a = np.exp(-dt / tau_safe)
            N_d = max(0, int(round(L / dt)))

            for k in range(len(t)):
                u_delayed = u[k - N_d] if k >= N_d else 0.0
                v_k = a * v_prev + K * (1.0 - a) * u_delayed
                v_sim[k] = v_k
                v_prev = v_k

            return v_sim - v_real

        initial_guess = [1.0, 0.2, 0.8]
        bounds = ([0.0, 0.0, 0.01], [5.0, 2.0, 5.0])

        res = least_squares(residuals, initial_guess, bounds=bounds)
        fitted_K, fitted_L, fitted_tau = res.x

        # Calcular R²
        residuals_opt = res.fun
        ss_res = np.sum(residuals_opt ** 2)
        ss_tot = np.sum((v_real - np.mean(v_real)) ** 2)
        r2_score = float(1.0 - (ss_res / (ss_tot + 1e-8)))

        fitted_params = {
            "K": float(fitted_K),
            "L": float(fitted_L),
            "tau": float(fitted_tau),
            "goodness_of_fit_r2": max(0.0, float(r2_score))
        }

        return fitted_params, r2_score
