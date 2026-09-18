"""
Motor de inferencia difusa Mamdani para la sintonización dinámica de ganancias (Fuzzy Self-Tuning).
Utiliza scikit-fuzzy para evaluar variaciones relativas ΔKp, ΔKi, ΔKd a partir del error y su derivada.
"""

import json
import logging
import numpy as np
from typing import Dict, Tuple

try:
    import skfuzzy as fuzz
    from skfuzzy import control as ctrl
    HAS_SKFUZZY = True
except ImportError:
    HAS_SKFUZZY = False

logger = logging.getLogger(__name__)

class FuzzyTuner:
    """
    Sistema de Inferencia Difusa (FIS) Mamdani para sintonizar ganancias PID en tiempo real.
    Carga reglas y funciones de membresía desde un archivo JSON configurable.
    """

    def __init__(self, config_filepath: str):
        self.config_filepath = config_filepath
        self.output_scaling_pct = 0.25
        self.simulation = None
        self._load_and_build_fis()

    def _load_and_build_fis(self) -> None:
        """Carga la configuración de reglas y construye el sistema en scikit-fuzzy."""
        with open(self.config_filepath, "r", encoding="utf-8") as f:
            cfg = json.load(f)

        self.output_scaling_pct = cfg.get("output_scaling_pct", 0.25)

        if not HAS_SKFUZZY:
            logger.warning("scikit-fuzzy no instalado. Usando fallback proporcional simple.")
            return

        # Definir Universos de Discurso
        err_u = cfg["inputs"]["error"]["universe"]
        derr_u = cfg["inputs"]["delta_error"]["universe"]
        dkp_u = cfg["outputs"]["delta_kp"]["universe"]
        dki_u = cfg["outputs"]["delta_ki"]["universe"]
        dkd_u = cfg["outputs"]["delta_kd"]["universe"]

        error_var = ctrl.Antecedent(np.linspace(err_u[0], err_u[1], 101), "error")
        delta_error_var = ctrl.Antecedent(np.linspace(derr_u[0], derr_u[1], 101), "delta_error")

        delta_kp_var = ctrl.Consequent(np.linspace(dkp_u[0], dkp_u[1], 101), "delta_kp")
        delta_ki_var = ctrl.Consequent(np.linspace(dki_u[0], dki_u[1], 101), "delta_ki")
        delta_kd_var = ctrl.Consequent(np.linspace(dkd_u[0], dkd_u[1], 101), "delta_kd")

        # Funciones de Membresía Triangulares (7 términos: NB, NM, NS, Z, PS, PM, PB)
        terms = ["NB", "NM", "NS", "Z", "PS", "PM", "PB"]

        for var in [error_var, delta_error_var, delta_kp_var, delta_ki_var, delta_kd_var]:
            var["NB"] = fuzz.trimf(var.universe, [-1.0, -1.0, -0.66])
            var["NM"] = fuzz.trimf(var.universe, [-1.0, -0.66, -0.33])
            var["NS"] = fuzz.trimf(var.universe, [-0.66, -0.33, 0.0])
            var["Z"]  = fuzz.trimf(var.universe, [-0.33, 0.0, 0.33])
            var["PS"] = fuzz.trimf(var.universe, [0.0, 0.33, 0.66])
            var["PM"] = fuzz.trimf(var.universe, [0.33, 0.66, 1.0])
            var["PB"] = fuzz.trimf(var.universe, [0.66, 1.0, 1.0])

        # Definición de términos de entrada de 5 niveles para matriz 5x5
        in_terms = ["NB", "NS", "Z", "PS", "PB"]

        # Construcción de Reglas
        rule_list = []
        rules_cfg = cfg.get("rules", {})

        for out_name, out_var in [("delta_kp", delta_kp_var), ("delta_ki", delta_ki_var), ("delta_kd", delta_kd_var)]:
            matrix = rules_cfg.get(out_name, [])
            for i, e_term in enumerate(in_terms):
                for j, de_term in enumerate(in_terms):
                    if i < len(matrix) and j < len(matrix[i]):
                        target_term = matrix[i][j]
                        rule = ctrl.Rule(error_var[e_term] & delta_error_var[de_term], out_var[target_term])
                        rule_list.append(rule)

        fuzzy_system = ctrl.ControlSystem(rule_list)
        self.simulation = ctrl.ControlSystemSimulation(fuzzy_system)

    def evaluate(self, error: float, delta_error: float) -> Tuple[float, float, float]:
        """
        Evalúa el error y su derivada para obtener multiplicadores escalados ΔKp, ΔKi, ΔKd.
        Retorna multiplicadores escalados en el rango [-output_scaling_pct, +output_scaling_pct].
        """
        # Normalización saturada de entradas
        e_norm = max(-1.0, min(1.0, float(error)))
        de_norm = max(-1.0, min(1.0, float(delta_error)))

        if not HAS_SKFUZZY or self.simulation is None:
            # Fallback simple si skfuzzy no está disponible
            dkp = e_norm * 0.1
            dki = 0.0
            dkd = -de_norm * 0.1
            return dkp, dki, dkd

        try:
            self.simulation.input["error"] = e_norm
            self.simulation.input["delta_error"] = de_norm
            self.simulation.compute()

            raw_dkp = self.simulation.output.get("delta_kp", 0.0)
            raw_dki = self.simulation.output.get("delta_ki", 0.0)
            raw_dkd = self.simulation.output.get("delta_kd", 0.0)

            # Escalamiento al rango ±25% (u otro porcentaje configurado)
            dkp = raw_dkp * self.output_scaling_pct
            dki = raw_dki * self.output_scaling_pct
            dkd = raw_dkd * self.output_scaling_pct

            return dkp, dki, dkd
        except Exception as e:
            logger.error(f"Error evaluando FIS Mamdani: {e}")
            return 0.0, 0.0, 0.0
