"""
Orquestador de campañas experimentales por bloques aleatorizados.
Gestiona repeticiones por celda (controlador x escenario), seguimiento de progreso y pausa/reanudación.
"""

import random
import logging
from typing import Dict, Any, List, Optional
from missions.mission_a_to_b import MissionAToB

logger = logging.getLogger(__name__)

class ExperimentRunner:
    """
    Ejecutor de campañas científicas estructuradas por bloques aleatorizados.
    Garantiza que ambos controladores se prueben en orden aleatorio dentro de cada sesión.
    """

    def __init__(self, mission_runner: MissionAToB, target_repetitions_per_cell: int = 25):
        self.mission_runner = mission_runner
        self.target_repetitions = target_repetitions_per_cell
        self.is_paused = False

        # Estado de avance: celdas (controller, scenario) -> cantidad completada
        self.completed_counts: Dict[str, int] = {
            "pid_controlado": 0,
            "fuzzy_controlado": 0,
            "pid_viento_natural": 0,
            "fuzzy_viento_natural": 0,
            "pid_perturbacion_artificial": 0,
            "fuzzy_perturbacion_artificial": 0,
        }

    def pause_campaign(self) -> None:
        """Pausa la campaña activa al finalizar el vuelo en curso."""
        self.is_paused = True
        logger.info("Campaña pausada por el operador.")

    def resume_campaign(self) -> None:
        """Reanuda la ejecución de la campaña."""
        self.is_paused = False
        logger.info("Campaña reanudada.")

    def run_session_block(self, session_id: str, scenario: str, wind_speed: Optional[float] = None) -> List[Dict[str, Any]]:
        """
        Ejecuta un bloque de sesión para un escenario dado.
        Ordena aleatoriamente la pareja (PID, Fuzzy Self-Tuning) para evitar sesgo de secuencia.
        """
        controllers = ["pid", "fuzzy_self_tuning"]
        random.shuffle(controllers)

        results = []
        for ctrl in controllers:
            if self.is_paused:
                logger.info("Campaña en pausa. Cancelando siguiente vuelo del bloque.")
                break

            cell_key = f"{'pid' if ctrl == 'pid' else 'fuzzy'}_{scenario}"
            current_rep = self.completed_counts.get(cell_key, 0) + 1

            if current_rep > self.target_repetitions:
                logger.info(f"Celda {cell_key} ya completó la meta de {self.target_repetitions} repeticiones.")
                continue

            logger.info(f"Ejecutando Vuelo: Sesión={session_id}, Controlador={ctrl}, Escenario={scenario}, Repetición={current_rep}")
            res = self.mission_runner.execute_mission(
                controller_type=ctrl,
                scenario=scenario,
                session_id=session_id,
                repetition_number=current_rep,
                wind_measured=wind_speed
            )

            if res.get("status") == "completado":
                self.completed_counts[cell_key] = current_rep

            results.append(res)

        return results
