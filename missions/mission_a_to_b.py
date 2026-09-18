"""
Orquestador de misión de vuelo perfil de velocidad A -> B.
Ejecuta el lazo de control principal, simulación paralela de sombra digital y chequeos de seguridad RNF8.
"""

import time
import uuid
import logging
from datetime import datetime
from typing import Dict, Any, Optional

from core.tello_interface import TelloInterface
from core.pid_controller import PIDController
from core.fuzzy_tuner import FuzzyTuner
from core.digital_shadow import DigitalShadow
from data.data_logger import DataLogger

logger = logging.getLogger(__name__)

class MissionAToB:
    """
    Controlador de lazo cerrado para perfil de velocidad constante A -> B.
    Integra seguridad operativa en tiempo real (batería, timeout, parada manual).
    """

    def __init__(self, tello: TelloInterface, pid: PIDController, fuzzy_tuner: Optional[FuzzyTuner], digital_shadow: DigitalShadow, logger_inst: DataLogger, flight_config: Dict[str, Any]):
        self.tello = tello
        self.pid = pid
        self.fuzzy_tuner = fuzzy_tuner
        self.shadow = digital_shadow
        self.data_logger = logger_inst
        self.config = flight_config

        self.abort_requested = False
        self.abort_reason: Optional[str] = None

    def request_emergency_abort(self, reason: str = "parada_manual_operador") -> None:
        """Solicita la parada de emergencia desde la GUI o hilo externo."""
        self.abort_requested = True
        self.abort_reason = reason
        logger.warning(f"Solicitud de parada de emergencia: {reason}")

    def execute_mission(self, controller_type: str = "pid", scenario: str = "controlado", session_id: str = "default_session", repetition_number: int = 1, wind_measured: Optional[float] = None) -> Dict[str, Any]:
        """
        Ejecuta la misión A->B completa.
        Retorna resumen del vuelo y estado de finalización.
        """
        flight_id = str(uuid.uuid4())[:8]
        setpoint = float(self.config.get("setpoint_vgx", 30.0))
        duration = float(self.config.get("mission_duration_s", 8.0))
        target_hz = float(self.config.get("sampling_rate_hz", 10.0))
        dt_target = 1.0 / target_hz

        safety_limits = self.config.get("safety_limits", {})
        min_bat = float(safety_limits.get("battery_min_pct", 15.0))
        telemetry_timeout = float(safety_limits.get("telemetry_timeout_s", 2.0))

        # Bloqueo preventivo por viento en escenario viento_natural
        if scenario == "viento_natural" and wind_measured is not None and wind_measured > 5.0:
            raise ValueError(f"Velocidad de viento medida ({wind_measured} m/s) excede el límite operativo (5.0 m/s). Despegue bloqueado.")

        # Iniciar logger y reiniciar controladores
        self.pid.reset()
        if self.fuzzy_tuner:
            pass  # El FIS no almacena estado previo
        self.shadow.reset(0.0)
        self.abort_requested = False
        self.abort_reason = None

        initial_telemetry = self.tello.get_telemetry()
        bat_start = float(initial_telemetry.get("bat", 100.0))

        metadata = {
            "flight_id": flight_id,
            "session_id": session_id,
            "controller_type": controller_type,
            "scenario": scenario,
            "repetition_number": repetition_number,
            "setpoint_vgx": setpoint,
            "mission_duration_s": duration,
            "battery_pct_start": bat_start,
            "wind_speed_measured": wind_measured
        }
        self.data_logger.start_flight_log(flight_id, metadata)

        t_start = time.time()
        t_prev = t_start
        status = "completado"

        try:
            while True:
                t_now = time.time()
                t_rel = t_now - t_start

                if t_rel >= duration or self.abort_requested:
                    if self.abort_requested:
                        status = f"abortado_{self.abort_reason}"
                    break

                dt = max(0.01, t_now - t_prev)
                t_prev = t_now

                # Read telemetry
                telemetry = self.tello.get_telemetry()
                vgx_real = float(telemetry.get("vgx", 0.0))
                bat_current = float(telemetry.get("bat", 100.0))

                # RNF8 Safety Checks
                if bat_current < min_bat:
                    self.request_emergency_abort("bateria_baja")
                    status = "abortado_bateria"
                    break

                if (t_now - self.tello.last_telemetry_time) > telemetry_timeout:
                    self.request_emergency_abort("timeout_telemetria")
                    status = "abortado_telemetria"
                    break

                # Control Calculation
                error = setpoint - vgx_real

                if controller_type == "fuzzy_self_tuning" and self.fuzzy_tuner:
                    # Calcular derivada de error por diferencia finita
                    delta_e = (error - self.pid.prev_error) / dt if not self.pid.is_first_step else 0.0
                    dkp, dki, dkd = self.fuzzy_tuner.evaluate(error / 100.0, delta_e / 100.0)

                    kp_eff = self.pid.kp_base * (1.0 + dkp)
                    ki_eff = self.pid.ki_base * (1.0 + dki)
                    kd_eff = self.pid.kd_base * (1.0 + dkd)
                    self.pid.update_gains(kp_eff, ki_eff, kd_eff)
                else:
                    kp_eff, ki_eff, kd_eff = self.pid.kp_base, self.pid.ki_base, self.pid.kd_base
                    self.pid.update_gains(kp_eff, ki_eff, kd_eff)

                u_cmd, _ = self.pid.step(error, dt)

                # Send command
                self.tello.send_rc_control(0, int(u_cmd), 0, 0)

                # Digital Shadow simulation step
                v_shadow, residual = self.shadow.step(u_cmd, vgx_real, dt)

                # Log telemetry sample
                sample = {
                    "flight_id": flight_id,
                    "session_id": session_id,
                    "timestamp": datetime.now().isoformat(),
                    "t_rel_s": round(t_rel, 3),
                    "controller_type": controller_type,
                    "scenario": scenario,
                    "repetition_number": repetition_number,
                    "setpoint_vgx": setpoint,
                    "vgx_real": round(vgx_real, 2),
                    "vgx_shadow": round(v_shadow, 2),
                    "shadow_residual": round(residual, 2),
                    "control_signal": round(u_cmd, 2),
                    "kp_effective": round(kp_eff, 4),
                    "ki_effective": round(ki_eff, 4),
                    "kd_effective": round(kd_eff, 4),
                    "pitch": telemetry.get("pitch", 0.0),
                    "roll": telemetry.get("roll", 0.0),
                    "yaw": telemetry.get("yaw", 0.0),
                    "tof": telemetry.get("tof", 0.0),
                    "h": telemetry.get("h", 0.0),
                    "bat": bat_current,
                    "agx": telemetry.get("agx", 0.0),
                    "agy": telemetry.get("agy", 0.0),
                    "agz": telemetry.get("agz", 0.0),
                    "mission_pad_x": telemetry.get("mission_pad_x"),
                    "mission_pad_y": telemetry.get("mission_pad_y"),
                    "wind_speed_measured": wind_measured,
                    "battery_pct_start": bat_start
                }
                self.data_logger.log_sample(sample)

                # Mantener cadencia
                elapsed_loop = time.time() - t_now
                sleep_time = max(0.001, dt_target - elapsed_loop)
                time.sleep(sleep_time)

        finally:
            # Descenso suave o frenado controlado
            self.tello.send_rc_control(0, 0, 0, 0)
            if "abortado" in status:
                self.tello.emergency_land()

            self.data_logger.close_flight_log()

        return {"flight_id": flight_id, "status": status, "duration_s": time.time() - t_start}
