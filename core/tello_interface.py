"""
Capa de interfaz con el hardware del dron DJI Tello.
Proporciona abstracción unificada para conexión real vía SDK y modo Mock para simulación offline.
"""

import time
import logging
from typing import Dict, Any, Optional

try:
    from djitellopy import Tello
    HAS_TELLO_SDK = True
except ImportError:
    HAS_TELLO_SDK = False

logger = logging.getLogger(__name__)

class TelloInterface:
    """
    Abstracción de interfaz para comunicación con dron DJI Tello EDU.
    Soporta modo 'mock' seguro para desarrollo offline y pruebas automáticas.
    """

    def __init__(self, use_mock: bool = False):
        self.use_mock = use_mock or not HAS_TELLO_SDK
        self.tello: Optional[Any] = None
        self.is_connected = False
        self.last_telemetry_time = 0.0

        # Estado interno simulado para modo Mock
        self._mock_state = {
            "vgx": 0.0,
            "vgy": 0.0,
            "vgz": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
            "yaw": 0.0,
            "tof": 100.0,
            "h": 100.0,
            "bat": 95.0,
            "agx": 0.0,
            "agy": 0.0,
            "agz": -980.0,
            "mission_pad_x": None,
            "mission_pad_y": None
        }
        self._last_rc_cmd = 0.0

    def connect(self) -> bool:
        """Establece conexión física o virtual con el dron."""
        if self.use_mock:
            self.is_connected = True
            self.last_telemetry_time = time.time()
            logger.info("Conectado en Modo Mock (Simulación Offline).")
            return True

        try:
            self.tello = Tello()
            self.tello.connect()
            self.tello.streamoff()
            self.is_connected = True
            self.last_telemetry_time = time.time()
            logger.info("Conexión exitosa con DJI Tello EDU físico.")
            return True
        except Exception as e:
            logger.error(f"Error al conectar con Tello físico: {e}")
            self.is_connected = False
            return False

    def disconnect(self) -> None:
        """Desconecta el dron de forma segura."""
        if self.is_connected and not self.use_mock and self.tello:
            try:
                self.tello.send_rc_control(0, 0, 0, 0)
                self.tello.end()
            except Exception as e:
                logger.warning(f"Excepción durante desconexión: {e}")
        self.is_connected = False
        logger.info("Desconectado de la interfaz Tello.")

    def send_rc_control(self, left_right: int, forward_backward: int, up_down: int, yaw: int) -> None:
        """
        Envía comandos continuos RC [-100, 100].
        `forward_backward` controla el canal vgx.
        """
        if not self.is_connected:
            return

        cmd_fb = max(-100, min(100, int(forward_backward)))

        if self.use_mock:
            self._last_rc_cmd = float(cmd_fb)
            # Modelo cinemático simple para actualizar respuesta del mock
            target_vgx = self._last_rc_cmd * 0.8  # Ganancia estática aproximada
            self._mock_state["vgx"] += (target_vgx - self._mock_state["vgx"]) * 0.3
            self.last_telemetry_time = time.time()
            return

        try:
            self.tello.send_rc_control(left_right, cmd_fb, up_down, yaw)
            self.last_telemetry_time = time.time()
        except Exception as e:
            logger.error(f"Error enviando comando RC: {e}")

    def emergency_land(self) -> None:
        """Aterrizaje inmediato o corte seguro de comandos."""
        logger.warning("Ejecutando comando de ATERRIZAJE DE EMERGENCIA.")
        if self.use_mock:
            self._mock_state["vgx"] = 0.0
            self.send_rc_control(0, 0, 0, 0)
            return

        if self.tello:
            try:
                self.tello.send_rc_control(0, 0, 0, 0)
                self.tello.land()
            except Exception as e:
                logger.error(f"Fallo en orden de aterrizaje: {e}")

    def get_telemetry(self) -> Dict[str, Any]:
        """Obtiene el estado de telemetría actual de forma no bloqueante."""
        if not self.is_connected:
            return self._mock_state

        if self.use_mock:
            # Simular ligera descarga de batería
            self._mock_state["bat"] = max(0.0, self._mock_state["bat"] - 0.001)
            self.last_telemetry_time = time.time()
            return dict(self._mock_state)

        try:
            telemetry = {
                "vgx": float(self.tello.get_speed_x()),
                "vgy": float(self.tello.get_speed_y()),
                "vgz": float(self.tello.get_speed_z()),
                "pitch": float(self.tello.get_pitch()),
                "roll": float(self.tello.get_roll()),
                "yaw": float(self.tello.get_yaw()),
                "tof": float(self.tello.get_distance_tof()),
                "h": float(self.tello.get_height()),
                "bat": float(self.tello.get_battery()),
                "agx": float(self.tello.get_acceleration_x()),
                "agy": float(self.tello.get_acceleration_y()),
                "agz": float(self.tello.get_acceleration_z()),
                "mission_pad_x": self.tello.get_mission_pad_distance_x() if hasattr(self.tello, "get_mission_pad_distance_x") else None,
                "mission_pad_y": self.tello.get_mission_pad_distance_y() if hasattr(self.tello, "get_mission_pad_distance_y") else None,
            }
            self.last_telemetry_time = time.time()
            return telemetry
        except Exception as e:
            logger.warning(f"Error al leer telemetría real: {e}")
            return self._mock_state
