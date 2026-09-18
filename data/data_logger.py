"""
Módulo de persistencia incremental de datos de telemetría y metadatos de vuelo.
Escribe cada muestra inmediatamente en formato CSV con flush a disco.
"""

import os
import csv
import json
import logging
from datetime import datetime
from typing import Dict, Any, Optional

logger = logging.getLogger(__name__)

CSV_HEADERS = [
    "flight_id",
    "session_id",
    "timestamp",
    "t_rel_s",
    "controller_type",
    "scenario",
    "repetition_number",
    "setpoint_vgx",
    "vgx_real",
    "vgx_shadow",
    "shadow_residual",
    "control_signal",
    "kp_effective",
    "ki_effective",
    "kd_effective",
    "pitch",
    "roll",
    "yaw",
    "tof",
    "h",
    "bat",
    "agx",
    "agy",
    "agz",
    "mission_pad_x",
    "mission_pad_y",
    "wind_speed_measured",
    "battery_pct_start"
]

class DataLogger:
    """
    Gestor de logs de telemetría y exportación incremental a CSV.
    Garantiza persistencia física ante desconexiones inesperadas.
    """

    def __init__(self, output_dir: str = "data_output"):
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)
        self.active_file_path: Optional[str] = None
        self.file_handle = None
        self.csv_writer = None
        self.flight_metadata: Dict[str, Any] = {}

    def start_flight_log(self, flight_id: str, metadata: Dict[str, Any]) -> str:
        """Inicializa un nuevo archivo de registro de vuelo."""
        self.flight_metadata = metadata
        filename = f"flight_{flight_id}.csv"
        self.active_file_path = os.path.join(self.output_dir, filename)

        self.file_handle = open(self.active_file_path, mode="w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.file_handle, fieldnames=CSV_HEADERS)
        self.csv_writer.writeheader()
        self.file_handle.flush()

        # Guardar metadatos JSON adyacentes
        meta_filepath = os.path.join(self.output_dir, f"flight_{flight_id}_meta.json")
        with open(meta_filepath, "w", encoding="utf-8") as f:
            json.dump(metadata, f, indent=2)

        logger.info(f"Registro de vuelo iniciado: {self.active_file_path}")
        return self.active_file_path

    def log_sample(self, sample_data: Dict[str, Any]) -> None:
        """
        Escribe una muestra en disco y aplica flush inmediato.
        """
        if not self.csv_writer or not self.file_handle:
            logger.error("Intento de registrar muestra sin log abierto.")
            return

        row = {}
        for key in CSV_HEADERS:
            row[key] = sample_data.get(key, "")

        self.csv_writer.writerow(row)
        self.file_handle.flush()

    def close_flight_log(self) -> None:
        """Cierra el archivo de log de forma segura."""
        if self.file_handle:
            try:
                self.file_handle.flush()
                self.file_handle.close()
            except Exception as e:
                logger.error(f"Error cerrando archivo de log: {e}")
            finally:
                self.file_handle = None
                self.csv_writer = None
                logger.info("Registro de vuelo cerrado correctamente.")
