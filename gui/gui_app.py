"""
Interfaz Gráfica de Usuario (GUI) basada en Tkinter.
Proporciona pantallas de configuración, monitoreo en tiempo real y parada de emergencia prominente.
"""

import sys
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

from core.tello_interface import TelloInterface
from core.pid_controller import PIDController
from core.fuzzy_tuner import FuzzyTuner
from core.dynamics_model import FOPDTDynamicsModel
from core.digital_shadow import DigitalShadow
from data.data_logger import DataLogger
from missions.mission_a_to_b import MissionAToB

class FlightControlGUI:
    """
    Aplicación gráfica para configuración de vuelos, monitoreo en tiempo real y parada de emergencia.
    Cumple con el requerimiento de separación estricta: sólo invoca métodos del Core.
    """

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Sistema de Control Fuzzy Self-Tuning PID & Sombra Digital - Dron Educativo")
        self.root.geometry("1024x720")

        # Configuración por defecto
        self.flight_config = {
            "controller": "pid",
            "scenario": "controlado",
            "setpoint_vgx": 30.0,
            "mission_duration_s": 8.0,
            "sampling_rate_hz": 10.0,
            "pid_gains": {"kp": 1.2, "ki": 0.15, "kd": 0.05},
            "safety_limits": {"battery_min_pct": 15.0, "telemetry_timeout_s": 2.0}
        }

        # Instanciación Core
        self.tello = TelloInterface(use_mock=True)
        self.pid = PIDController(kp=1.2, ki=0.15, kd=0.05)
        self.fuzzy = FuzzyTuner("config/fuzzy_rules.json")
        self.dynamics = FOPDTDynamicsModel("config/dynamics_params.json")
        self.shadow = DigitalShadow(self.dynamics)
        self.logger_inst = DataLogger()
        self.mission = MissionAToB(self.tello, self.pid, self.fuzzy, self.shadow, self.logger_inst, self.flight_config)

        self._build_ui()

    def _build_ui(self) -> None:
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True)

        # Tab 1: Configuración
        tab_config = ttk.Frame(notebook)
        notebook.add(tab_config, text="Configuración de Vuelo")
        self._build_config_tab(tab_config)

        # Tab 2: Monitoreo en Tiempo Real
        tab_monitor = ttk.Frame(notebook)
        notebook.add(tab_monitor, text="Monitor en Tiempo Real")
        self._build_monitor_tab(tab_monitor)

    def _build_config_tab(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Parámetros del Experimento", padding=15)
        frame.pack(fill="both", expand=True, padx=10, pady=10)

        ttk.Label(frame, text="Controlador:").grid(row=0, column=0, sticky="w", pady=5)
        self.combo_ctrl = ttk.Combobox(frame, values=["pid", "fuzzy_self_tuning"], state="readonly")
        self.combo_ctrl.set("pid")
        self.combo_ctrl.grid(row=0, column=1, pady=5)

        ttk.Label(frame, text="Escenario:").grid(row=1, column=0, sticky="w", pady=5)
        self.combo_scenario = ttk.Combobox(frame, values=["controlado", "viento_natural", "perturbacion_artificial"], state="readonly")
        self.combo_scenario.set("controlado")
        self.combo_scenario.grid(row=1, column=1, pady=5)

        ttk.Label(frame, text="Setpoint vgx (cm/s):").grid(row=2, column=0, sticky="w", pady=5)
        self.entry_sp = ttk.Entry(frame)
        self.entry_sp.insert(0, "30.0")
        self.entry_sp.grid(row=2, column=1, pady=5)

        btn_connect = ttk.Button(frame, text="Conectar Dron (Mock/Real)", command=self._connect_drone)
        btn_connect.grid(row=3, column=0, columnspan=2, pady=15)

    def _build_monitor_tab(self, parent: ttk.Frame) -> None:
        # Botón de Parada de Emergencia Prominente (RNF8)
        btn_emergency = tk.Button(parent, text="🚨 PARADA DE EMERGENCIA 🚨", bg="red", fg="white", font=("Helvetica", 16, "bold"), command=self._emergency_stop)
        btn_emergency.pack(fill="x", padx=10, pady=10)

        fig, self.ax = plt.subplots(figsize=(6, 3))
        self.ax.set_title("Respuesta de Velocidad: Real vs. Sombra Digital")
        self.ax.set_xlabel("Tiempo (s)")
        self.ax.set_ylabel("vgx (cm/s)")

        self.canvas = FigureCanvasTkAgg(fig, master=parent)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=10)

    def _connect_drone(self) -> None:
        if self.tello.connect():
            messagebox.showinfo("Conexión", "Conectado correctamente al dron.")

    def _emergency_stop(self) -> None:
        self.mission.request_emergency_abort("parada_manual_gui")
        messagebox.showwarning("Emergencia", "Comando de Parada de Emergencia Enviado.")

def main():
    root = tk.Tk()
    app = FlightControlGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
