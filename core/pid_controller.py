"""
Controlador PID clásico con anti-windup por saturación integral (clamping).
Soporta actualización dinámica de ganancias para sintonización en tiempo real.
"""

from typing import Dict, Tuple

class PIDController:
    """
    Controlador PID con anti-windup por clamping.
    Diseñado para trabajar sobre la señal de control RC [-100, 100].
    """

    def __init__(self, kp: float, ki: float, kd: float, min_output: float = -100.0, max_output: float = 100.0):
        self.kp_base = float(kp)
        self.ki_base = float(ki)
        self.kd_base = float(kd)

        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)

        self.min_output = float(min_output)
        self.max_output = float(max_output)

        self.integral = 0.0
        self.prev_error = 0.0
        self.is_first_step = True

    def reset(self) -> None:
        """Reinicia el estado interno del controlador para un nuevo vuelo."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.is_first_step = True

    def update_gains(self, kp: float, ki: float, kd: float) -> None:
        """Actualiza las ganancias efectivas del controlador."""
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)

    def step(self, error: float, dt: float) -> Tuple[float, Dict[str, float]]:
        """
        Calcula la señal de control PID dada la desviación actual y dt.
        Aplica anti-windup clamping en el acumulador integral.
        """
        if dt <= 0.0:
            dt = 0.1  # Fallback seguro para evitar división por cero

        # Término Proporcional
        p_term = self.kp * error

        # Término Derivativo (diferencia finita)
        if self.is_first_step:
            d_term = 0.0
            self.is_first_step = False
        else:
            delta_e = (error - self.prev_error) / dt
            d_term = self.kd * delta_e

        # Integración preliminar
        new_integral = self.integral + (error * dt)
        i_term = self.ki * new_integral

        # Salida no saturada
        raw_output = p_term + i_term + d_term

        # Aplicación de saturación y Anti-Windup Clamping
        if raw_output > self.max_output:
            output = self.max_output
            # Clamping: no acumular integral si el error incrementa la saturación
            if error <= 0:
                self.integral = new_integral
        elif raw_output < self.min_output:
            output = self.min_output
            if error >= 0:
                self.integral = new_integral
        else:
            output = raw_output
            self.integral = new_integral

        self.prev_error = error

        diagnostics = {
            "p_term": p_term,
            "i_term": i_term,
            "d_term": d_term,
            "raw_output": raw_output,
            "kp_effective": self.kp,
            "ki_effective": self.ki,
            "kd_effective": self.kd,
        }

        return output, diagnostics
