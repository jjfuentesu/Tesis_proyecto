"""
Módulo para el cálculo numérico de métricas de desempeño de control y validación de sombra.
Soporta cálculo cuantitativo estandarizado (RMSE, MAE, Settling Time, Rise Time, ISE, IAE, ITAE, etc.).
"""

import numpy as np
import pandas as pd
from typing import Dict, Any

def calculate_flight_metrics(df: pd.DataFrame, setpoint_col: str = "setpoint_vgx", real_col: str = "vgx_real", shadow_col: str = "vgx_shadow", control_col: str = "control_signal", time_col: str = "t_rel_s") -> Dict[str, float]:
    """
    Calcula el conjunto completo de métricas cuantitativas sobre un DataFrame de vuelo.
    """
    if df.empty:
        return {}

    t = df[time_col].values
    sp = df[setpoint_col].values
    v_real = df[real_col].values
    v_shadow = df[shadow_col].values
    u = df[control_col].values

    error = sp - v_real

    # Métricas de error clásicas
    rmse = float(np.sqrt(np.mean(error ** 2)))
    mae = float(np.mean(np.abs(error)))

    # Integrales de error
    dt = np.diff(t, prepend=t[0])
    dt[0] = 0.1  # Primer dt

    ise = float(np.sum((error ** 2) * dt))
    iae = float(np.sum(np.abs(error) * dt))
    itae = float(np.sum(t * np.abs(error) * dt))

    # Esfuerzo de control
    control_variance = float(np.var(u))
    # Número de cambios de signo en la derivada del comando
    du = np.diff(u)
    sign_changes = int(np.sum(np.diff(np.sign(du)) != 0))

    # Tiempo de asentamiento (±5% del setpoint)
    setpoint_target = sp[-1] if len(sp) > 0 else 0.0
    band = 0.05 * abs(setpoint_target) if setpoint_target != 0 else 1.0
    out_of_band = np.abs(v_real - setpoint_target) > band

    if not np.any(out_of_band):
        settling_time = 0.0
    else:
        last_out_idx = np.where(out_of_band)[0][-1]
        settling_time = float(t[last_out_idx]) if last_out_idx < len(t) - 1 else float(t[-1])

    # Tiempo de subida (10% a 90%)
    target_10 = 0.1 * setpoint_target
    target_90 = 0.9 * setpoint_target

    idx_10 = np.where(v_real >= target_10)[0]
    idx_90 = np.where(v_real >= target_90)[0]

    if len(idx_10) > 0 and len(idx_90) > 0:
        rise_time = float(t[idx_90[0]] - t[idx_10[0]])
    else:
        rise_time = float('nan')

    # Sobreimpulso máximo
    if setpoint_target != 0:
        max_v = float(np.max(v_real))
        overshoot = float(max(0.0, (max_v - setpoint_target) / abs(setpoint_target) * 100.0))
    else:
        overshoot = 0.0

    # Fidelidad de la sombra (RMSE entre real y sombra)
    shadow_rmse = float(np.sqrt(np.mean((v_real - v_shadow) ** 2)))

    return {
        "rmse": rmse,
        "mae": mae,
        "ise": ise,
        "iae": iae,
        "itae": itae,
        "settling_time_s": settling_time,
        "rise_time_s": rise_time,
        "overshoot_pct": overshoot,
        "control_variance": control_variance,
        "sign_changes": sign_changes,
        "shadow_rmse": shadow_rmse,
    }
