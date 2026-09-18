import pytest
import pandas as pd
import numpy as np
from data.metrics import calculate_flight_metrics

def test_metrics_calculation():
    df = pd.DataFrame({
        "t_rel_s": [0.0, 0.1, 0.2, 0.3, 0.4],
        "setpoint_vgx": [30.0, 30.0, 30.0, 30.0, 30.0],
        "vgx_real": [0.0, 10.0, 20.0, 28.0, 30.0],
        "vgx_shadow": [0.0, 8.0, 18.0, 27.0, 30.0],
        "control_signal": [0.0, 50.0, 40.0, 30.0, 25.0]
    })

    metrics = calculate_flight_metrics(df)
    assert "rmse" in metrics
    assert "mae" in metrics
    assert "shadow_rmse" in metrics
    assert metrics["rmse"] > 0
