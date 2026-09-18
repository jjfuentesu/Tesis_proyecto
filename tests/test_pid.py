import pytest
import numpy as np
from core.pid_controller import PIDController

def test_pid_proportional_response():
    pid = PIDController(kp=2.0, ki=0.0, kd=0.0)
    out, _ = pid.step(error=10.0, dt=0.1)
    assert out == 20.0

def test_pid_anti_windup_clamping():
    pid = PIDController(kp=1.0, ki=2.0, kd=0.0, max_output=50.0)
    for _ in range(100):
        out, _ = pid.step(error=100.0, dt=0.1)
    assert out == 50.0
    assert pid.integral < 1000.0  # La integral no debe crecer indefinidamente
