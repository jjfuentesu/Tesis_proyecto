import pytest
from core.dynamics_model import FOPDTDynamicsModel
from core.digital_shadow import DigitalShadow

def test_fopdt_step_response():
    model = FOPDTDynamicsModel("config/dynamics_params.json")
    model.reset(0.0)

    # Aplicar escalón de control 50
    velocities = []
    for _ in range(50):
        v = model.step(50.0, dt=0.1)
        velocities.append(v)

    # La velocidad debe aumentar monotonicamente hacia la ganancia estática
    assert velocities[-1] > velocities[0]

def test_digital_shadow_residual():
    model = FOPDTDynamicsModel("config/dynamics_params.json")
    shadow = DigitalShadow(model)
    shadow.reset(0.0)

    v_sim, residual = shadow.step(control_signal=30.0, vgx_real=25.0, dt=0.1)
    assert residual == (25.0 - v_sim)
