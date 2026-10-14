import numpy as np
import pytest
from muscle_audit_utils import backends, fixture, numeric, ca


@pytest.mark.parametrize("b", backends)
@pytest.mark.parametrize("angle", [0.0, 0.3, np.pi / 3])
def test_fiber_derivative_and_virtual_power(b, angle):
    model = b.Model(str(fixture))
    model.muscle(0).characteristics().setPennationAngle(angle)
    q, v = np.array([0.2]), np.array([0.1])
    model.updateMuscles(q, v, True)
    geometry = model.muscle(0).position()
    velocity = numeric(geometry.velocity()).item()
    mt_velocity = numeric(geometry.musculoTendonVelocity()).item()
    np.testing.assert_allclose(velocity, v[0] / np.cos(angle), atol=1e-12)
    np.testing.assert_allclose(mt_velocity, v[0], atol=1e-12)
    np.testing.assert_allclose(
        numeric(geometry.fiberLengthJacobian()), [[1 / np.cos(angle)]], atol=1e-12
    )
    model.updateMuscles(q + 1e-7, v, True)
    plus = numeric(geometry.length()).item()
    model.updateMuscles(q - 1e-7, v, True)
    minus = numeric(geometry.length()).item()
    np.testing.assert_allclose(velocity, (plus - minus) / 2e-7 * v[0], atol=1e-9)
    torque = numeric(model.muscularJointTorque(np.array([100.0]))).item()
    assert abs(torque * v[0] + 100 * mt_velocity) < 1e-12
    if b.currentLinearAlgebraBackend() == b.CASADI:
        q_sym, v_sym = ca.MX.sym("q", 1), ca.MX.sym("v", 1)
        model.updateMuscles(q_sym, v_sym, True)
        derivative = ca.jacobian(geometry.length().to_mx(), q_sym) @ v_sym
        difference = ca.Function(
            "fiber_identity", [q_sym, v_sym], [geometry.velocity().to_mx() - derivative]
        )
        np.testing.assert_allclose(difference(q, v), 0, atol=1e-12)
