import numpy as np
import pytest
from muscle_audit_utils import backends, model_file, numeric


@pytest.mark.parametrize("b", backends)
@pytest.mark.parametrize(
    "family,length", [("thelen", 0.2), ("thelen", 0.22), ("hill", 0.22)]
)
def test_static_optimization_reconstructs_physical_target(b, tmp_path, family, length):
    if not hasattr(b, "StaticOptimization"):
        pytest.skip("Static optimization is Eigen-only")
    model = b.Model(model_file(tmp_path, **{"Type thelen": "Type " + family}))
    q, v = np.array([length]), np.array([0.0])
    state = model.stateSet()
    state[0].setActivation(0.37)
    target = numeric(model.muscularJointTorque(state, q, v)).ravel()
    optimization = b.StaticOptimization(model, q, v, target, 0.5, 2, False, 0)
    optimization.run(True)
    activation = numeric(optimization.finalSolution()[0]).item()
    np.testing.assert_allclose(activation, 0.37, atol=1e-7)
    state[0].setActivation(activation)
    np.testing.assert_allclose(
        numeric(model.muscularJointTorque(state, q, v)).ravel(), target, atol=1e-5
    )
