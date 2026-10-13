import numpy as np
import pytest
from muscle_audit_utils import backends, fixture, numeric


@pytest.mark.parametrize("b", backends)
def test_solver_failure_and_physical_balance(b):
    if not hasattr(b, "StaticOptimization"):
        pytest.skip("Static optimization is Eigen-only")
    model = b.Model(str(fixture))
    q, v = np.array([0.2]), np.array([0.0])
    failed = b.StaticOptimization(model, q, v, np.array([-2000.0]), 0.5, 2, False, 0)
    with pytest.raises(RuntimeError, match="failed"):
        failed.run(False)
    assert list(failed.solverStatuses())[0] not in [0, 1]
    with pytest.raises(RuntimeError):
        failed.finalSolution()
    optimization = b.StaticOptimization(
        model, q, v, np.array([-500.0]), 0.5, 2, True, 0
    )
    for _ in range(2):
        optimization.run(False)
        assert len(optimization.finalSolution()) == 1
        assert len(optimization.solverStatuses()) == 1
        np.testing.assert_allclose(
            numeric(optimization.finalTorqueError(0)), 0, atol=1e-5
        )
        assert (
            numeric(optimization.finalResidual(0)).size == model.nbGeneralizedTorque()
        )
