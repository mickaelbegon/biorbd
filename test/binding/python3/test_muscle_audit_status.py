import importlib
import sys

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


@pytest.mark.skipif(sys.version_info < (3, 12), reason="The high-level wrapper requires Python 3.12")
@pytest.mark.parametrize("b", backends)
def test_wrapper_frames_match_physically_validated_nonlinear_solution(b):
    if not hasattr(b, "StaticOptimization"):
        pytest.skip("Static optimization is Eigen-only")
    package = importlib.import_module(b.__package__)
    model = package.Biorbd(str(fixture.parent / "arm26.bioMod"))
    all_q = np.zeros((3, model.nb_q))
    all_v = np.zeros((3, model.nb_qdot))
    all_tau = np.asarray(
        [model.inverse_dynamics(q, v, np.zeros(model.nb_qddot)) for q, v in zip(all_q, all_v)]
    )
    reference = b.StaticOptimization(
        model.internal,
        [b.GeneralizedCoordinates(q) for q in all_q],
        [b.GeneralizedVelocity(v) for v in all_v],
        [b.GeneralizedTorque(tau) for tau in all_tau],
    )
    reference.run(False)
    assert all(status in (0, 1) for status in reference.solverStatuses())
    activations = list(package.StaticOptimization(model).perform_frames(all_q, all_v, all_tau))
    assert len(activations) == len(all_q)
    state = model.internal.stateSet()
    for i, (activation, q, v, target) in enumerate(zip(activations, all_q, all_v, all_tau)):
        np.testing.assert_allclose(
            activation, numeric(reference.finalSolution(i)).ravel(), rtol=1e-7, atol=1e-9
        )
        for muscle, value in zip(state, activation):
            muscle.setActivation(float(value))
        reconstructed = numeric(model.internal.muscularJointTorque(state, q, v)).ravel()
        reconstructed += numeric(reference.finalResidual(i)).ravel()
        assert np.all(np.abs(reconstructed - target) <= 1e-6 * (1 + np.abs(target)))
