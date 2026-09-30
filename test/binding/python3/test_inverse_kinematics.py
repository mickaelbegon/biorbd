"""
Test for file IO
"""

import pytest
import numpy as np

brbd_to_test = []
try:
    import biorbd

    brbd_to_test.append(biorbd)
except ModuleNotFoundError as e:
    print(f"Error importing biorbd: {e}")
    pass

try:
    import biorbd_casadi

    brbd_to_test.append(biorbd_casadi)
except ModuleNotFoundError as e:
    print(f"Error importing biorbd_casadi: {e}")
    pass

if not brbd_to_test:
    raise RuntimeError("No biorbd version could be imported")


@pytest.mark.parametrize("brbd", brbd_to_test)
@pytest.mark.parametrize("method", ["only_lm", "lm", "trf"])
def test_solve(brbd, method):
    if brbd.backend == brbd.CASADI:
        pytest.skip("Skip inverse kinematics for biorbd_casadi")

    biorbd_model = brbd.Model("../../models/pyomecaman.bioMod")

    # Remove the dampings in this test
    joint_dampings = [0, 0, 0]
    biorbd_model.segment(0).setJointDampings(joint_dampings)

    # qinit must lie inside the joint ranges of the model (the knees are in [-2.09, 0]),
    # otherwise the bounded methods ("lm" on the first frame and "trf") cannot recover it
    qinit = np.array([0.1, 0.1, -0.3, 0.35, 1.15, -0.35, 1.15, 0.1, -0.1, 0.1, 0.1, -0.1, 0.1])

    markers = np.ndarray((3, biorbd_model.nbMarkers(), 1))
    markers[:, :, 0] = np.array([mark.to_array() for mark in biorbd_model.markers(qinit)]).T

    ik = biorbd.InverseKinematics(biorbd_model, markers)
    ik_q = ik.solve(method=method)

    np.testing.assert_almost_equal(np.squeeze(ik_q.T), qinit)


if __name__ == "__main__":
    for brbd in brbd_to_test:
        for method in ["only_lm", "lm", "trf"]:
            test_solve(brbd, method)
