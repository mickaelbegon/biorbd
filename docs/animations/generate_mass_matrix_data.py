"""Data for video 07 (mass matrix): M(q) of examples/arm26.bioMod at two elbow postures.

Run with the biorbd_anim environment. The lines between the two ``# code:`` markers are the ones shown
in the code panel of the video (rule D5); everything else only stores the results.
"""

from pathlib import Path

import biorbd
import numpy as np

ANIM = Path(__file__).resolve().parent
path = str(ANIM.parents[1] / "examples" / "arm26.bioMod")
DRAWN_MARKERS = ["r_acromion", "r_humerus_epicondyle", "r_radius_styloid"]  # shoulder, elbow, wrist (drawing only)

model = biorbd.Biorbd(path)


def drawn_points(q):
    """World positions of the shoulder, elbow and wrist markers (only used to draw the arm)."""
    markers = {mk.name: mk for mk in model.markers}
    return np.array([markers[name].forward_kinematics(q) for name in DRAWN_MARKERS])


# code:start
q = np.array([0.3, 0.3])
M = model.mass_matrix(q)
err = np.abs(M - M.T).max()
lam = np.linalg.eigvalsh(M).min()
# code:end
q_a, M_a, asym_a, lam_a = q.copy(), M.copy(), float(err), float(lam)

# code:start
q = np.array([0.3, 1.8])
M2 = model.mass_matrix(q)
err = np.abs(M2 - M2.T).max()
lam = np.linalg.eigvalsh(M2).min()
dM = M2 - M
# code:end
q_b, M_b, asym_b, lam_b = q.copy(), M2.copy(), float(err), float(lam)

# Not displayed: consistency of the inverse option with numpy, and of the list-of-matrices shape.
inv_err = float(np.abs(model.mass_matrix(q_b, inverse=True) - np.linalg.inv(M_b)).max())
assert inv_err < 1e-8 and np.allclose(dM, M_b - M_a) and model.nb_q == 2

np.savez(
    ANIM / "data" / "mass_matrix_main.npz",
    nb_q=model.nb_q,
    q_a=q_a,
    q_b=q_b,
    M_a=M_a,
    M_b=M_b,
    dM=dM,
    asym_a=asym_a,
    asym_b=asym_b,
    lam_a=lam_a,
    lam_b=lam_b,
    points_a=drawn_points(q_a),
    points_b=drawn_points(q_b),
    model_name=np.array("arm26.bioMod"),
)
print("M_a", M_a, "M_b", M_b, "dM", dM, "asym", asym_a, asym_b, "lam", lam_a, lam_b, "inv_err", inv_err)
