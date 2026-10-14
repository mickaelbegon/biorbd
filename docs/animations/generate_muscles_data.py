"""Data for the video 11 (muscles): muscle length, moment arm (length Jacobian) and joint torque along an elbow sweep.

Run with the biorbd_anim environment:  python generate_muscles_data.py
The lines between the CODE-BEGIN and CODE-END markers are displayed as is in the code panel (rule D5).
Sign convention checked numerically here: muscles.joint_torque equals -J^T F, with J = length_jacobian = dLength/dq.
"""

from pathlib import Path

import biorbd
import numpy as np

HERE = Path(__file__).resolve().parent
MODEL_PATH = HERE.parents[1] / "examples" / "arm26.bioMod"
MUSCLE = "BIClong"  # biceps long head (Hill-Thelen type in arm26.bioMod), crosses the elbow
SHOULDER = 0.3  # rad, fixed
ELBOW = np.linspace(0.2, 2.2, 41)  # rad, the swept joint (q[1])
ACTIVATION = 0.5
SHOWN = 20  # index of the highlighted elbow angle

model = biorbd.Biorbd(str(MODEL_PATH))
n_mus = len(model.muscles)
act = np.full(n_mus, ACTIVATION)
qdot = np.zeros(model.nb_q)

LENGTH = np.zeros(len(ELBOW))
DL_DQ = np.zeros((len(ELBOW), model.nb_q))
FORCE = np.zeros(len(ELBOW))
TAU = np.zeros((len(ELBOW), model.nb_q))
TAU_HAND = np.zeros((len(ELBOW), model.nb_q))
for i, elbow in enumerate(ELBOW):
    q = np.array([SHOULDER, elbow])
    # CODE-BEGIN
    ms = model.muscles
    ms.update_geometry(q, qdot)
    mus = ms["BIClong"]
    length = mus.muscle_tendon_length
    dl_dq = mus.length_jacobian
    forces = ms.forces(act, q, qdot)
    tau = ms.joint_torque(act, q, qdot)
    Js = [m.length_jacobian for m in ms]
    J = np.vstack(Js)
    tau_hand = -J.T @ forces
    # CODE-END
    LENGTH[i], DL_DQ[i], TAU[i], TAU_HAND[i] = length, np.ravel(dl_dq), tau, tau_hand
    FORCE[i] = forces[[m.name for m in model.muscles].index(MUSCLE)]

# Extra check (not displayed): dLength/dq of the Jacobian against a central finite difference of the length
fd = np.gradient(LENGTH, ELBOW)
diff = TAU - TAU_HAND
np.savez(
    HERE / "data" / "muscles_main.npz",
    muscle=MUSCLE,
    muscle_type="hillthelen",
    activation=ACTIVATION,
    shoulder=SHOULDER,
    elbow=ELBOW,
    length=LENGTH,
    dl_dq=DL_DQ[:, 1],
    moment_arm=-DL_DQ[:, 1],
    force=FORCE,
    tau_elbow=TAU[:, 1],
    tau_hand_elbow=TAU_HAND[:, 1],
    diff_elbow=diff[:, 1],
    max_diff=np.abs(diff).max(),
    max_fd_gap=np.abs(fd[1:-1] - DL_DQ[1:-1, 1]).max(),
    shown=SHOWN,
)
print("max |tau - tau_hand| =", np.abs(diff).max(), " max |tau_elbow| =", np.abs(TAU[:, 1]).max())
print("max |dL/dq - finite diff| (interior) =", np.abs(fd[1:-1] - DL_DQ[1:-1, 1]).max())
print("moment arm range", (-DL_DQ[:, 1]).min(), (-DL_DQ[:, 1]).max(), " length range", LENGTH.min(), LENGTH.max())
print("shoulder torque range", TAU[:, 0].min(), TAU[:, 0].max(), "elbow", TAU[:, 1].min(), TAU[:, 1].max())
print(
    "sign check: corr(tau, tau_hand)",
    np.corrcoef(TAU.ravel(), TAU_HAND.ravel())[0, 1],
    " vs +J^T F gap",
    np.abs(TAU + TAU_HAND).max(),
)
