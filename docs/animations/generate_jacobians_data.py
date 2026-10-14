"""Data for the video 05 (jacobians): v = J(q) * qdot checked against finite differences of marker positions.

Run with the biorbd_anim environment:  python generate_jacobians_data.py
The lines between the CODE-BEGIN and CODE-END markers are displayed as is in the code panel (rule D5).
"""

from pathlib import Path

import biorbd
import numpy as np

HERE = Path(__file__).resolve().parent
MODEL_PATH = HERE.parents[1] / "examples" / "arm26.bioMod"
MARKER_INDEX = 3  # r_radius_styloid, attached to the forearm (moved by both degrees of freedom)
N_FRAMES = 101
DURATION = 1.0  # s
SHOWN_FRAME = 40  # frame whose matrix is displayed

model = biorbd.Biorbd(str(MODEL_PATH))
time = np.linspace(0.0, DURATION, N_FRAMES)
dt = time[1] - time[0]
# Synthetic smooth trajectory (not a biorbd output): sinusoids in the two joints, with analytic derivatives.
Q = np.stack(
    [0.3 + 0.4 * np.sin(2 * np.pi * time), 0.9 + 0.5 * np.sin(2 * np.pi * time + 1.0)],
    axis=1,
)
QDOT = np.stack(
    [0.4 * 2 * np.pi * np.cos(2 * np.pi * time), 0.5 * 2 * np.pi * np.cos(2 * np.pi * time + 1.0)],
    axis=1,
)

frames = range(1, N_FRAMES - 1)
JAC = np.zeros((len(frames), 3, model.nb_q))
V_JAC = np.zeros((len(frames), 3))
V_FD = np.zeros((len(frames), 3))
for i, k in enumerate(frames):
    q, qdot, q0, q1 = Q[k], QDOT[k], Q[k - 1], Q[k + 1]
    # CODE-BEGIN
    marker = model.markers[3]
    J = marker.jacobian(q)
    v_jac = J @ qdot
    p0 = marker.forward_kinematics(q0)
    p1 = marker.forward_kinematics(q1)
    v_fd = (p1 - p0) / (2 * dt)
    # CODE-END
    JAC[i], V_JAC[i], V_FD[i] = J, v_jac, v_fd

diff = V_JAC - V_FD
shown = SHOWN_FRAME - 1  # index in the arrays that start at frame 1
np.savez(
    HERE / "data" / "jacobians_main.npz",
    time=time[1:-1],
    dt=dt,
    marker_name=model.markers[MARKER_INDEX].name,
    q_shown=Q[SHOWN_FRAME],
    qdot_shown=QDOT[SHOWN_FRAME],
    jac_shown=JAC[shown],
    v_jac_shown=V_JAC[shown],
    v_fd_shown=V_FD[shown],
    speed_jac=np.linalg.norm(V_JAC, axis=1),
    speed_fd=np.linalg.norm(V_FD, axis=1),
    diff_norm=np.linalg.norm(diff, axis=1),
    shown_time=time[SHOWN_FRAME],
)
print("max |v_jac - v_fd| =", np.abs(diff).max(), " max speed =", np.linalg.norm(V_JAC, axis=1).max())
