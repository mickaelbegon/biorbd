"""Data for the video 09 (inverse dynamics): tau from model.inverse_dynamics equals M(q) qddot + N(q, qdot).

Run with the biorbd_anim environment:  python generate_inverse_dynamics_data.py
The lines between the CODE-BEGIN and CODE-END markers are displayed as is in the code panel (rule D5).
"""

from pathlib import Path

import biorbd
import numpy as np

HERE = Path(__file__).resolve().parent
MODEL_PATH = HERE.parents[1] / "examples" / "arm26.bioMod"
N_FRAMES = 101
DURATION = 1.0  # s
SHOWN_FRAME = 30  # frame whose numbers are displayed

model = biorbd.Biorbd(str(MODEL_PATH))
time = np.linspace(0.0, DURATION, N_FRAMES)
w = 2 * np.pi
# Synthetic smooth motion (not a biorbd output): sinusoids in the two joints, with analytic derivatives.
AMP, OFF, PHASE = np.array([0.4, 0.5]), np.array([0.3, 0.9]), np.array([0.0, 1.0])
Q = OFF + AMP * np.sin(w * time[:, None] + PHASE)
QDOT = AMP * w * np.cos(w * time[:, None] + PHASE)
QDDOT = -AMP * w**2 * np.sin(w * time[:, None] + PHASE)

n = model.nb_q
TAU_ID, TAU_MN, M_QDDOT, NLE = (np.zeros((N_FRAMES, n)) for _ in range(4))
for k in range(N_FRAMES):
    q, qdot, qddot = Q[k], QDOT[k], QDDOT[k]
    # CODE-BEGIN
    tau = model.inverse_dynamics(q, qdot, qddot)
    M = model.mass_matrix(q)
    N = model.non_linear_effect(q, qdot)
    tau_check = M @ qddot + N
    # CODE-END
    TAU_ID[k], TAU_MN[k], M_QDDOT[k], NLE[k] = tau, tau_check, M @ qddot, N

diff = TAU_ID - TAU_MN
np.savez(
    HERE / "data" / "inverse_dynamics_main.npz",
    time=time,
    q=Q,
    qdot=QDOT,
    qddot=QDDOT,
    tau_id=TAU_ID,
    tau_check=TAU_MN,
    m_qddot=M_QDDOT,
    nle=NLE,
    diff_abs=np.abs(diff).max(axis=1),
    diff_max=np.abs(diff).max(),
    shown_index=SHOWN_FRAME,
    shown_time=time[SHOWN_FRAME],
)
print("max |tau - (M qddot + N)| =", np.abs(diff).max(), " max |tau| =", np.abs(TAU_ID).max())
print("shown", TAU_ID[SHOWN_FRAME], M_QDDOT[SHOWN_FRAME], NLE[SHOWN_FRAME])
