"""Data for the video 08 (nonlinear effects): N(q, 0) is gravity, N(q, qdot) - N(q, 0) grows with qdot squared.

Run with the biorbd_anim environment:  python generate_nonlinear_effects_data.py
The lines between the CODE-BEGIN and CODE-END markers are displayed as is in the code panel (rule D5).
Model: examples/pyomecaman.bioMod (13 degrees of freedom, z up, gravity (0, 0, -9.81)).
The posture q and the velocities qdot are chosen by hand (not measured): they are only an example.
"""

from pathlib import Path

import biorbd
import numpy as np

HERE = Path(__file__).resolve().parent
MODEL_PATH = HERE.parents[1] / "examples" / "pyomecaman.bioMod"
SHOWN_NAMES = ["Pelvis_TransZ", "Pelvis_RotX", "BrasD_RotX", "CuisseD_RotX"]

model = biorbd.Biorbd(str(MODEL_PATH))
names = list(model.dof_names)
shown = np.array([names.index(n) for n in SHOWN_NAMES])
nq = model.nb_q

# Hand-chosen posture: right arm raised sideways, right hip and knee flexed (radians).
q = np.zeros(nq)
q[names.index("BrasD_RotX")] = 1.2
q[names.index("CuisseD_RotX")] = 0.6
q[names.index("JambeD_RotX")] = -0.9
q[names.index("PiedD_RotX")] = 0.3
# Hand-chosen velocities (rad/s and m/s), all degrees of freedom move.
qdot = np.array([0.3, 0.2, 1.0, 1.5, 2.0, -1.0, 1.0, 2.0, -2.5, 1.0, -1.0, 1.0, 0.5])

# CODE-BEGIN
g = model.non_linear_effect(q, 0 * qdot)
n1 = model.non_linear_effect(q, qdot)
n2 = model.non_linear_effect(q, 2 * qdot)
v1 = n1 - g
v2 = n2 - g
ratio = v2[shown] / v1[shown]
# CODE-END

# --- checks that are not displayed as code ---
# (1) sign of gravity: holding the pelvis up needs +mass*9.81 along the vertical translation.
mass = float(model.mass)
g_norm = float(-model.gravity[2])
weight = mass * g_norm
g_transz = float(g[names.index("Pelvis_TransZ")])
# (2) N(q, 0) = M g d(z_com)/dq, central finite differences on the centre of mass of the whole body.
eps = 1e-6
grad_z = np.zeros(nq)
for i in range(nq):
    dq = np.zeros(nq)
    dq[i] = eps
    grad_z[i] = (model.center_of_mass(q + dq)[2] - model.center_of_mass(q - dq)[2]) / (2 * eps)
g_from_com = weight * grad_z
err_gravity = float(np.abs(g - g_from_com).max())
# (3) N(q, qdot) is what inverse dynamics needs at zero acceleration. The pelvis has joint dampings in the
# bioMod (0.1, 0.2, 0.3), which inverse dynamics adds and NonLinearEffect does not: compare the other DoF.
id_minus_n = model.inverse_dynamics(q, qdot, np.zeros(nq)) - n1
err_id = float(np.abs(id_minus_n[3:]).max())
err_id_pelvis = float(np.abs(id_minus_n[:3] + np.array([0.1, 0.2, 0.3]) * qdot[:3]).max())
# (4) ratio over every degree of freedom where the velocity part is not negligible.
active = np.abs(v1) > 1e-6
ratio_all = v2[active] / v1[active]
ratio_error = float(np.abs(ratio_all - 4.0).max())
# (5) the same 4x law for another scale, 3 qdot -> 9x.
v3 = model.non_linear_effect(q, 3 * qdot) - g
ratio9_error = float(np.abs(v3[active] / v1[active] - 9.0).max())

np.savez(
    HERE / "data" / "nonlinear_effects_main.npz",
    dof_names=np.array(names),
    shown_names=np.array(SHOWN_NAMES),
    shown=shown,
    q=q,
    qdot=qdot,
    g=g,
    v1=v1,
    v2=v2,
    ratio=ratio,
    mass=mass,
    g_norm=g_norm,
    weight=weight,
    g_transz=g_transz,
    err_gravity=err_gravity,
    err_id=err_id,
    err_id_pelvis=err_id_pelvis,
    n_active=int(active.sum()),
    ratio_error=ratio_error,
    ratio9_error=ratio9_error,
)
print("names", names)
print("g", np.round(g, 3))
print("v1", np.round(v1, 3))
print("v2", np.round(v2, 3))
print("ratio(shown)", ratio, " n_active", active.sum(), " max|ratio-4|", ratio_error, " max|r3-9|", ratio9_error)
print(
    "mass*g",
    weight,
    " N_TransZ",
    g_transz,
    " err vs M g dz/dq",
    err_gravity,
    " err vs ID",
    err_id,
    err_id_pelvis,
    id_minus_n[:3],
)
