"""Data for the video 12 (contacts): forward dynamics with and without the rigid contacts of the model.

Run with the biorbd_anim environment:  python generate_contacts_data.py
The lines between the CODE-BEGIN and CODE-END markers are displayed as is in the code panel (rule D5).
"""

from pathlib import Path

import biorbd
import numpy as np
from biorbd.wrapper.misc import to_biorbd_array_output

HERE = Path(__file__).resolve().parent
model_path = str(HERE.parents[1] / "test" / "models" / "cubeWithRigidContactsExternalForces.bioMod")

# CODE-BEGIN
m = biorbd.Biorbd(model_path)
q, qd, tau = np.zeros(4), np.zeros(4), np.zeros(4)
free = m.forward_dynamics(q, qd, tau, ignore_contacts=True)
qdd, forces = m.forward_dynamics(q, qd, tau, return_contact_forces=True)
a_free = m.internal.rigidContactsAcceleration(q, qd, free)
a = m.internal.rigidContactsAcceleration(q, qd, qdd)
# CODE-END

acc_free = np.array([to_biorbd_array_output(x) for x in a_free])  # (n_contact_points, 3)
acc = np.array([to_biorbd_array_output(x) for x in a])
n_constraints = m.internal.nbContacts()
constraint_names = [m.internal.contactName(i).to_string() for i in range(n_constraints)]
mass = float(m.mass)
np.savez(
    HERE / "data" / "contacts_main.npz",
    dof_names=np.array(m.dof_names),
    qddot_free=free,
    qddot=qdd,
    forces=forces,
    constraint_names=np.array(constraint_names),
    acc_free=acc_free,
    acc=acc,
    weight=mass * 9.81,
    sum_z=forces[1:].sum(),
)
print("qddot_free", free, "\nqddot", qdd, "\nforces", forces, constraint_names)
print("acc_free", acc_free, "\nacc", acc, "\nweight", mass * 9.81)
