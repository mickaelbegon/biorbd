# Video 12 - contacts (AnimContacts)

Idea chosen: (a) a rigid contact defined in the .bioMod, forward dynamics with contacts.

## What was verified (biorbd 1.12.3, biorbd_anim environment)
- Model: `test/models/cubeWithRigidContactsExternalForces.bioMod` (read-only): one segment, 4 dof (TransX/Y/Z, RotY), mass 1 kg, two rigid contacts: PiedG_1 (axes y z) and PiedG_2 (axis z). 2 contact points, 3 constraints (`nbContacts()` = 3, names PiedG_1_Y, PiedG_1_Z, PiedG_2_Z; `nbRigidContacts()` = 2).
- `model.forward_dynamics(q, qdot, tau, ignore_contacts=True)` and `..., return_contact_forces=True` (wrapper `biorbd_model.py` 297-358) run. No external force is set (the file name only says it could take some).
- `model.internal.rigidContactsAcceleration(q, qdot, qddot)` (include/RigidBody/Contacts.h) accepts numpy arrays directly and returns one Vector3d per contact point; converted with `biorbd.wrapper.misc.to_biorbd_array_output` (done after the displayed lines, not shown). It is also used in test/test_rigidbody.cpp 361-378.
- Results at q = qdot = tau = 0: free qddot = [0, 0, -9.81, 0]; with contacts qddot ~ 1e-13 (all zero); contact point accelerations: free z = -9.81, with contacts <= 2e-14 m/s^2 (all three components, both points; contact 1 has axes y z only, but x is also ~0 here because nothing moves along x); forces = [~0, -74.2, 84.0] N (PiedG_1_Y, PiedG_1_Z, PiedG_2_Z). The z forces sum to 9.81 N = m g.
- The displayed code is read between CODE-BEGIN/CODE-END of `generate_contacts_data.py` (D5). Names are shortened (m, qd, qdd) so that the lines fit the panel.

## Not verified / open
- A non-static pose was tried on the raw model (q = [.1 .2 .3 .4], qdot nonzero): contact y/z accelerations are ~1e-14 and the free axis (x of contact 1) moves as expected, but this is not shown in the video.
- The two z contacts are nearly redundant at q = 0 (same z constraint on one rigid body): the individual forces (-74.2 N, 84.0 N) are not physically unique; only their sum equals the weight. The video says "sum", not "each contact carries".
- The ExternalForceSet alternative (b) was not used.
- The contact names are not displayed. The `test_wrapper_dynamics.py` test only checks ignore_contacts=True, so it is not linked.
