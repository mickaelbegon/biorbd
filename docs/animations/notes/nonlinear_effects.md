# Video 08 - nonlinear effects (AnimNonlinearEffects)

## What was verified (biorbd 1.12.3, biorbd_anim environment)
- `Biorbd.non_linear_effect(q, qdot)` (binding/python3/wrapper/biorbd_model.py:101) wraps `Joints::NonLinearEffect` (include/RigidBody/Joints.h:1148, src/RigidBody/Joints.cpp:1543, calls RBDL `NonlinearEffects`). Both q and qdot are mandatory in the wrapper (NotImplementedError otherwise). Confirmed by running it.
- There is NO separate Coriolis / centrifugal function in the Python wrapper (grep for "coriolis" in binding/python3: nothing). The video says so on screen. Gravity alone is obtained only by calling with qdot = 0.
- Model: `examples/pyomecaman.bioMod` (13 DoF, z up, gravity (0, 0, -9.81), mass 52.41 kg). arm26 was rejected: its gravity is along z while the arm hangs along y and rotates about z, so the sign of the gravity torque is not intuitive there.
- Posture (right arm raised 1.2 rad, right hip 0.6, knee -0.9, foot 0.3) and qdot are chosen by hand; the on-screen text and the "Synthetic data" tag say so.
- Sign check: N_TransZ(q, 0) = 514.16 = mass x 9.81 (positive: the force needed to hold the pelvis up). For all 13 DoF, N(q, 0) equals mass x 9.81 x d(z_com)/dq (central finite differences of `center_of_mass`): max error 3e-9.
- Quadratic law: v(s) = N(q, s qdot) - N(q, 0). v(2)/v(1) = 4 on all 13 DoF, max |ratio - 4| = 1.7e-13; v(3)/v(1) = 9 within 4e-13 (stored, not shown).
- Inverse dynamics at zero acceleration equals N(q, qdot) exactly on the non-pelvis DoF; on the pelvis it differs by -jointdampings x qdot (0.1, 0.2, 0.3 in the bioMod), which is not part of NonLinearEffect (stored as err_id, err_id_pelvis).
- The code panel reads the lines between `# CODE-BEGIN` / `# CODE-END` of generate_nonlinear_effects_data.py (D5). Numbers come from data/nonlinear_effects_main.npz.
- Catalog entry validated with tools/validate_catalog.py on a temp catalog: 0 error.

## Not verified / open
- The "Go further" card was not checked with content (catalog.json not merged).
- Units on screen: the vertical force is labelled N; the other rows are torques (N.m) and the table has no unit column.
- Joint dampings and muscle forces are not part of non_linear_effect; external forces are ignored (none defined).

## Consistency review (series pass)
- Code panel at 16 pt (was 18 pt): at 18 pt the box came within 0.15 units of the right edge of the frame.
- The two weight / velocity sentences moved down by 0.15 so that a French text 20 % longer does not touch the bottom of the code panel.
- Open point: the velocity part (N(q, qdot) - N(q, 0)) is drawn in C_DIFF because it is literally a difference of two biorbd outputs; it is not an error.
