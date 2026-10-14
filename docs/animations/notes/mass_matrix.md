# Video 07 - mass matrix (AnimMassMatrix)

## What was verified (biorbd 1.12.3, biorbd_anim environment)
- `Biorbd.mass_matrix(q, inverse=False)` (binding/python3/wrapper/biorbd_model.py, lines 73-99) calls `massMatrix(Q, updateKin)` or, with `inverse=True`, `massMatrixInverse` (include/RigidBody/Joints.h, 932 and 942). Confirmed by running it on `examples/arm26.bioMod` (`nb_q == 2`).
- Also checked by running: `mass_matrix(q, inverse=True)` equals `np.linalg.inv(mass_matrix(q))` (max difference 4e-15). The inverse is NOT shown in the video; the generator only asserts it.
- Model: `examples/arm26.bioMod`, q[0] shoulder flexion (z rotation of r_humerus), q[1] elbow. Poses A q = [0.3, 0.3] rad and B q = [0.3, 1.8] rad (chosen by hand, not synthetic data).
- Numbers in the .npz (`data/mass_matrix_main.npz`): M_A = [[0.4292, 0.1471], [0.1471, 0.0705]], M_B = [[0.2388, 0.0527], [0.0527, 0.0705]], change = [[-0.1905, -0.0945], [-0.0945, 0]]. Symmetry error 0 (exactly, both poses). Smallest eigenvalue 0.0179 (A) and 0.0554 (B), both > 0, from `np.linalg.eigvalsh`. M22 (forearm about the elbow) does not depend on q here; this is physical for this model.
- The code panel reads the two `# code:start` / `# code:end` blocks of `generate_mass_matrix_data.py` (D5). The arm drawing uses the marker world positions (r_acromion, r_humerus_epicondyle, r_radius_styloid) projected on the x-y plane; it is a schematic of the stick figure, not a meshed model.
- Catalog entry validated with `tools/validate_catalog.py` on a temporary copy: 0 error.

## Not verified / open
- Positive definiteness is shown for two poses only (a numerical check, not a proof); the video says "symmetric and positive definite" as a property of M(q), which is a general result for rigid-body systems without singular inertias.
- The formula tau = M(q) qddot + ... is only alluded to ("inertial part of the joint torques"); the other terms belong to videos 08/09.
- The eigenvalues at pose A (0.018 and 0.482) differ by a factor of about 27; conditioning is not discussed in the video.

## Consistency review (series pass)
- Title shortened to "Mass matrix" (no article, like the other titles of the series).
- Pose A: the symmetry / eigenvalue / meaning lines now start at the left margin (x = -6.8) below the arm, because a French text 20 % longer ran into the code panel.
