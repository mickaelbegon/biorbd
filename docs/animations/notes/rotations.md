# Video 04 - rotations (notes)

## What was verified (biorbd 1.12.3, env `biorbd_anim`)
- `biorbd.Rotation.fromEulerAngles(angles, seq)` and `biorbd.Rotation.toEulerAngles(rotation, seq)`: declared in `include/Utils/Rotation.h` (lines 123, 175), exposed by SWIG (`dir(biorbd.Rotation)`), run OK. Results are SWIG objects; `.to_array()` gives numpy.
- `biorbd.Quaternion.fromMatrix(rotation)` and `.toMatrix()`, `.w() .x() .y() .z()`: `include/Utils/Quaternion.h`, run OK (w is the scalar part).
- The generator checks independently with numpy that "xyz" means R = Rx(a0) Ry(a1) Rz(a2) and "zyx" means R = Rz(a0) Ry(a1) Rx(a2), that toEulerAngles returns the input angles in the same sequence, that the quaternion has unit norm and that quaternion.toMatrix() equals the matrix.
- Displayed code = the two blocks between `# code:start` / `# code:end` in `generate_rotations_data.py`, read by the scene (rule D5). The second block continues the first (`Rot`, `r_xyz` already defined); the quaternion class is aliased `Q` to keep the line short.
- All numbers (angles, two matrices, largest matrix difference, quaternion, recovered angles) are formatted from `data/rotations_main.npz`.

## Limits
- The drawing is an oblique projection of the world axes (x toward the viewer, y right, z up); the coloured arrows are the columns of each matrix. Grey faded axes are the world frame. It is a schematic view, not a measured scene.
- Only the xyz quaternion is shown; the zyx round trip was checked in the generator (stored as `back_zyx`) but is not displayed.
- Gimbal lock and angle ranges (toEulerAngles returns angles inside the principal range) are not covered.
- No new conversion was needed: everything shown exists in the Python API.

## Checks
- 480p15 EN render with `--strict`: no audit finding (only the expected missing catalog entry). Frames looked at (EN): 1 s, 35 %, before card.
- `validate_catalog.py` on a temporary catalog with the entry: 0 error.
