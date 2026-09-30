# Video 03 - forward kinematics (notes)

## What was verified (biorbd 1.12.3, env `biorbd_anim`)
- `examples/pyomecaman.bioMod` loads with `biorbd.Biorbd(path)`; `path` must be a `str` (a `Path` raises a TypeError in `new_Model`).
- `model.nb_q` = 13; q order: Pelvis (TransY, TransZ, RotX) = 0-2, BrasD = 3-4, BrasG = 5-6, CuisseD = 7, JambeD = 8, PiedD = 9, left leg = 10-12.
  The right leg (Pelvis > CuisseD > JambeD > PiedD) moves in the y-z plane (rotations about x), hence the 2D picture.
- `model.segment_frames(q)["PiedD"].world_translation` and `.world_rotation`: read in `binding/python3/wrapper/segment_frame.py`, run OK.
- `model.markers(q)["piedd1"].world` / `.local`: `binding/python3/wrapper/marker.py`, run OK; `piedd1` belongs to segment PiedD.
- The generator asserts that the foot origin and the marker returned in the two code blocks equal the stored arrays.
- Displayed code = the two blocks between `# code:start` / `# code:end` in `generate_forward_kinematics_data.py`, read by the scene at render time (rule D5).
- All numbers on screen (foot origin, marker local and world, q of the leg, marker difference) are formatted from `data/forward_kinematics_main.npz`.

## Not verified / limits
- The scene draws y to the right and z up (sagittal view); x (normal to the plane) is not shown.
- "Each frame sits on its parent" is the description of the chain of `globalJCS`; I did not display the matrix product, because the stored local frame excludes the q-dependent rotation.
- The blocks in the panel are two consecutive runs in one script (second block continues from the first, so `model` and `q` already exist).

## Checks
- `render_series.py AnimForwardKinematics --lang en/fr`: no audit finding, no missing key (only the expected "no catalog.json entry" in strict mode).
- Frames extracted and looked at for EN and FR (480p15 during development, 1080p30 final).
- `validate_catalog.py --catalog <temp copy containing catalog_part_forward_kinematics.json>`: 0 error.

## Open issues
- catalog.json is still empty, so the final "Go further" card has no links until the part is merged.
