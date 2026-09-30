# Video 06 - centre of mass (notes)

## What was verified (biorbd 1.12.3, env `biorbd_anim`)
- `model.mass` is a property (`binding/python3/wrapper/biorbd_model.py`, line 63); value 52.41212 kg for `examples/pyomecaman.bioMod` (11 segments, `nb_q` = 13).
- `model.center_of_mass(q)` (`biorbd_model.py`, line 126) calls `Joints::CoM(Q, updateKin)` (`include/RigidBody/Joints.h`, line 680). Run OK.
- `segment.mass`, `segment.center_of_mass` (local, in the segment frame) and `segment.inertia` (3x3): `binding/python3/wrapper/segment.py`, lines 52-107; run OK (Pelvis inertia printed as a 3x3 diagonal matrix). The inertia is NOT shown in the video.
- `model.segment_frames(q)[name].world_translation` / `.world_rotation` (`segment_frame.py`): world position and orientation of the segment frame.
- Validation: world CoM of a segment = `R @ seg.center_of_mass + t` (R, t = world rotation and translation of the segment frame); `sum(m_i * com_i) / model.mass` was compared with `model.center_of_mass(q)` for two postures. The difference is exactly 0.0 in both (same floating-point operations as in the C++ sum, or at worst below 1e-16); the video displays it as `0.0e+00 m` on a difference axis whose full length is 1 mm.
- Postures: A = all q = 0 (standing), B = right arm raised (q[4] = -1.4), right leg q[7:10] = [0.6, -0.9, 0.3], left leg q[10:13] = [0.3, -0.4, 0.1].
- Key numbers (from `data/center_of_mass_main.npz`): total mass 52.41 kg; CoM A = (y 0.004, z -0.051) m; CoM B = (y 0.025, z -0.027) m; x is ~0 in both.
- The code panel is the body of the loop between `# CODE-BEGIN` / `# CODE-END` in `generate_center_of_mass_data.py` (rule D5); `q` is set just before it (q_a then q_b).

## Not verified / limits
- Sagittal view (y to the right, z up); x (about 0 by symmetry) is not shown, and the right/left limbs overlap in this projection.
- The drawing shows, for each segment, the link frame origin - centre of mass and the link parent origin - frame origin (parents read from the bioMod text by the generator); hands and head top are not drawn because the model has no end point for them. It is a schematic, not a mesh.
- The bars of the difference axis are invisible (value 0); only the printed `0.0e+00 m` carries the information.
- `examples/python3/manipulation_model.py` line 19 calls `model.mass()`; with this wrapper `mass` is a property, so that example line probably fails (not run by me, not used in the video). Worth a check by the lead.

## Checks
- Generator runs in `biorbd_anim` and asserts that `masses @ coms / masses.sum()` equals the displayed average and that the masses sum to `model.mass`.
- `validate_catalog.py` on a temporary catalog: 0 error.
- Final 1080p30 EN+FR renders: only the expected "no catalog.json entry" finding. Frames looked at (EN and FR): 1 s, 35 %, 65 %, before the card; nothing overflows or touches the logo. A first FR render had the legend overlapping a caption (not caught by the audit); fixed by splitting the legend into two lines.

## Consistency review (series pass)
- Code panel: a second, manual indentation shift (on top of the one done by `bio_style.code_panel`) doubled the nesting and pushed the `com = R @ seg.center_of_mass + t` line out of the box; removed, the panel now fits.
- The average is computed by a hand-written loop, so a "Hand-written code" label (rule D3) now sits under the code panel.
- Text column moved 0.3 to the left (x = -4.0) so that a French text 20 % longer still stops before the code panel.
