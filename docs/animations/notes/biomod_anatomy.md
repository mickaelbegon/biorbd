# Video 01 - anatomy of a .bioMod file (notes)

## What was verified (biorbd 1.12.3, env `biorbd_anim`)
- Model: `examples/pyomecaman.bioMod` (loaded with `biorbd.Biorbd(str(path))`), Pelvis segment and marker `pelv1`.
- Excerpts shown in the video are real lines of that file (line numbers 1, 9-11 and 22, 27, 35, 38-41), copied by the generator and stored in the `.npz`
  (`rows_a`, `rows_b`). Only the common left indentation of each excerpt is removed, and `...` rows mark skipped lines (display only).
- The generator asserts that the excerpt text matches what biorbd returned: `seg.translations == "yz"`, `seg.rotations == "x"`,
  `model.dof_names[:3] == [Pelvis_TransY, Pelvis_TransZ, Pelvis_RotX]`, `seg.mass` equals the `mass` line, `seg.center_of_mass` the `com` line,
  the marker `local` the `position` line.
- API read in `binding/python3/wrapper/segment.py` (translations, rotations, mass, center_of_mass), `biorbd_model.py` (segments, dof_names, nb_q),
  `marker.py` (local); all run OK. `model.markers(q)[name]` as in the forward-kinematics video.
- Tags checked in `README.md` (segment 452-494, translations/rotations order of q 484-488, mass in kg 490, com in the segment frame 493, marker 523-539)
  and `src/ModelReader.cpp` (segment 122, translations 160, rotations 171, mass 217, com 255-259, marker 410, parent 421, position 430).
- Displayed code = lines between `# code:start` / `# code:end` of `generate_biomod_anatomy_data.py`. The scene shows the first 5 lines with the
  first point and all 9 with the second.

## Not verified / limits
- The scene only shows 3 of the 13 coordinates of the model (those of the Pelvis); `nb_q` = 13 is stored but not shown.
- The mapping "tag -> q name" (`Pelvis_TransY`...) is shown as biorbd's output; I did not read the naming code.
- The `gravity` tag is not in this file and is not shown.
- The link to `examples/pyomecaman.bioMod` lines 1-41 covers the version tag, the Pelvis segment and the first marker.

## Checks
- `validate_catalog.py` on a temp catalog containing this entry; 480p15 and 1080p30 EN+FR renders with `--strict`; frames looked at (see final report).

## Open issues
- catalog.json has no entry until the lead merges the part (the final card is empty until then).
