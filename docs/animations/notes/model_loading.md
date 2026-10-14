# Video 02 - model-loading (AnimModelLoading)

## What the video shows
`biorbd.Biorbd("examples/arm26.bioMod")` then the parts of the model object: nb_q / dof_names, segments, markers, muscles.
Real values (biorbd 1.12.3): 2 DoF (`r_humerus_rotation1_RotZ`, `r_ulna_radius_hand_rotation1_RotZ`), 11 segments
(including the helper translation/rotation segments of each joint), 5 markers, 6 muscles (TRIlong, BIClong, BICshort, TRIlat, TRImed, BRA).
Only the first 3 segment names are shown (followed by "..."), all other names are shown in full.

## Verified (biorbd_anim env, run by `generate_model_loading_data.py`)
- `Biorbd(path)`, `model.name` ('arm26'), `nb_q`, `dof_names`, `segments` (iterable of Segment with `.name`), `markers`, `muscles`
  (Muscle `.name`): checked in `binding/python3/wrapper/biorbd_model.py`, `segment.py`, `marker.py`, `muscle.py` and executed.
- The code panel is read from the generator between `# CODE-BEGIN` and `# CODE-END`; the generator does `os.chdir(repo root)` so the displayed relative path is the executed one.
- The path is passed as a str (the example code wraps it in f-strings / str()); a `Path` was not tested, so the video does not claim what happens with one.
- Counts and names on screen come from `data/model_loading_main.npz`.

## Not covered / not verified
- Contacts and IMUs are left out: no model with them was used or API-verified for this video.
- The "Biorbd (arm26)" label uses `model.name`; it is not the Python repr of the object.
- No synthetic data, nothing hand-written.

## Checks
- 480p15 EN iteration, then final 1080p30 EN+FR with --strict (only 'no catalog.json entry' allowed). Frames looked at: see the report.
- Catalog validated on a temporary catalog.
