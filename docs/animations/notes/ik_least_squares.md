# Video 14 - ik-least-squares (AnimIkLeastSquares)

## What the video shows
`biorbd.InverseKinematics(model, markers).solve(method=...)` on SYNTHETIC data (labelled): a squat-like motion q(t) of
`examples/pyomecaman.bioMod` (13 q, 97 markers, 61 frames), markers = biorbd forward kinematics + Gaussian noise (2 mm per coordinate, seed 14).
Point 1: lm / trf / only_lm on the complete data: RMSE of q against the truth (0.250 / 0.248 / 0.251 deg) and the largest excess over the joint
ranges (0.595 / 0.000 / 0.595 deg). Point 2: marker `jambed4` set to NaN for frames 20-40 (lm): the solver skips it; the error of that marker
(model position vs truth, the red ring) goes from 0.74 to 0.87 mm in the gap and the error over all markers from 0.74 to 0.75 mm.
All numbers are read from `data/ik_least_squares_main.npz`.

## Verified (biorbd 1.12.3, `biorbd_anim` env, run in `generate_ik_least_squares_data.py`)
- `biorbd.Model(path)` (raw model, not the `Biorbd` wrapper), `model.nbQ()`, `nbMarkers()`, `markerNames()[i].to_string()`,
  `model.markers(q)` (list; `.to_array()`), `model.globalJCS(q, i).trans().to_array()`, `biorbd.utils.get_range_q(model)`.
- `biorbd.InverseKinematics(model, markers)`: markers of shape (3, nb_marker, nb_frame), in metres; `.solve(method)` returns q of shape
  (nb_q, nb_frame); `.list_sol[i].nfev` exists (scipy OptimizeResult). Read in `binding/python3/rigid_body.py` lines 154-394 and run.
- NaN handling: `_get_nan_index` keeps only finite markers per frame; confirmed by the run (markers_nan solves fine, no NaN in q).
- Method semantics (docstring + code): `lm` uses `trf` (bounded) on the first frame then `lm` (no bounds) on the others; `trf` uses bounds on all
  frames; `only_lm` never uses bounds. Confirmed by the numbers: lm and only_lm exceed the ranges by 0.595 deg, trf never.
- Joint dampings: the biorbd test sets `segment(0).setJointDampings([0,0,0])`. I ran all three methods with and without it on the test pose:
  the solutions were identical (difference 0.0 to display precision). Dampings only enter the dynamics, so the call is not needed for IK and is
  not copied. The real reason is not documented in the test; it is only my observation that IK is unaffected.
- Observed pitfall (not shown in the video): in the test, `qinit` has values outside the model ranges (knee +0.1 vs range [-2.09, 0]); with
  `lm` or `trf` the solver then stops at a bounded solution 0.1 rad away (the test only checks to 1 decimal), `only_lm` recovers it exactly.
- The displayed code (2 + 2 lines) is read from the generator between `# CODE-BEGIN/END` and `# CODE2-BEGIN/END`, so it is the code that ran.
  The panel shows `method=method` (a variable of the generator function); the video says which methods exist in the table.
- Catalog entry validated with `tools/validate_catalog.py` (0 error); links checked with grep -n.

## Not verified / limits
- Synthetic motion, noise and missing marker; no real c3d. The missing marker has 5+ neighbours on the same segment, so its loss barely changes
  the solution (honest result: 0.74 -> 0.75 mm). Losing most markers of a segment was not tested.
- Only one seed and one motion. The 0.595 deg excess comes from the knee being at its bound (0) at the start/end of the squat with noise.
- `biorbd_casadi` backend not covered (the class is skipped for casadi in the test).
- The drawing is a y-z projection; the arms are out of plane partially, so the stick figure is a simplified view built from segment origins and
  a few markers (hand marker `brasd17`, head = mean of `tete*`).
- Error plotted for the missing marker is the model position vs the true position, not a measured value.

## Open issues
- `--strict` reports only "no catalog.json entry" until the lead merges the catalog.

## Review pass (consistency)
- Title is now "Inverse kinematics in biorbd" (subtitle "Least squares with the InverseKinematics class"): the former French title overflowed the frame with +20 % text and left no free corner for the logo. The on-screen title differs from the catalog title, which is unchanged.
- Text column moved left by 0.4 (more room before the code panel), code panel 15 pt with right margin kept, error column and error ghost curve now in C_DIFF (errors), the last sentence is a whole sentence, FR "Pose estimée" -> "Posture estimée" (glossary usage of the other scenes).
- Stress test (French +20 %, scratch runner, 480p15): audit clean.
