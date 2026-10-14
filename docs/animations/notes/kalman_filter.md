# Video 16 - kalman-filter (AnimKalmanFilter)

## What the video shows
Synthetic motion of the 2-DoF arm of `examples/arm26.bioMod` (q_true known, 151 frames at 100 Hz, fixed seed 16);
"measured" markers = biorbd forward kinematics + Gaussian noise of 2 mm per coordinate.
Point 1: solving each frame alone (hand-written Gauss-Newton, same step as video 15, cold start q = 0) gives RMSE 0.43 deg.
Point 2: `biorbd.ExtendedKalmanFilterMarkers` gives RMSE 0.35 deg (0.33 deg after the first 20 frames).
Both are compared with the truth on a difference axis (ghost = per-frame estimate, faded). All numbers come from `data/kalman_filter_main.npz`.

## Tuning (honest)
- noise_factor = 1e-10 and error_factor = 1e-5: the biorbd defaults (`include/RigidBody/KalmanRecons.h`), NOT tuned. frequency = 100 Hz = the real sampling rate of the synthetic data.
- q_init/qdot_init/qddot_init left out (None -> filter initialises itself; first frame is slow, as noted in the test). Starting from zeros gave the same final RMSE as starting from the true q.
- Exploration done while developing (same data, not stored): noise_factor 1e-8 -> RMSE 0.83-1.0 deg; 1e-6 -> 3.5-5.5 deg; 1e-3/1e-3 -> 12 deg. Larger noise_factor = the filter trusts the markers less, so it lags the motion; the default is the best of those tried. Only a coarse grid was tried; no claim of optimality.
- The gain over the per-frame solution is modest (about 18 % in RMSE) because the noise is small (2 mm) and the motion is smooth and only 2 DoF with 4 informative markers. Velocity: RMSE 27.8 deg/s (filter qdot) vs 28.9 deg/s (finite differences of the per-frame q), i.e. nearly equal; not shown in the video.

## Verified (biorbd 1.12.3, `biorbd_anim` env)
- `biorbd.has_extended_kalman_filter` is True; `biorbd.ExtendedKalmanFilterMarkers(model, frequency, noise_factor, error_factor, ...)`, `reconstruct_frame(s)` read in `binding/python3/wrapper/extended_kalman_filter.py`; markers must be shaped (3, n_markers) (hence `.T`) and count must equal the technical markers (5 in arm26, including the fixed acromion).
- Defaults read in `include/RigidBody/KalmanRecons.h`; behaviour cross-checked with `test/binding/python3/tests_wrapper/test_wrapper_kalman_filter.py` and `examples/inverseKinematicsKalmanExample.cpp` (C++, not run).
- Displayed code = lines between CODE-BEGIN/CODE-END of the generator (executed). Constants FREQ, NOISE, ERROR are shown under the code from the npz.

## Not verified / limits
- Only one seed, one noise level, one model. No real data. The C++ example was not compiled/run. Gain could differ for other motions.
- Displayed names NOISE/ERROR are the script's constants for noise_factor/error_factor (short to fit the panel; positional arguments).
- Indentation is not rendered by the Text layer (leading spaces lost), so the displayed code is kept flat (no loop body).

## Checks
- black on both .py; catalog validated on a temporary catalog (0 error); EN/FR renders --strict; frames looked at (see final report).

## Review pass (consistency)
- Code panel shifted left to keep a right margin; FREQ/NOISE/ERROR lines are now one multi-line Text (even line spacing); result sentence says "the error" (not "it") because the first sentence is replaced; sentences left-aligned at x = -6.8, up to 7.4 units wide.
- The "Hand-written code" label refers to the per-frame Gauss-Newton comparison curve (not shown in the code panel); the code panel itself is biorbd's own filter.
- Stress test (French +20 %, scratch runner, 480p15): audit clean.
