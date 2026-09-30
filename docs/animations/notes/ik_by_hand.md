# Video 15 - ik-by-hand (AnimIkByHand)

## What the video shows
Hand-written Gauss-Newton inverse kinematics on the two-degree-of-freedom arm of `examples/arm26.bioMod`, with synthetic markers.
Point 1: iterations on one frame (two drawn with ghosts, then the converged pose after 7) (marker error 343 mm at iteration 0, values read from the npz) with ghost poses and an error axis.
Point 2: the loop on 101 frames, compared with the known true q (ghost = one step only, difference axis = joint-angle error).

## Verified (biorbd 1.12.3, `biorbd_anim` env)
- `biorbd.Biorbd(path)`, `model.nb_q` (= 2 for arm26), `model.markers` (5 markers, all technical), `model.markers(q)` (updates kinematics and
  returns the list), `Marker.world`, `model.markers.jacobian(q)` (list of 3x2 arrays): all run in `generate_ik_by_hand_data.py`
  and match `binding/python3/wrapper/marker.py` (lines 62, 172, 199, 255, 266).
- Jacobian not degenerate at the true pose (singular values stored in the npz as `singular_values`, both non-zero).
  The acromion marker is fixed to the ground (zero Jacobian, zero residual) and is not drawn.
- The code panel is read from the generator between `# CODE-BEGIN` and `# CODE-END`, so it is the executed code.
- Convergence: cold start q = (0, 0) on all frames converges in at most 8 iterations (stored in `n_iter`).
- Numbers on screen (marker error per iteration, iteration count, RMSE in degrees) come from `data/ik_by_hand_main.npz`.
  The marker error shown is the RMS distance over the four moving markers, computed in the scene from the stored per-marker distances.
- The synthetic and hand-written labels are both shown (bottom left, side by side).
- Catalog links checked with grep -n / validate_catalog on a temporary catalog.

## Not verified / limits
- Only planar motion (rotations about z): the z rows of the Jacobian are zero; the pinv handles it. No 3D case tested.
- The reconstruction error (RMSE) includes the effect of the added noise (2 mm per coordinate, seed 15); it is not zero by design.
- The ghost curve of point 2 is the one-iteration estimate, not the previous value of the same quantity in time; the difference axis
  shows converged minus truth (joint-angle error), which is what the message needs.
- Real data, joint limits and weighting are not covered.
- The notebook of pyomeca/biorbd-tutorial was not consulted (no network access needed); the algorithm was written from scratch.

## Open issues
- `--strict` reports "no catalog.json entry" until the catalog part is merged (expected).

## Checks performed
- black -t py311 -l120 on both .py files; 480p15 and 1080p30 renders EN+FR with --strict: no audit finding, no missing key;
  duration 37.9 s (EN) / 40.0 s (FR) = about 19.5 s native content + 2.5 s hold + 4.5 s card.
- Frames looked at (EN and FR, 1080p30): 1 s, 35 %, 65 %, before the card, middle of the card; nothing overlaps the logo.
