# Video 05 - jacobians (AnimJacobians)

## What was verified (biorbd 1.12.3, biorbd_anim environment)
- Model: `examples/arm26.bioMod`, loads with `biorbd.Biorbd(path)`, `nb_q == 2`. Marker index 3 (`r_radius_styloid`) is on the forearm and depends on both DoF (marker 0 is fixed to the thorax: its Jacobian is all zeros).
- `model.markers[i].jacobian(q)` (binding/python3/wrapper/marker.py, `Marker.jacobian`) returns a 3 x nb_q numpy array; confirmed by running it.
- `model.markers[i].forward_kinematics(q)` returns the world position (3,); `model.markers(q)[i].world` also works but the generator uses `forward_kinematics(q)` so that each call is explicit about q.
- `data/jacobians_main.npz` is written by `generate_jacobians_data.py`; the code panel reads the lines between `# CODE-BEGIN` and `# CODE-END` of that script, so the displayed code is the executed code (D5).
- Trajectory: synthetic sinusoids on the two joints (labelled on screen), 101 frames over 1 s, analytic qdot. Finite differences are central (`(p1 - p0) / (2 * dt)`) on frames 1..99. Largest difference between J qdot and finite differences: 0.0018 m/s for a peak speed of 1.78 m/s (about 0.1 %, the O(dt^2) error of central differences).
- Catalog entry validated with `tools/validate_catalog.py --catalog <temp copy containing the entry>`: 0 error. Links checked with `grep -n` (marker.py 172-222, Markers.h 554-575, test_wrapper_markers.py 84-103, forward_kinematics.py 39-44).
- Rendering EN and FR at 1080p30 with `--strict`: the only finding is "no catalog.json entry" (catalog.json is still empty, expected). No missing French key.
- Frames (1 s, 35 %, 65 %, before the card, middle of the card) of both languages extracted and looked at.

## Not verified / open
- The final "Go further" card was not checked with real content: catalog.json is empty, so it shows the heading only.
- The Jacobian is the 3D world-frame position Jacobian (biorbd `markersJacobian`, `removeAxis` default); axes removed by the model (`axis` option in bioMod) would change the rows, not the case for arm26.
- Whether v = J qdot holds for markers of models with dependent (constrained) coordinates was not tested.
