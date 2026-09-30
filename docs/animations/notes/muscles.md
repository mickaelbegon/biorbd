# Video 11 - muscles (AnimMuscles)

## What was verified (biorbd 1.12.3, biorbd_anim environment)
- Model: `examples/arm26.bioMod`: 2 DoF (shoulder q[0], elbow q[1]) and 6 muscles (TRIlong, BIClong, BICshort, TRIlat, TRImed, BRA). TRIlong is `hillthelenfatigable`, BIClong is `hillthelen` (read in the file; type names checked in `src/ModelReader.cpp`). The video uses BIClong.
- API, read in `binding/python3/wrapper/muscle.py` and run: `model.muscles` (list, `["BIClong"]` by name works), `muscles.update_geometry(q, qdot)`, `muscle.muscle_tendon_length`, `muscle.length_jacobian` (shape (1, 2)), `muscles.forces(activations, q, qdot)` (shape (6,)), `muscles.joint_torque(activations, q, qdot)` (shape (2,)).
- `muscle.force` / `forces` need the geometry updated with BOTH q and qdot (update_geometry(q) alone raises "Geometry must be computed before calling velocity()"), hence qdot = 0 in the generator.
- Sign convention verified numerically: over 41 elbow angles (shoulder 0.3 rad, all activations 0.5) `joint_torque` equals `-J.T @ forces` with J the stacked `length_jacobian` (max difference 5.3e-15 N.m); `+J.T @ forces` differs by up to 35 N.m. The C++ doc of `Muscles::muscularJointTorque` (include/InternalForces/Muscles/Muscles.h) and `examples/python3/manipulating_muscles.py` also say -J^T F.
- `length_jacobian` is the derivative of the length: compared with a central finite difference of `muscle_tendon_length` along the sweep, the largest gap over interior points is 3.6e-5 m/rad (finite-difference error). `muscle.length` (0.06-0.14 m) and `muscle_tendon_length` (0.34-0.41 m) have the same derivative here; the video shows the muscle-tendon length. I did not investigate the exact definition of `length` versus the tendon part.
- The video shows the moment arm as the opposite of dLength/dq (BIClong: 1.6 to 4.9 cm, all positive, the flexor torque sign being positive with -J^T F). I only verified this sign for this model, not as a general statement about biorbd joint conventions.
- Data: `generate_muscles_data.py` -> `data/muscles_main.npz`. The displayed code is read between its CODE-BEGIN/CODE-END markers (D5). The elbow sweep (0.2 to 2.2 rad) is chosen, not measured; it is not labelled with the synthetic tag because it is a plain parameter sweep of real biorbd calls, the scene says so in the caption "Elbow angle from ... to ...". Activation 0.5 is applied to all six muscles, so the torque curve is that of the six muscles together (including triceps), not only of BIClong.
- Catalog entry validated with `tools/validate_catalog.py` (0 error); links checked with grep -n / sed.
- 1080p30 EN and FR renders with --strict: only finding is "no catalog.json entry"; no missing French key. Frames looked at (1080p30): EN before the card; FR end of stage 1 (dots and read-out) and before the card. Earlier 480p15 EN: 35 %, 65 %, end of stage 1, before the card. Not looked at in 1080p: the 1 s frames and the card middle (card empty until catalog merge).

## Not verified / open
- The difference curve is plotted scaled to its own maximum (5.3e-15 N.m, rounding noise); its caption gives the real maximum.
- Muscle force as a function of length and activation is not plotted (only the resulting torque); the video does not claim the Hill-Thelen force-length shape.
- Wrapping objects, via points and the fatigable state of TRIlong are not discussed.
- Whether `muscles.joint_torque` keeps the same sign for models with other joint types was not tested.
