# Video 09 - inverse dynamics (AnimInverseDynamics)

## Verified (biorbd 1.12.3, biorbd_anim)
- `model.inverse_dynamics(q, qdot, qddot)`, `model.mass_matrix(q)`, `model.non_linear_effect(q, qdot)` read in binding/python3/wrapper/biorbd_model.py (lines 355, 73, 101) and run by generate_inverse_dynamics_data.py. non_linear_effect requires q and qdot.
- Model: examples/arm26.bioMod (nb_q = 2). Motion: SYNTHETIC sinusoids, 101 frames over 1 s, analytic qdot and qddot (labelled on screen).
- Result: max |tau - (M qddot + N)| = 1.8e-15 N.m (peak |tau| = 9.0 N.m), i.e. machine precision. Shown values come from data/inverse_dynamics_main.npz.
- Displayed code = lines between CODE-BEGIN / CODE-END of the generator.
- Catalog entry validated (0 error). Renders EN+FR 480p15 --strict: only "no catalog.json entry". Frames looked at (EN and FR).

## Not verified / open
- Model has no external forces; identity with external forces not tested. Gravity is on (default), so N includes gravity.
- The difference curve is floating-point noise; its shape has no meaning.

## Consistency review (series pass)
- Torque axes 0.2 shorter and moved down 0.1: the caption above them touched the top of the curves.
- "Torques in N·m." became the sentence "All torques are in N·m."; the unused `font=None` path of the label helper was removed.
- Open point: `inverse_dynamics` is drawn in C_MODEL in the first stage and in C_DATA (reference of the check) in the second; the time axis has no tick labels.
