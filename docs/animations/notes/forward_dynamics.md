# Video 10 - forward-dynamics (AnimForwardDynamics)

## What the video shows
Passive double pendulum on a sliding support (`test/models/pendulum.bioMod`, 3 dofs, gravity (0, 0, -9.81), tau = 0), released at rest from q = (0, 0.3, 0.2).
Point 1: `model.forward_dynamics(q, qdot, tau)` returns qddot (23.4, -24.5, 25.5 at t = 0, read from the npz); biorbd does not integrate.
Point 2: hand-written explicit Euler and RK4, same step (0.01 s, 2 s): relative drift of the total energy, (E - E0) / |E0| in %.
Euler: max |drift| 35.64 %; RK4: 0.02 % (0.0177). Ghost = Euler (opacity 0.3), difference axis = Euler minus RK4.

## Verified (biorbd 1.12.3, `biorbd_anim` env)
- `Biorbd.forward_dynamics(q, qdot, tau)` (binding/python3/wrapper/biorbd_model.py:297), `nb_q`, `nb_tau`, `gravity`, `mass_matrix`: run in the generator.
- Energy: `model.internal.TotalEnergy(q, qdot)` (include/RigidBody/Joints.h:1115) IS reachable from Python through `model.internal`
  (the Python wrapper has no energy method); it accepts numpy arrays. Cross-check in the npz: 0.5 qdot' M(q) qdot from `model.mass_matrix` equals
  `model.internal.KineticEnergy` (3.676281 both). Potential energy not cross-checked independently.
- Model has no damping (no `damping` keyword in pendulum.bioMod, no muscles). Energy is conserved by the physics: RK4 drift goes to ~1e-9 % with dt = 1e-4 s (scratch test, not stored).
- Code panels are read from the generator between `# CODE1-BEGIN/END` and `# CODE2-BEGIN/END` (blank lines are not displayed, indentation is kept).
- "biorbd has no integration code in its core": not searched exhaustively; the video only says that biorbd does not integrate it for you (forward_dynamics returns qddot only).

## Not verified / limits
- Pendulum model oddities: Seg1 has a y-translation and an x-rotation; the markers of Seg2 in the file are declared with parent Seg1 (irrelevant here, markers unused).
- Explicit Euler is unstable for larger steps here (dt = 0.02 s diverges to inf over 2 s); the chosen dt = 0.01 s keeps it finite.
- The left area is nearly empty during point 1 (only text); no pendulum drawing (no geometry was computed).
- biorbd-tutorial notebook 6 was not consulted.

## Checks performed
- black on both .py files; validate_catalog on a temporary catalog: 0 error; 1080p30 EN+FR --strict (see report); frames looked at.

## Consistency review (series pass)
- The two bottom labels ("Synthetic data", "Hand-written code") are now on the same baseline.
- Sentences are 22 pt and may be 6.8 wide (they were shrunk silently to about 20 pt); the Euler / RK4 key moved right so it no longer meets the French axis title; "Time (s)" no longer touches the axis; the difference axis title now carries its unit: "Euler minus RK4 (%)".
- Open point: this scene keeps its own `code_panel` (keeps indentation, 16 pt) instead of `bio_style.code_panel`, because the shared one sizes its box before adding the indentation.
