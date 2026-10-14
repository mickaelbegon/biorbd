# Video 17 - static-optimization (AnimStaticOptimization)

## What the video shows
Point 1: at one frame (index 50, t = 0.5 s) of a SYNTHETIC motion of `examples/arm26.bioMod` (2 dof, 6 muscles), three activation
sets that all give the same joint torque (3.17 and 1.53 N.m): two alternative sets, then the set found by biorbd. The muscle torque
(`muscles.joint_torque`) is identical, the sum of squared activations drops (1.34, 1.54, then 0.50).
Point 2: all 101 frames: activations found, target torque (inverse dynamics) and muscle torque on one axis, difference axis
(largest difference 0.0002 N.m, from the .npz).

## Verified (biorbd 1.12.3, `biorbd_anim` env)
- `biorbd.has_static_optimization` is True. Model: 2 q, 6 muscles (TRIlong, BIClong, BICshort, TRIlat, TRImed, BRA), gravity on.
- `biorbd.Biorbd.inverse_dynamics(q, qdot, qddot)` and `model.muscles.joint_torque(activations, q, qdot)` (wrapper/muscle.py:462) run.
- The muscle joint torque is exactly linear in the activations (passive part + M a): checked by rebuilding `M` with unit activations
  (rank 2, null space of dimension 4 = 6 muscles - 2 dof).
- Objective in C++ (src/InternalForces/Muscles/StaticOptimizationIpopt.cpp l.176): sum of squared activations (p-norm 2, root skipped)
  + 1000 x squared torque residual; activation bounds 1e-4..0.9999; equality constraint muscle torque + residual = target.
  So the result is the minimum-sum-of-squares solution (convex problem, unique) with a heavily penalised residual.
- Generator: `generate_static_optimization_data.py`; displayed code read from it between `# CODE-BEGIN/END` and `# CODE-BEGIN-CHECK/END-CHECK`.

## IMPORTANT finding: the Python wrapper gives wrong activations
`biorbd.StaticOptimization(model).perform_frames(...)` (binding/python3/wrapper/static_optimization.py) calls `optim.run()` and the C++
default is `run(useLinearizedState = true)`. With this default the returned activations are all about 1e-4..1e-3 (the lower bound),
and the muscle torque they produce does NOT match the target (for my synthetic motion: mean |difference| 2.4 N.m, max 6.3 N.m; in a
single-frame test where the target was made with known activations the difference was 5 N.m). With `run(False)` (exact muscle torque in the
constraint) the same problem is solved correctly: the difference is at most 2.3e-4 N.m over 101 frames, and the sum of squares is lower
than that of the activations used to build the target. The repository test `test_static_optimization` only checks target = gravity torque at
q = 0 and therefore does not detect it. For that reason the video uses the lower-level class
`from biorbd.biorbd import StaticOptimization` with `model.internal` and `run(False)` (same class the wrapper uses). I did not
investigate the root cause in the linearized Jacobian (`StaticOptimizationIpoptLinearized::prepareJacobian`) and did not modify the library.
Also, `run()` prints the whole Ipopt log (print_level 5 and a derivative check that reports 5 "errors" for the gradient, which is the
finite-difference check on the objective, harmless here); the generator output is therefore noisy.

## Hand-written parts (labelled "Hand-written code" on screen, together with "Synthetic data")
- The synthetic motion (two sinusoids, analytic qdot/qddot).
- The two alternative activation sets: random activations projected onto {M a = target - passive torque} with a pseudo-inverse
  (seed 17), accepted when inside (1e-4, 0.9) and with a larger sum of squares. Their torque is computed by biorbd (`joint_torque`)
  and equals the target to 1e-14 N.m. No hand-written optimiser is used (no SLSQP comparison of notebook 5).
- The biorbd-tutorial notebook 5 was not consulted (no network access).

## Checks done
- Difference shown comes from the .npz: largest |muscle torque - target| = 2.3e-4 N.m (activation lower bound 1e-4 makes it non-zero).
- The optimum at the demo frame uses BIClong, TRIlat, TRImed; 3 muscles stay at the lower bound over the whole motion.
- Catalog entry validated with validate_catalog.py (0 error). Renders at 480p15 EN and FR with --strict: only "no catalog.json entry".
- Content lasts about 22 s at native speed (slightly above the 20 s guideline; same order as AnimIkLeastSquares).

## Not verified / limits
- Only one demo frame is used to show redundancy; the two alternatives are not "all" solutions.
- Real (measured) torques, muscle dynamics (excitation -> activation), force-velocity effects beyond what `joint_torque` includes.
- Root cause of the linearized-mode discrepancy (see above).
