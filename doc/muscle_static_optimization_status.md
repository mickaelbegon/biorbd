# Static optimization result validation

`run()` accepts IPOPT success or acceptable-level termination, then reconstructs
the physical joint torque including residual torque. Each coordinate must have
finite error below 1e-6 * (1 + abs(target)). The tolerance uses the coordinate's
torque/force unit. Only a verified frame can warm start the next frame.

Failures raise an exception. `solverStatuses()` remains accessible for
diagnostics, while `finalSolution()`, `finalResidual()` and `finalTorqueError()`
require a completely successful run. Re-running clears previously cached frames.
Residual vectors use nbGeneralizedTorque(), including quaternion models.

Integrate the separate constraint correction before using the affine path:
this check intentionally rejects physically incorrect historical solutions.
The status-only regression suite uses `run(false)` to test the full force law.

Tracking issue: https://github.com/pyomeca/biorbd/issues/402
