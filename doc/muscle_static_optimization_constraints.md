# Static optimization torque constraints

The affine constraint includes the zero-activation torque and the requested
target: g = tau_zero + A * activation + residual - target. Passive force and
damping therefore contribute to the torque balance.

Generic Hill active force-length curves depend on activation. `run(true)` uses
the nonlinear force law when these muscles are present. Constructing a
linearized problem directly with these muscles raises an error.
The nonlinear Jacobian uses a finite-difference step of 1e-6; its regression
compares with the analytic derivative at low, medium and high activation.
The reported optimum can change substantially because the historical affine
constraint omitted the requested torque. Compare the reconstructed joint torque.

Tracking issue: https://github.com/pyomeca/biorbd/issues/402
