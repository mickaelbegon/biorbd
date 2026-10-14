# Fiber and musculotendon velocity

The current geometry uses a rigid tendon and a constant pennation angle:
lf = (lMT - lSlack) / cos(alpha). Consequently, the fiber velocity is
vMT / cos(alpha), and its Jacobian is JMT / cos(alpha).

`MuscleGeometry.velocity()` now returns the derivative of `length()`.
Use `musculoTendonVelocity()` for the path velocity and
`fiberLengthJacobian()` for the fiber-length derivative.
`jacobianLength()` and `musclesLengthJacobian()` retain the path Jacobian.
Joint torque still uses tau = -JMT^T * pathForce, preserving joint/path virtual
power. This test does not assert energy conservation between isolated fibers
and the complete pennation model.

This change makes the force-velocity and damping inputs consistent with the
implemented fiber length. It changes results for pennated muscles. The
constant-angle assumption is retained; a constant-thickness, variable-angle
model requires different kinematics and remains a separate modeling extension.

Tracking issue: https://github.com/pyomeca/biorbd/issues/402
