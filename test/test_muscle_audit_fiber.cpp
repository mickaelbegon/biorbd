#include "muscle_audit_test_utils.h"
TEST(MuscleAuditFiber, FiberDerivativeAndVirtualPower) {
  Model model("models/muscle_audit.bioMod");
  rigidbody::GeneralizedCoordinates q(model);
  rigidbody::GeneralizedVelocity v(model);
  q[0] = 0.2;
  v[0] = 0.1;
  for (double angle : {0.0, 0.3, 1.0471975511965976}) {
    auto& muscle = model.muscle(0);
    muscle.characteristics().setPennationAngle(angle);
    model.updateMuscles(q, v, true);
    double fiberVelocity = muscle_audit::value(muscle.position().velocity());
    EXPECT_NEAR(fiberVelocity, 0.1 / std::cos(angle), 1e-12);
    EXPECT_NEAR(
        muscle_audit::value(muscle.position().musculoTendonVelocity()),
        0.1,
        1e-12);
    EXPECT_NEAR(
        muscle_audit::value(muscle.position().fiberLengthJacobian()(0, 0)),
        1 / std::cos(angle),
        1e-12);
    rigidbody::GeneralizedCoordinates plus(q), minus(q);
    plus[0] += 1e-7;
    minus[0] -= 1e-7;
    model.updateMuscles(plus, v, true);
    double lengthPlus = muscle_audit::value(muscle.position().length());
    model.updateMuscles(minus, v, true);
    double lengthMinus = muscle_audit::value(muscle.position().length());
    EXPECT_NEAR(fiberVelocity, (lengthPlus - lengthMinus) / 2e-7 * 0.1, 1e-9);
    utils::Vector force(1);
    force[0] = 100;
    EXPECT_NEAR(
        muscle_audit::value(model.muscularJointTorque(force)[0]) * 0.1 +
            100 * 0.1,
        0,
        1e-12);
    auto copy = muscle.position().DeepCopy();
    EXPECT_NEAR(muscle_audit::value(copy.velocity()), fiberVelocity, 1e-12);
  }
}
