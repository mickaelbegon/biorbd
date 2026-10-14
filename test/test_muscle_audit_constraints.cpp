#include "muscle_audit_test_utils.h"
#ifdef MODULE_STATIC_OPTIM
using namespace internal_forces::muscles;
TEST(MuscleAuditConstraints, NonzeroTargetAndPassiveOffset) {
  Model model("models/muscle_audit.bioMod");
  rigidbody::GeneralizedCoordinates q(model);
  rigidbody::GeneralizedVelocity v(model);
  v[0] = 0;
  for (double length : {0.2, 0.22}) {
    q[0] = length;
    std::vector<std::shared_ptr<State>> state{std::make_shared<State>(0, 0.37)};
    rigidbody::GeneralizedTorque target(model.muscularJointTorque(state, q, v));
    StaticOptimization optimization(model, q, v, target, 0.5, 2, false, 0);
    optimization.run(true);
    auto activation = optimization.finalSolution()[0];
    EXPECT_NEAR(activation[0], 0.37, 1e-7);
    state[0]->setActivation(activation[0]);
    EXPECT_NEAR(model.muscularJointTorque(state, q, v)[0], target[0], 1e-5);
  }
}
TEST(MuscleAuditConstraints, ActivationDependentHillUsesFullLaw) {
  muscle_audit::TemporaryModel fixture(
      "muscle_audit_hill", "Type thelen", "Type hill");
  Model model(fixture.path);
  rigidbody::GeneralizedCoordinates q(model);
  rigidbody::GeneralizedVelocity v(model);
  q[0] = 0.22;
  v[0] = 0;
  std::vector<std::shared_ptr<State>> state{std::make_shared<State>(0, 0.37)};
  rigidbody::GeneralizedTorque target(model.muscularJointTorque(state, q, v));
  StaticOptimization optimization(model, q, v, target, 0.5, 2, false, 0);
  optimization.run(true);
  state[0]->setActivation(optimization.finalSolution()[0][0]);
  EXPECT_NEAR(model.muscularJointTorque(state, q, v)[0], target[0], 1e-5);
}

TEST(MuscleAuditConstraints, NonlinearJacobianMatchesAnalyticHillDerivative) {
  muscle_audit::TemporaryModel fixture(
      "muscle_audit_jacobian", "Type thelen", "Type hill");
  Model model(fixture.path);
  rigidbody::GeneralizedCoordinates q(model);
  q[0] = 0.22;
  rigidbody::GeneralizedVelocity v(model);
  v[0] = 0;
  rigidbody::GeneralizedTorque target(model);
  target[0] = 0;
  utils::Vector initial(1);
  initial[0] = 0.5;
  StaticOptimizationIpopt problem(model, q, v, target, initial, false, 2, 0);
  for (double activation : {0.01, 0.5, 0.99}) {
    double jacobian;
    ASSERT_TRUE(problem.eval_jac_g(
        1, &activation, true, 1, 1, nullptr, nullptr, &jacobian));
    const double denominator = 1 + 0.15 * (1 - activation);
    const double normalizedLength = 1.2;
    const double distance = normalizedLength / denominator - 1;
    const double fl = std::exp(-distance * distance / 0.45);
    const double derivative =
        -1000 * fl *
        (1 - activation * 2 / 0.45 * distance * normalizedLength * 0.15 /
                 (denominator * denominator));
    EXPECT_NEAR(jacobian, derivative, 0.002);
  }
}

#endif
