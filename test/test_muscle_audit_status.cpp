#include "muscle_audit_test_utils.h"
#ifdef MODULE_STATIC_OPTIM
using namespace internal_forces::muscles;
TEST(MuscleAuditStatus, FailedFrameCannotReturnOrWarmStartInvalidSolution) {
  Model model("models/muscle_audit.bioMod");
  rigidbody::GeneralizedCoordinates q(model);
  q[0] = 0.2;
  rigidbody::GeneralizedVelocity v(model);
  v[0] = 0;
  rigidbody::GeneralizedTorque feasible(model), impossible(model);
  feasible[0] = -500;
  impossible[0] = -2000;
  std::vector<rigidbody::GeneralizedCoordinates> qs{q, q};
  std::vector<rigidbody::GeneralizedVelocity> vs{v, v};
  std::vector<rigidbody::GeneralizedTorque> targets{feasible, impossible};
  StaticOptimization optimization(model, qs, vs, targets, 0.5, 2, false, 0);
  EXPECT_THROW(optimization.run(false), std::runtime_error);
  auto statuses = optimization.solverStatuses();
  ASSERT_EQ(statuses.size(), 2);
  EXPECT_EQ(statuses[0], 0);
  EXPECT_NE(statuses[1], 0);
  EXPECT_THROW(optimization.finalSolution(), std::runtime_error);
  EXPECT_THROW(optimization.finalResidual(0), std::runtime_error);
}
TEST(MuscleAuditStatus, PhysicalResidualAndRepeatedRun) {
  Model model("models/muscle_audit.bioMod");
  rigidbody::GeneralizedCoordinates q(model);
  q[0] = 0.2;
  rigidbody::GeneralizedVelocity v(model);
  v[0] = 0;
  rigidbody::GeneralizedTorque target(model);
  target[0] = -500;
  StaticOptimization optimization(model, q, v, target, 0.5, 2, true, 0);
  for (int repeat = 0; repeat < 2; ++repeat) {
    optimization.run(false);
    ASSERT_EQ(optimization.finalSolution().size(), 1);
    ASSERT_EQ(optimization.solverStatuses().size(), 1);
    EXPECT_NEAR(optimization.finalTorqueError(0)[0], 0, 1e-5);
    EXPECT_EQ(
        optimization.finalResidual(0).size(), model.nbGeneralizedTorque());
  }
}

TEST(MuscleAuditStatus, QuaternionResidualUsesTorqueDimension) {
  Model model("models/simple_quat.bioMod");
  ASSERT_NE(model.nbQ(), model.nbGeneralizedTorque());
  rigidbody::GeneralizedCoordinates q(model);
  q.setZero();
  q[3] = 1;
  rigidbody::GeneralizedVelocity v(model);
  v.setZero();
  rigidbody::GeneralizedTorque target(model);
  target.setZero();
  utils::Vector initial(0);
  StaticOptimizationIpopt problem(model, q, v, target, initial, true, 2, 0);
  Ipopt::Index variables, constraints, nnzJacobian, nnzHessian;
  Ipopt::TNLP::IndexStyleEnum style;
  ASSERT_TRUE(problem.get_nlp_info(
      variables, constraints, nnzJacobian, nnzHessian, style));
  EXPECT_EQ(variables, model.nbGeneralizedTorque());
  EXPECT_EQ(constraints, model.nbGeneralizedTorque());
  EXPECT_EQ(nnzJacobian, model.nbGeneralizedTorque());
}

#endif
