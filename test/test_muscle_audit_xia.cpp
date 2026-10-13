#include "muscle_audit_test_utils.h"
using namespace internal_forces::muscles;
template <class Regular, class Fatigable>
void checkRecruitment() {
  Model model("models/muscle_audit.bioMod");
  rigidbody::GeneralizedCoordinates q(model);
  rigidbody::GeneralizedVelocity v(model);
  q[0] = 0.2;
  v[0] = 0;
  model.updateMuscles(q, v, true);
  Characteristics characteristics =
      model.muscle(0).characteristics().DeepCopy();
  FatigueParameters rates = characteristics.fatigueParameters().DeepCopy();
  rates.setFatigueRate(0);
  characteristics.setFatigueParameters(rates);
  Regular regular("reference", model.muscle(0).position(), characteristics);
  Fatigable xia(
      "xia", model.muscle(0).position(), characteristics, DYNAMIC_XIA);
  Fatigable simple(
      "capacity",
      model.muscle(0).position(),
      characteristics,
      SIMPLE_STATE_FATIGUE);
  for (double active : {0.0, 0.2, 0.5, 1.0}) {
    FatigueDynamicStateXia compartments(active, 0, 1 - active);
    xia.fatigueState() = compartments;
    StateDynamics command(0, active);
    double expected = muscle_audit::value(regular.force(command));
    EXPECT_NEAR(muscle_audit::value(xia.force(command)), expected, 1e-9);
    State otherCommand(0, 1 - active);
    EXPECT_NEAR(muscle_audit::value(xia.force(otherCommand)), expected, 1e-9);
#ifndef BIORBD_USE_CASADI_MATH
    xia.computeTimeDerivativeState(command);
    auto& dynamic = dynamic_cast<FatigueDynamicStateXia&>(xia.fatigueState());
    EXPECT_NEAR(dynamic.activeFibersDot(), 0, 1e-12);
    EXPECT_NEAR(dynamic.fatiguedFibersDot(), 0, 1e-12);
    EXPECT_NEAR(dynamic.restingFibersDot(), 0, 1e-12);
#endif
  }
  FatigueState capacity(0.5, 0, 0.5);
  simple.fatigueState() = capacity;
  State halfCommand(0, 0.5);
  EXPECT_NEAR(
      muscle_audit::value(simple.force(halfCommand)),
      0.5 * muscle_audit::value(regular.force(halfCommand)),
      1e-9);
}
TEST(MuscleAuditXia, ThelenNoFatigueLimitAndSimpleCapacity) {
  checkRecruitment<HillThelenType, HillThelenTypeFatigable>();
}
TEST(MuscleAuditXia, DeGrooteNoFatigueLimitAndSimpleCapacity) {
  checkRecruitment<HillDeGrooteType, HillDeGrooteTypeFatigable>();
}
