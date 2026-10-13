#include "muscle_audit_test_utils.h"
using namespace internal_forces::muscles;
template <class Hill>
void checkUnits() {
  Model model("models/muscle_audit.bioMod");
  rigidbody::GeneralizedCoordinates q(model);
  rigidbody::GeneralizedVelocity v(model);
  q[0] = 0.2;
  v[0] = 0.1;
  model.updateMuscles(q, v, true);
  Characteristics characteristics =
      model.muscle(0).characteristics().DeepCopy();
  Hill muscle("test", model.muscle(0).position(), characteristics);
  muscle.characteristics().setMaxShorteningSpeedUnits(
      OPTIMAL_LENGTHS_PER_SECOND);
  double fv = muscle_audit::value(muscle.FvCE());
  double damping = muscle_audit::value(muscle.damping());
  EXPECT_NEAR(
      muscle_audit::value(muscle.maxShorteningSpeedInLengthUnits()), 1, 1e-12);
  muscle.characteristics().setMaxShorteningSpeed(1);
  muscle.characteristics().setMaxShorteningSpeedUnits(LENGTH_UNITS_PER_SECOND);
  EXPECT_NEAR(muscle_audit::value(muscle.FvCE()), fv, 1e-12);
  EXPECT_NEAR(muscle_audit::value(muscle.damping()), damping, 1e-12);
  EXPECT_EQ(
      muscle.characteristics().DeepCopy().maxShorteningSpeedUnits(),
      LENGTH_UNITS_PER_SECOND);
}
TEST(MuscleAuditUnits, EquivalentPhysicalSpeedsAcrossFamilies) {
  checkUnits<HillType>();
  checkUnits<HillThelenType>();
  checkUnits<HillDeGrooteType>();
}
TEST(MuscleAuditUnits, FileUnitsAndInvalidUnits) {
  muscle_audit::TemporaryModel fixture(
      "muscle_audit_units",
      "maxShorteningSpeed 10",
      "maxShorteningSpeed 10\n    maxShorteningSpeedUnits "
      "optimal_lengths_per_second");
  Model model(fixture.path);
  EXPECT_EQ(
      model.muscle(0).characteristics().maxShorteningSpeedUnits(),
      OPTIMAL_LENGTHS_PER_SECOND);
  muscle_audit::TemporaryModel invalid(
      "muscle_audit_invalid_units",
      "maxShorteningSpeed 10",
      "maxShorteningSpeed 10\n    maxShorteningSpeedUnits "
      "meters_per_fortnight");
  EXPECT_THROW(Model rejected(invalid.path), std::runtime_error);
}

TEST(MuscleAuditUnits, LegacyConventionFollowsTheCurveNotCopiedTypeMetadata) {
  Model model("models/muscle_audit.bioMod");
  HillType generic(model.muscle(0));
  HillThelenType thelen(model.muscle(0));
  EXPECT_NEAR(
      muscle_audit::value(generic.maxShorteningSpeedInLengthUnits()),
      10,
      1e-12);
  EXPECT_NEAR(
      muscle_audit::value(thelen.maxShorteningSpeedInLengthUnits()), 1, 1e-12);
}
