#include "InternalForces/Muscles/StateDynamicsDeGroote.h"
#include "muscle_audit_test_utils.h"
using namespace internal_forces::muscles;
TEST(MuscleAuditActivation, RawAndNormalizedExcitationsAreEquivalent) {
  Characteristics characteristics;
  characteristics.setStateMax(State(2, 1));
  for (double rawExcitation : {0.8, 1.6}) {
    StateDynamicsDeGroote raw(rawExcitation, 0.5);
    StateDynamicsDeGroote normalized(rawExcitation / 2, 0.5);
    double actual = muscle_audit::value(
        raw.timeDerivativeActivation(characteristics, false));
    double reference = muscle_audit::value(
        normalized.timeDerivativeActivation(characteristics, true));
    EXPECT_NEAR(actual, reference, 1e-12);
    EXPECT_EQ(actual > 0, rawExcitation / 2 > 0.5);
    EXPECT_NEAR(muscle_audit::value(raw.excitation()), rawExcitation, 1e-12);
  }
}
