#include "muscle_audit_test_utils.h"
TEST(MuscleAuditReader, LegacyVelocityAndTypoDiagnostics) {
  for (const auto& property : {"maxVelocity", "maxShorteningSpeed"}) {
    muscle_audit::TemporaryModel fixture(
        "muscle_audit_reader",
        "maxShorteningSpeed 10",
        std::string(property) + " 5");
    Model model(fixture.path);
    EXPECT_NEAR(
        muscle_audit::value(
            model.muscle(0).characteristics().maxShorteningSpeed()),
        5,
        1e-12);
  }
  muscle_audit::TemporaryModel typo(
      "muscle_audit_typo", "maxShorteningSpeed 10", "maxShortningSpeed 5");
  EXPECT_THROW(Model model(typo.path), std::runtime_error);
}
