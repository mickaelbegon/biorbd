#include <gtest/gtest.h>
#include "biorbdConfig.h"

#ifdef BIORBD_USE_CASADI_MATH
#ifdef MODULE_MUSCLES
#include <casadi/casadi.hpp>
#include <string>
#include <vector>
#include "BiorbdModel.h"
#include "Utils/Matrix.h"
#include "Utils/Vector3d.h"
#include "InternalForces/Muscles/Muscle.h"
#include "InternalForces/Muscles/MuscleGeometry.h"
#include "RigidBody/GeneralizedCoordinates.h"

using namespace BIORBD_NAMESPACE;

namespace {
casadi::Function wrappingFunction(const std::string& fixture, const std::string& name)
{
    Model model(fixture);
    const casadi::MX input = casadi::MX::sym(name + "_q", model.nbQ());
    rigidbody::GeneralizedCoordinates q(input);
    model.updateMuscles(q, true);
    const auto& geometry = model.muscle(0).position();
    const casadi::MX length = geometry.musculoTendonLength();
    const casadi::MX gradient = geometry.jacobianLength();
    std::vector<casadi::MX> points;
    for (const auto& point : geometry.pathPointsInGlobal()) points.push_back(point);
    return casadi::Function(name, {input},
        {length, gradient, casadi::MX::jacobian(length, input), casadi::MX::horzcat(points)});
    // The model and all of its geometry are destroyed on return. Functions must
    // own the callback and its immutable kinematic snapshot independently.
}

std::vector<casadi::DM> evaluate(const casadi::Function& function, const casadi::DM& q)
{
    return function(std::vector<casadi::DM>{q});
}
}

TEST(WrappingCasadi, ThreeDimensionalReferencesGradientAdAndFiniteDifference)
{
    const char* fixtures[] = {"models/wrapping_ellipsoid_3d_moving.bioMod",
        "models/wrapping_torus_3d_moving.bioMod", "models/wrapping_mixed_3d_moving.bioMod"};
    const double references[] = {0.3589189402510813, 0.18982123405216328, 0.4999204268042869};
    for (unsigned int fixture = 0; fixture < 3; ++fixture) {
        SCOPED_TRACE(fixtures[fixture]);
        const casadi::Function function = wrappingFunction(fixtures[fixture], "reference_" + std::to_string(fixture));
        const casadi::DM q = casadi::DM::zeros(9, 1);
        const auto result = evaluate(function, q);
        EXPECT_NEAR(static_cast<double>(result[0]), references[fixture], 1e-9);
        ASSERT_EQ(result[1].size1(), 1);
        ASSERT_EQ(result[1].size2(), 9);
        EXPECT_EQ(result[3].size1(), 3);
        EXPECT_EQ(result[3].size2(), fixture == 2 ? 8 : 4);
        for (unsigned int coordinate = 0; coordinate < 9; ++coordinate) {
            EXPECT_NEAR(static_cast<double>(result[1](0, coordinate)),
                static_cast<double>(result[2](0, coordinate)), 1e-10);
            casadi::DM plus(q), minus(q);
            plus(coordinate) += 1e-6;
            minus(coordinate) -= 1e-6;
            const double difference = (static_cast<double>(evaluate(function, plus)[0]) -
                static_cast<double>(evaluate(function, minus)[0])) / 2e-6;
            EXPECT_NEAR(static_cast<double>(result[1](0, coordinate)), difference, 2e-6)
                << "coordinate " << coordinate;
        }
    }
}

TEST(WrappingCasadi, ExportedFunctionSurvivesModelAndInterleavedEvaluations)
{
    casadi::Function function;
    {
        function = wrappingFunction("models/wrapping_mixed_3d_moving.bioMod", "lifetime_mixed");
    }
    casadi::DM qA = casadi::DM::zeros(9, 1), qB(qA);
    for (unsigned int coordinate = 0; coordinate < 9; ++coordinate)
        qB(coordinate) = 1e-4 * (-0.3 + 0.7 * coordinate / 8);
    const auto first = evaluate(function, qA);
    evaluate(function, qB);
    const auto last = evaluate(function, qA);
    EXPECT_NEAR(static_cast<double>(last[0]), 0.4999204268042869, 1e-9);
    for (std::size_t output = 0; output < first.size(); ++output) {
        ASSERT_EQ(first[output].numel(), last[output].numel());
        for (casadi_int i = 0; i < first[output].numel(); ++i)
            EXPECT_NEAR(static_cast<double>(first[output](i)), static_cast<double>(last[output](i)), 1e-10);
    }
}

TEST(WrappingCasadi, InactiveHoleKeepsFixedContactSlots)
{
    const casadi::Function function = wrappingFunction("models/wrapping_torus_hole.bioMod", "inactive_hole");
    const auto result = evaluate(function, casadi::DM::zeros(0, 1));
    EXPECT_NEAR(static_cast<double>(result[0]), 0.2, 1e-10);
    ASSERT_EQ(result[3].size1(), 3);
    ASSERT_EQ(result[3].size2(), 4);
    unsigned int duplicatedSegments = 0;
    for (unsigned int point = 1; point < 4; ++point) {
        double squaredDistance = 0;
        for (unsigned int coordinate = 0; coordinate < 3; ++coordinate) {
            const double delta = static_cast<double>(result[3](coordinate, point)) -
                static_cast<double>(result[3](coordinate, point - 1));
            squaredDistance += delta * delta;
        }
        if (squaredDistance < 1e-20) ++duplicatedSegments;
    }
    EXPECT_GE(duplicatedSegments, 2u);
}
#endif // MODULE_MUSCLES
#endif // BIORBD_USE_CASADI_MATH
