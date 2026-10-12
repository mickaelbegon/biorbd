#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "../src/InternalForces/WrappingNumericalKernel.h"

namespace wrapping = ::biorbd::wrapping;
namespace {
using wrapping::Object;
using wrapping::Region;
using wrapping::Shape;
using wrapping::PathResult;

Eigen::Matrix4d pose(const Eigen::Matrix3d& rotation = Eigen::Matrix3d::Identity(),
                     const Eigen::Vector3d& translation = Eigen::Vector3d::Zero())
{
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.block<3, 3>(0, 0) = rotation;
    result.block<3, 1>(0, 3) = translation;
    return result;
}

Object ellipsoid(const Eigen::Matrix4d& transform = Eigen::Matrix4d::Identity())
{
    return Object(Shape(Shape::Ellipsoid, Eigen::Vector3d(0.07, 0.045, 0.055), +1), transform);
}

Object torus(const Eigen::Matrix4d& transform = Eigen::Matrix4d::Identity())
{
    return Object(Shape(Shape::Torus, Eigen::Vector3d(0.065, 0.021, 0.0), +1), transform);
}

PathResult solveOne(const Object& object, const Eigen::Vector3d& start,
                    const Eigen::Vector3d& end)
{
    const Region region{start, end, {0}};
    return wrapping::solvePath({object}, {region});
}

void expectSameContact(const wrapping::Contact& a, const wrapping::Contact& b, double tol)
{
    EXPECT_EQ(a.active, b.active);
    EXPECT_NEAR((a.entry - b.entry).norm(), 0.0, tol);
    EXPECT_NEAR((a.exit - b.exit).norm(), 0.0, tol);
    EXPECT_NEAR(a.length, b.length, tol);
}

wrapping::RegionSolver expectedSpatialSolver()
{
    return wrapping::ngvAvailable() ? wrapping::RegionSolver::Scholz2015
                                    : wrapping::RegionSolver::NativeGeodesic;
}
}

TEST(WrappingHybrid, AnalyticalCasesDispatchWithoutExternalGeodesic)
{
    const Eigen::Vector3d cylinderA(-0.15, -0.01, -0.065);
    const Eigen::Vector3d cylinderB(0.15, -0.01, 0.065);
    const Object cylinder(Shape(Shape::Cylinder, Eigen::Vector3d(0.05, 0.18, 0.0), +1),
                          Eigen::Matrix4d::Identity());
    const PathResult helix = solveOne(cylinder, cylinderA, cylinderB);
    ASSERT_EQ(helix.regionSolvers.size(), 1u);
    EXPECT_EQ(helix.regionSolvers[0], wrapping::RegionSolver::Analytical);
    ASSERT_EQ(helix.contacts.size(), 1u);
    EXPECT_TRUE(helix.contacts[0].active);
    EXPECT_NEAR(helix.length, 0.3492105912719195, 1e-9);

    // An exact free chord through the torus hole must keep the direct-path rule;
    // a geodesic library is not asked to construct a contact where none exists.
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(0.37, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
    const Eigen::Vector3d translation(0.3, -0.2, 0.4);
    const Eigen::Matrix4d torusPose = pose(rotation, translation);
    const Eigen::Vector3d a = rotation * Eigen::Vector3d(0, 0, -0.1) + translation;
    const Eigen::Vector3d b = rotation * Eigen::Vector3d(0, 0, 0.1) + translation;
    const PathResult hole = solveOne(torus(torusPose), a, b);
    ASSERT_EQ(hole.regionSolvers.size(), 1u);
    EXPECT_EQ(hole.regionSolvers[0], wrapping::RegionSolver::Analytical);
    ASSERT_EQ(hole.contacts.size(), 1u);
    EXPECT_FALSE(hole.contacts[0].active);
    EXPECT_NEAR(hole.contacts[0].length, 0.0, 1e-14);
    EXPECT_NEAR(hole.length, 0.2, 1e-12);

    // The torus meridian plane has an analytic tube-circle solution, while the
    // oblique 3-D torus above is covered by the separate geodesic test.
    const PathResult meridian = solveOne(torus(), Eigen::Vector3d(0, 0, -0.01),
                                         Eigen::Vector3d(0.13, 0, -0.01));
    ASSERT_EQ(meridian.regionSolvers.size(), 1u);
    EXPECT_EQ(meridian.regionSolvers[0], wrapping::RegionSolver::Analytical);
    EXPECT_NEAR(meridian.length, 0.14470526426579974, 1e-9);

    // Frame 119 of the 201-frame cycle is a clear direct-path case. Prime the warm seed
    // with the active identity-pose baseline before moving to this frame; stale
    // contact state must not prevent the analytical inactive chord.
    const Eigen::Vector3d baselineA(0.0, -0.038, -0.03);
    const Eigen::Vector3d baselineB(0.13, 0.07, -0.01);
    std::vector<wrapping::Seed> torusSeeds(1);
    const Region activeBaseline{baselineA, baselineB, {0}};
    const PathResult baseline = wrapping::solvePath({torus()}, {activeBaseline}, &torusSeeds);
    ASSERT_EQ(baseline.contacts.size(), 1u);
    EXPECT_TRUE(baseline.contacts[0].active);
    EXPECT_NEAR(baseline.length, 0.18982123405216328, 1e-9);

    Eigen::Matrix4d torus119Pose;
    torus119Pose << 0.9997539720346027, 0.0032093335266765713, -0.021947564314739758, -0.00044966670228170424,
                    -0.003371683320646893, 0.9999671998589528, -0.007364167151193894, -0.0018270805742745622,
                     0.021923210363000212, 0.007436355596661672, 0.9997320008196295, 0.0004648882429441256,
                     0.0, 0.0, 0.0, 1.0;
    const Object turnedTorus = torus(torus119Pose);
    const Eigen::Vector3d frame119A(0.0, -0.038, -0.03);
    const Eigen::Vector3d frame119B(0.12943791662214787, 0.06955768718687928, -0.010780733409550998);
    const Region frame119Region{frame119A, frame119B, {0}};
    const PathResult frame119 = wrapping::solvePath({turnedTorus}, {frame119Region}, &torusSeeds);
    ASSERT_EQ(frame119.regionSolvers.size(), 1u);
    EXPECT_EQ(frame119.regionSolvers[0], wrapping::RegionSolver::Analytical);
    ASSERT_EQ(frame119.contacts.size(), 1u);
    EXPECT_FALSE(frame119.contacts[0].active);
    EXPECT_NEAR(frame119.contacts[0].length, 0.0, 1e-14);
    EXPECT_NEAR(frame119.length, 0.16938775203876882, 1e-12);

    const Eigen::Matrix3d frameRotation = torus119Pose.block<3, 3>(0, 0);
    const Eigen::Vector3d frameTranslation = torus119Pose.block<3, 1>(0, 3);
    const Eigen::Vector3d holeA = frameRotation * Eigen::Vector3d(0, 0, -0.1) + frameTranslation;
    const Eigen::Vector3d holeB = frameRotation * Eigen::Vector3d(0, 0, 0.1) + frameTranslation;
    const Region holeRegion{holeA, holeB, {0}};
    const PathResult holeAfter119 = wrapping::solvePath({turnedTorus}, {holeRegion}, &torusSeeds);
    ASSERT_EQ(holeAfter119.regionSolvers.size(), 1u);
    EXPECT_EQ(holeAfter119.regionSolvers[0], wrapping::RegionSolver::Analytical);
    ASSERT_EQ(holeAfter119.contacts.size(), 1u);
    EXPECT_FALSE(holeAfter119.contacts[0].active);
    EXPECT_NEAR(holeAfter119.length, 0.2, 1e-12);
}

TEST(WrappingHybrid, ThreeDimensionalReferencesRetainComplexSolverAndColdDeterminism)
{
    const Eigen::Vector3d ellipsoidA(-0.17, -0.01, -0.025);
    const Eigen::Vector3d ellipsoidB(0.17, 0.005, 0.035);
    const Object e = ellipsoid();
    const PathResult ellipsoidResult = solveOne(e, ellipsoidA, ellipsoidB);
    ASSERT_EQ(ellipsoidResult.regionSolvers.size(), 1u);
    EXPECT_EQ(ellipsoidResult.regionSolvers[0], expectedSpatialSolver());
    ASSERT_EQ(ellipsoidResult.contacts.size(), 1u);
    EXPECT_TRUE(ellipsoidResult.contacts[0].active);
    EXPECT_NEAR(ellipsoidResult.length, 0.3589189402510813, 1e-9);
    const PathResult repeated = solveOne(e, ellipsoidA, ellipsoidB);
    EXPECT_NEAR(repeated.length, ellipsoidResult.length, 1e-12);
    expectSameContact(repeated.contacts[0], ellipsoidResult.contacts[0], 1e-10);

    const Eigen::Vector3d torusA(0.0, -0.038, -0.03);
    const Eigen::Vector3d torusB(0.13, 0.07, -0.01);
    const PathResult torusResult = solveOne(torus(), torusA, torusB);
    ASSERT_EQ(torusResult.regionSolvers.size(), 1u);
    EXPECT_EQ(torusResult.regionSolvers[0], expectedSpatialSolver());
    ASSERT_EQ(torusResult.contacts.size(), 1u);
    EXPECT_TRUE(torusResult.contacts[0].active);
    EXPECT_NEAR(torusResult.length, 0.18982123405216328, 1e-9);

    // Cylinder-only chains keep native helices, including non-parallel axes.
    const std::array<Eigen::Vector3d, 3> centers = {
        Eigen::Vector3d(-0.1, 0.005, -0.008), Eigen::Vector3d(0.0, 0.017, 0.01),
        Eigen::Vector3d(0.1, -0.002, -0.005)};
    const std::array<Eigen::Vector3d, 3> angles = {
        Eigen::Vector3d(20, -8, 0), Eigen::Vector3d(-18, 12, 0), Eigen::Vector3d(10, 17, 0)};
    const std::array<double, 3> radii = {0.032, 0.03, 0.027};
    const std::array<double, 3> heights = {0.17, 0.18, 0.17};
    constexpr double pi = 3.14159265358979323846;
    std::vector<Object> cylinders;
    for (std::size_t i = 0; i < centers.size(); ++i) {
        const Eigen::Matrix3d R =
            (Eigen::AngleAxisd(angles[i][2] * pi / 180, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(angles[i][1] * pi / 180, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(angles[i][0] * pi / 180, Eigen::Vector3d::UnitX())).toRotationMatrix();
        cylinders.emplace_back(Shape(Shape::Cylinder, Eigen::Vector3d(radii[i], heights[i], 0), +1),
                               pose(R, centers[i]));
    }
    const Region chain{Eigen::Vector3d(-0.22, -0.026, -0.055),
                       Eigen::Vector3d(0.22, -0.018, 0.06), {0, 1, 2}};
    const PathResult chainResult = wrapping::solvePath(cylinders, {chain});
    ASSERT_EQ(chainResult.regionSolvers.size(), 1u);
    EXPECT_EQ(chainResult.regionSolvers[0], wrapping::RegionSolver::Analytical);
    EXPECT_NEAR(chainResult.length, 0.48235866649711817, 1e-9);
    ASSERT_EQ(chainResult.contacts.size(), 3u);
    for (const auto& contact : chainResult.contacts) EXPECT_TRUE(contact.active);
    for (std::size_t i = 0; i < cylinders.size(); ++i) {
        const auto& item = chainResult.contacts[i];
        const Eigen::Matrix3d R = cylinders[i].pose.block<3, 3>(0, 0);
        const Eigen::Vector3d t = cylinders[i].pose.block<3, 1>(0, 3);
        for (const Eigen::Vector3d& p : {item.entry, item.exit}) {
            const double localZ = (R.transpose() * (p - t))[2];
            EXPECT_LE(std::abs(localZ), 0.5 * heights[i] + 1e-12);
        }
    }

    const auto rot = [](double xDeg, double yDeg) {
        constexpr double pi = 3.14159265358979323846;
        return (Eigen::AngleAxisd(yDeg * pi / 180, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(xDeg * pi / 180, Eigen::Vector3d::UnitX())).toRotationMatrix();
    };
    std::vector<Object> mixed;
    mixed.emplace_back(Shape(Shape::Ellipsoid, Eigen::Vector3d(0.048, 0.032, 0.044), +1),
                       pose(rot(3, -5), Eigen::Vector3d(-0.12, 0, -0.006)));
    mixed.emplace_back(Shape(Shape::Cylinder, Eigen::Vector3d(0.034, 0.1, 0), +1),
                       pose(rot(-5, 3), Eigen::Vector3d(0, 0.017, 0.004)));
    mixed.emplace_back(Shape(Shape::Torus, Eigen::Vector3d(0.03, 0.015, 0), +1),
                       pose(rot(5, -3), Eigen::Vector3d(0.12, 0, -0.002)));
    const Region mixedRegion{Eigen::Vector3d(-0.23, -0.025, -0.015),
                             Eigen::Vector3d(0.23, -0.025, 0.015), {0, 1, 2}};
    const PathResult mixedResult = wrapping::solvePath(mixed, {mixedRegion});
    ASSERT_EQ(mixedResult.regionSolvers.size(), 1u);
    EXPECT_EQ(mixedResult.regionSolvers[0], expectedSpatialSolver());
    EXPECT_NEAR(mixedResult.length, 0.4999204268042869, 1e-9);
    ASSERT_EQ(mixedResult.contacts.size(), 3u);
    for (const auto& contact : mixedResult.contacts) EXPECT_TRUE(contact.active);
}

TEST(WrappingHybrid, StationaryEnvelopeMatchesAnchorAndRigidMaterialFiniteDifferences)
{
    const Eigen::Vector3d start(-0.17, -0.01, -0.025);
    const Eigen::Vector3d end(0.17, 0.005, 0.035);
    const Eigen::Matrix3d R =
        (Eigen::AngleAxisd(0.21, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(-0.17, Eigen::Vector3d::UnitY())).toRotationMatrix();
    const Eigen::Vector3d t(0.01, -0.008, 0.003);
    const Object base = ellipsoid(pose(R, t));
    const PathResult reference = solveOne(base, start, end);
    ASSERT_EQ(reference.anchorGradients.size(), 2u);
    ASSERT_EQ(reference.poseGradients.size(), 1u);
    const double h = 1e-6;

    for (int coordinate = 0; coordinate < 3; ++coordinate) {
        Eigen::Vector3d plusStart = start, minusStart = start;
        plusStart[coordinate] += h;
        minusStart[coordinate] -= h;
        const double fdStart = (solveOne(base, plusStart, end).length -
                                solveOne(base, minusStart, end).length) / (2 * h);
        EXPECT_NEAR(reference.anchorGradients[0][coordinate], fdStart, 2e-6);

        Eigen::Vector3d plusEnd = end, minusEnd = end;
        plusEnd[coordinate] += h;
        minusEnd[coordinate] -= h;
        const double fdEnd = (solveOne(base, start, plusEnd).length -
                              solveOne(base, start, minusEnd).length) / (2 * h);
        EXPECT_NEAR(reference.anchorGradients[1][coordinate], fdEnd, 2e-6);

        Eigen::Matrix4d plusPose = base.pose, minusPose = base.pose;
        plusPose(coordinate, 3) += h;
        minusPose(coordinate, 3) -= h;
        const double fdTranslation =
            (solveOne(Object(base.shape, plusPose), start, end).length -
             solveOne(Object(base.shape, minusPose), start, end).length) / (2 * h);
        EXPECT_NEAR(reference.poseGradients[0](coordinate, 3), fdTranslation, 2e-6);
    }

    // The pose gradient is contracted only with tangent rotations dR = [w]x R.
    const Eigen::Vector3d axis(1, -2, 3);
    const Eigen::Vector3d unitAxis = axis.normalized();
    const Eigen::Matrix3d skew =
        (Eigen::Matrix3d() << 0, -unitAxis[2], unitAxis[1],
                              unitAxis[2], 0, -unitAxis[0],
                              -unitAxis[1], unitAxis[0], 0).finished();
    const Eigen::Matrix3d dR = skew * R;
    const double predicted =
        (reference.poseGradients[0].block<3, 3>(0, 0).array() * dR.array()).sum();
    const Eigen::Matrix3d plusR = Eigen::AngleAxisd(h, unitAxis).toRotationMatrix() * R;
    const Eigen::Matrix3d minusR = Eigen::AngleAxisd(-h, unitAxis).toRotationMatrix() * R;
    const double fdRotation =
        (solveOne(ellipsoid(pose(plusR, t)), start, end).length -
         solveOne(ellipsoid(pose(minusR, t)), start, end).length) / (2 * h);
    EXPECT_NEAR(predicted, fdRotation, 2e-6);
}

TEST(WrappingHybrid, FiniteCylinderRimAndViaCrossingNeverReturnInfeasiblePath)
{
    const Object finiteCylinder(
        Shape(Shape::Cylinder, Eigen::Vector3d(0.05, 0.18, 0.0), +1),
        Eigen::Matrix4d::Identity());
    const Region rimRegion{Eigen::Vector3d(-0.15, -0.01, -0.9),
                           Eigen::Vector3d(0.15, -0.01, 0.9), {0}};
    EXPECT_THROW(wrapping::solvePath({finiteCylinder}, {rimRegion}), std::runtime_error);

    const Object left(Shape(Shape::Cylinder, Eigen::Vector3d(0.04, 0.12, 0.0), +1),
                      pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-0.065, -0.005, 0.0)));
    const Object right(Shape(Shape::Cylinder, Eigen::Vector3d(0.038, 0.12, 0.0), +1),
                       pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.065, 0.015, 0.0)));
    const Region first{Eigen::Vector3d(-0.18, -0.025, 0.0),
                       Eigen::Vector3d(0.18, -0.015, 0.0), {0}};
    const Region second{Eigen::Vector3d(0.18, -0.015, 0.0),
                        Eigen::Vector3d(0.3, -0.015, 0.0), {1}};
    std::vector<wrapping::Seed> seeds(2);
    seeds[0].parameters = {0.123};
    seeds[1].parameters = {-0.456};
    const auto before = seeds;
    EXPECT_THROW(wrapping::solvePath({left, right}, {first, second}, &seeds), std::runtime_error);
    // Failure must not commit partially updated warm starts.
    EXPECT_EQ(seeds[0].parameters, before[0].parameters);
    EXPECT_EQ(seeds[1].parameters, before[1].parameters);
}


TEST(WrappingHybrid, NativeFallbackHandlesFiniteCylinderInitialization)
{
    // The origin is outside the second finite cylinder, but inside its infinite
    // radial extension. NGV's circular initializer is inadmissible there. The
    // first ellipsoid lifts the accepted path clear of the finite second solid.
    const Object first(Shape(Shape::Ellipsoid, Eigen::Vector3d(0.1, 0.1, 0.1)),
        pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-0.07, 0, 0)));
    const Object second(Shape(Shape::Cylinder, Eigen::Vector3d(0.02, 0.02, 0)),
        pose(Eigen::AngleAxisd(std::acos(-1.) / 2, Eigen::Vector3d::UnitY()).toRotationMatrix(),
             Eigen::Vector3d(0.04, 0, 0)));
    const Region coupled{Eigen::Vector3d(-0.2, 0, 0), Eigen::Vector3d(0.4, 0, 0), {0, 1}};
    const PathResult actual = wrapping::solvePath({first, second}, {coupled});
    const PathResult single = wrapping::solvePath({first},
        {Region{coupled.start, coupled.end, {0}}});
    ASSERT_EQ(actual.regionSolvers.size(), 1u);
    EXPECT_EQ(actual.regionSolvers[0], wrapping::ngvAvailable() ?
        wrapping::RegionSolver::NativeFallback : wrapping::RegionSolver::NativeGeodesic);
    ASSERT_EQ(actual.contacts.size(), 2u);
    EXPECT_TRUE(actual.contacts[0].active);
    EXPECT_FALSE(actual.contacts[1].active);
    EXPECT_NEAR(actual.length, single.length, 1e-12);
    EXPECT_NEAR((actual.anchorGradients[0] - single.anchorGradients[0]).norm(), 0, 1e-12);
    EXPECT_NEAR((actual.anchorGradients[1] - single.anchorGradients[1]).norm(), 0, 1e-12);
    EXPECT_NEAR(wrapping::surfaceLevelOnSegment(second, actual.contacts[0].exit, coupled.end),
                wrapping::surfaceLevelOnSegment(second, single.contacts[0].exit, coupled.end), 1e-12);
    EXPECT_GE(wrapping::surfaceLevelOnSegment(second, actual.contacts[0].exit, coupled.end), -1e-8);
}


TEST(WrappingHybrid, SingleTorusUsesValidatedNativeWarmSeedAfterColdSolve)
{
    const Object object = torus();
    const Region base{Eigen::Vector3d(0, -0.038, -0.03),
                      Eigen::Vector3d(0.13, 0.07, -0.01), {0}};
    std::vector<wrapping::Seed> seeds;
    const PathResult cold = wrapping::solvePath({object}, {base}, &seeds);
    ASSERT_EQ(seeds.size(), 1u);
    ASSERT_EQ(seeds[0].parameters.size(), 3u);
    EXPECT_EQ(cold.regionSolvers[0], expectedSpatialSolver());
    Region moved = base;
    moved.end += Eigen::Vector3d(1e-4, -2e-4, 1e-4);
    const PathResult warm = wrapping::solvePath({object}, {moved}, &seeds);
    ASSERT_EQ(warm.regionSolvers.size(), 1u);
    EXPECT_EQ(warm.regionSolvers[0], wrapping::RegionSolver::NativeWarm);
    const PathResult reference = wrapping::solvePath({object}, {moved});
    EXPECT_NEAR(warm.length, reference.length, 1e-9);
    expectSameContact(warm.contacts[0], reference.contacts[0], 1e-9);
    EXPECT_NEAR((warm.anchorGradients[0] - reference.anchorGradients[0]).norm(), 0, 2e-8);
    EXPECT_NEAR((warm.anchorGradients[1] - reference.anchorGradients[1]).norm(), 0, 2e-8);
    const PathResult returned = wrapping::solvePath({object}, {base}, &seeds);
    EXPECT_NEAR(returned.length, cold.length, 1e-9);
    expectSameContact(returned.contacts[0], cold.contacts[0], 1e-9);

    // A descriptor change invalidates the old seed before hybrid dispatch.
    seeds[0].dimensions[0] *= 1.001;
    const PathResult invalidated = wrapping::solvePath({object}, {base}, &seeds);
    EXPECT_EQ(invalidated.regionSolvers[0], expectedSpatialSolver());
    EXPECT_NEAR(invalidated.length, cold.length, 1e-9);
    expectSameContact(invalidated.contacts[0], cold.contacts[0], 1e-9);
}
