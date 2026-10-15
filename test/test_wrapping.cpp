#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "biorbdConfig.h"

#ifdef BIORBD_USE_EIGEN3_MATH
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include "BiorbdModel.h"
#include "InternalForces/Muscles/Muscle.h"
#include "InternalForces/Muscles/MuscleGeometry.h"
#include "InternalForces/WrappingCylinder.h"
#include "InternalForces/WrappingEllipsoid.h"
#include "InternalForces/WrappingTorus.h"
#include "Utils/RotoTrans.h"
#include "Utils/Matrix.h"
#include "Utils/Vector3d.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"

using namespace BIORBD_NAMESPACE;

namespace {
constexpr double kLengthTolerance = 1e-5;

double totalLength(const utils::Vector3d& a,
                   const utils::Vector3d& b,
                   const utils::Vector3d& contactA,
                   const utils::Vector3d& contactB,
                   double arcLength)
{
    return (contactA - a).norm() + arcLength + (b - contactB).norm();
}
}

TEST(Wrapping, CylinderReferenceAndContactGeometry)
{
    internal_forces::WrappingCylinder cylinder(utils::RotoTrans(), 0.05, 0.18);
    const utils::Vector3d a(-0.15, -0.01, -0.065);
    const utils::Vector3d b(0.15, -0.01, 0.065);
    utils::Vector3d contactA, contactB;
    double arcLength = 0;

    cylinder.setWrappingSide(+1);
    cylinder.wrapPoints(utils::RotoTrans(), a, b, contactA, contactB, &arcLength);

    EXPECT_TRUE(cylinder.isActive());
    EXPECT_NEAR(totalLength(a, b, contactA, contactB, arcLength),
                0.3492105912719195, 1e-6);
    EXPECT_NEAR(std::hypot(contactA[0], contactA[1]), 0.05, 1e-8);
    EXPECT_NEAR(std::hypot(contactB[0], contactB[1]), 0.05, 1e-8);
    EXPECT_LE(std::abs(contactA[2]), 0.09);
    EXPECT_LE(std::abs(contactB[2]), 0.09);

    const utils::Vector3d normalA(contactA[0], contactA[1], 0.0);
    const utils::Vector3d normalB(contactB[0], contactB[1], 0.0);
    EXPECT_LT(std::abs((contactA - a).normalized().dot(normalA.normalized())), 1e-6);
    EXPECT_LT(std::abs((b - contactB).normalized().dot(normalB.normalized())), 1e-6);
}

TEST(Wrapping, NoCollisionLeavesTheStraightPathInactive)
{
    internal_forces::WrappingCylinder cylinder(utils::RotoTrans(), 0.05, 0.18);
    const utils::Vector3d a(-0.15, 0.09, 0.0);
    const utils::Vector3d b(0.15, 0.09, 0.0);
    utils::Vector3d contactA, contactB;
    double arcLength = 1.0;

    cylinder.wrapPoints(utils::RotoTrans(), a, b, contactA, contactB, &arcLength);

    EXPECT_FALSE(cylinder.isActive());
    EXPECT_NEAR(arcLength, 0.0, 1e-12);
    EXPECT_NEAR((b - a).norm(), 0.3, 1e-12);
}

TEST(Wrapping, EllipsoidReferenceBranch)
{
    internal_forces::WrappingEllipsoid ellipsoid(
        utils::RotoTrans(), utils::Vector3d(0.07, 0.045, 0.055));
    const utils::Vector3d a(-0.17, -0.01, 0.0);
    const utils::Vector3d b(0.17, 0.005, 0.0);
    utils::Vector3d contactA, contactB;
    double arcLength = 0;

    ellipsoid.setWrappingSide(+1);
    ellipsoid.wrapPoints(utils::RotoTrans(), a, b, contactA, contactB, &arcLength);

    EXPECT_TRUE(ellipsoid.isActive());
    EXPECT_NEAR(totalLength(a, b, contactA, contactB, arcLength),
                0.35419781480619866, kLengthTolerance);
}

TEST(Wrapping, TorusTubeAndHoleReferences)
{
    internal_forces::WrappingTorus torus(utils::RotoTrans(), 0.065, 0.021);
    utils::Vector3d contactA, contactB;
    double arcLength = 0;
    const utils::Vector3d a(0.0, 0.0, -0.01);
    const utils::Vector3d b(0.13, 0.0, -0.01);

    torus.setWrappingSide(+1);
    torus.wrapPoints(utils::RotoTrans(), a, b, contactA, contactB, &arcLength);
    EXPECT_TRUE(torus.isActive());
    EXPECT_NEAR(totalLength(a, b, contactA, contactB, arcLength),
                0.14470526426579974, kLengthTolerance);

    const utils::Vector3d holeA(0.0, 0.0, -0.1);
    const utils::Vector3d holeB(0.0, 0.0, 0.1);
    torus.wrapPoints(utils::RotoTrans(), holeA, holeB, contactA, contactB, &arcLength);
    EXPECT_FALSE(torus.isActive());
    EXPECT_NEAR(arcLength, 0.0, 1e-12);
    EXPECT_NEAR((holeB - holeA).norm(), 0.2, 1e-12);
}

TEST(Wrapping, RigidTransformPreservesCylinderPathLength)
{
    const utils::RotoTrans identity;
    utils::RotoTrans moved = utils::RotoTrans::Identity();
    moved.block<3, 3>(0, 0) = Eigen::AngleAxisd(0.71, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
    moved.block<3, 1>(0, 3) = Eigen::Vector3d(0.4, -0.2, 0.3);
    const utils::Vector3d a(-0.15, -0.01, -0.065);
    const utils::Vector3d b(0.15, -0.01, 0.065);
    const utils::Vector3d movedA(moved.block<3, 3>(0, 0) * Eigen::Vector3d(a) + moved.block<3, 1>(0, 3));
    const utils::Vector3d movedB(moved.block<3, 3>(0, 0) * Eigen::Vector3d(b) + moved.block<3, 1>(0, 3));
    internal_forces::WrappingCylinder first(identity, 0.05, 0.18);
    internal_forces::WrappingCylinder second(moved, 0.05, 0.18);
    utils::Vector3d firstA, firstB, secondA, secondB;
    double firstArc = 0, secondArc = 0;

    first.setWrappingSide(+1);
    second.setWrappingSide(+1);
    first.wrapPoints(identity, a, b, firstA, firstB, &firstArc);
    second.wrapPoints(moved, movedA, movedB, secondA, secondB, &secondArc);

    EXPECT_NEAR(totalLength(a, b, firstA, firstB, firstArc),
                totalLength(movedA, movedB, secondA, secondB, secondArc), 1e-10);
}

TEST(Wrapping, CopiesOwnTheirComputedSurfacePoints)
{
    internal_forces::WrappingCylinder original(utils::RotoTrans(), 0.05, 0.18);
    const utils::Vector3d a(-0.15, -0.01, -0.065);
    const utils::Vector3d b(0.15, -0.01, 0.065);
    utils::Vector3d contactA, contactB;
    double arcLength = 0;
    original.setWrappingSide(+1);
    original.wrapPoints(utils::RotoTrans(), a, b, contactA, contactB, &arcLength);
    const auto originalSurface = original.surfacePoints();
    auto copy = original;

    copy.wrapPoints(utils::RotoTrans(), utils::Vector3d(-0.15, 0.09, 0.0),
                    utils::Vector3d(0.15, 0.09, 0.0), contactA, contactB, &arcLength);

    EXPECT_TRUE(original.isActive());
    EXPECT_FALSE(copy.isActive());
    ASSERT_EQ(original.surfacePoints().size(), originalSurface.size());
    for (std::size_t i = 0; i < originalSurface.size(); ++i) {
        EXPECT_LT((original.surfacePoints()[i] - originalSurface[i]).norm(), 1e-12);
    }
}

TEST(Wrapping, RejectsInvalidDimensions)
{
    EXPECT_THROW(internal_forces::WrappingCylinder(utils::RotoTrans(), 0.0, 0.18), std::invalid_argument);
    EXPECT_THROW(internal_forces::WrappingEllipsoid(
                     utils::RotoTrans(), utils::Vector3d(0.07, 0.0, 0.055)), std::invalid_argument);
    EXPECT_THROW(internal_forces::WrappingTorus(utils::RotoTrans(), 0.02, 0.021), std::invalid_argument);
}

namespace {
void checkThreeDimensionalContact(internal_forces::WrappingSurface& surface,
                                  const utils::Vector3d& a, const utils::Vector3d& b,
                                  double reference, bool ellipsoid)
{
    utils::Vector3d entry, exit;
    double arcLength = 0;
    surface.wrapPoints(utils::RotoTrans(), a, b, entry, exit, &arcLength);
    ASSERT_TRUE(surface.isActive());
    EXPECT_NEAR(totalLength(a, b, entry, exit, arcLength), reference, 1e-9);
    const auto& curve = surface.surfacePoints();
    ASSERT_GT(curve.size(), 4u);
    Eigen::MatrixXd centered(curve.size(), 3);
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& point : curve) mean += Eigen::Vector3d(point);
    mean /= curve.size();
    double polyline = (entry - a).norm() + (b - exit).norm();
    for (std::size_t i = 0; i < curve.size(); ++i) {
        const Eigen::Vector3d point(curve[i]);
        centered.row(i) = (point - mean).transpose();
        if (ellipsoid) {
            const Eigen::Vector3d radii(0.07, 0.045, 0.055);
            EXPECT_NEAR((point.array() / radii.array()).matrix().squaredNorm(), 1.0, 1e-7);
        } else {
            EXPECT_NEAR(std::pow(std::hypot(point[0], point[1]) - 0.065, 2) +
                        point[2] * point[2], 0.021 * 0.021, 1e-10);
        }
        if (i) polyline += (curve[i] - curve[i - 1]).norm();
    }
    EXPECT_NEAR(polyline, reference, 1e-7);
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
    const Eigen::VectorXd deviations = centered * svd.matrixV().col(2);
    EXPECT_GT(deviations.cwiseAbs().maxCoeff(), 1e-6);
    for (unsigned int i = 0; i < 2; ++i) {
        const Eigen::Vector3d point(i == 0 ? entry : exit);
        Eigen::Vector3d normal;
        if (ellipsoid) {
            normal = point.array() / Eigen::Vector3d(0.07, 0.045, 0.055).array().square();
        } else {
            const double rho = std::hypot(point[0], point[1]);
            normal = Eigen::Vector3d((rho - 0.065) * point[0] / rho,
                                    (rho - 0.065) * point[1] / rho, point[2]);
        }
        const Eigen::Vector3d direction(i == 0 ? utils::Vector3d(entry - a) : utils::Vector3d(b - exit));
        EXPECT_NEAR(direction.normalized().dot(normal.normalized()), 0.0, 2e-7);
    }
}
}

TEST(Wrapping, EllipsoidGeneralThreeDimensionalReference)
{
    internal_forces::WrappingEllipsoid surface(utils::RotoTrans(), utils::Vector3d(0.07, 0.045, 0.055));
    checkThreeDimensionalContact(surface, utils::Vector3d(-0.17, -0.01, -0.025),
        utils::Vector3d(0.17, 0.005, 0.035), 0.3589189402510813, true);
}

TEST(Wrapping, TorusGeneralThreeDimensionalReference)
{
    internal_forces::WrappingTorus surface(utils::RotoTrans(), 0.065, 0.021);
    checkThreeDimensionalContact(surface, utils::Vector3d(0, -0.038, -0.03),
        utils::Vector3d(0.13, 0.07, -0.01), 0.18982123405216328, false);
}

TEST(Wrapping, GeneralThreeDimensionalEllipsoidSideCopyAndRigidFrame)
{
    const utils::Vector3d a(-0.17, -0.01, -0.025), b(0.17, 0.005, 0.035);
    internal_forces::WrappingEllipsoid original(utils::RotoTrans(), utils::Vector3d(0.07, 0.045, 0.055));
    utils::Vector3d entry, exit;
    double arc = 0;
    original.wrapPoints(utils::RotoTrans(), a, b, entry, exit, &arc);
    const auto originalCurve = original.surfacePoints();
    const double originalLength = totalLength(a, b, entry, exit, arc);
    auto copied = original.DeepCopy();
    copied.setWrappingSide(-1);
    copied.wrapPoints(utils::RotoTrans(), a, b, entry, exit, &arc);
    EXPECT_LT(entry[1] + exit[1], 0.0);
    EXPECT_GT(originalCurve.front()[1] + originalCurve.back()[1], 0.0);
    ASSERT_EQ(original.surfacePoints().size(), originalCurve.size());
    for (std::size_t i = 0; i < originalCurve.size(); ++i)
        EXPECT_LT((original.surfacePoints()[i] - originalCurve[i]).norm(), 1e-12);
    utils::RotoTrans moved = utils::RotoTrans::Identity();
    moved.block<3, 3>(0, 0) = Eigen::AngleAxisd(0.71, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
    moved.block<3, 1>(0, 3) = Eigen::Vector3d(0.4, -0.2, 0.3);
    const utils::Vector3d movedA(moved.block<3, 3>(0, 0) * Eigen::Vector3d(a) + moved.block<3, 1>(0, 3));
    const utils::Vector3d movedB(moved.block<3, 3>(0, 0) * Eigen::Vector3d(b) + moved.block<3, 1>(0, 3));
    internal_forces::WrappingEllipsoid tilted(moved, utils::Vector3d(0.07, 0.045, 0.055));
    tilted.wrapPoints(moved, movedA, movedB, entry, exit, &arc);
    EXPECT_NEAR(totalLength(movedA, movedB, entry, exit, arc), originalLength, 1e-9);
    EXPECT_LT((entry - utils::Vector3d(moved.block<3, 3>(0, 0) * Eigen::Vector3d(originalCurve.front()) +
        moved.block<3, 1>(0, 3))).norm(), 1e-8);
    // Invalid interior endpoints remain unsupported by the general solver.
    EXPECT_THROW(original.wrapPoints(utils::RotoTrans(), utils::Vector3d(0, 0, 0), b,
        entry, exit, &arc), std::invalid_argument);
}

#ifdef MODULE_MUSCLES
TEST(Wrapping, GeneralThreeDimensionalNineDofJacobiansSpeedAndWarmReturn)
{
    const char* fixtures[] = {"models/wrapping_ellipsoid_3d_moving.bioMod", "models/wrapping_torus_3d_moving.bioMod", "models/wrapping_mixed_3d_moving.bioMod"};
    const double references[] = {0.3589189402510813, 0.18982123405216328, 0.4999204268042869};
    for (unsigned int fixture = 0; fixture < 3; ++fixture) {
        SCOPED_TRACE(fixtures[fixture]);
        Model model(fixtures[fixture]);
        ASSERT_EQ(model.nbQ(), 9u);
        rigidbody::GeneralizedCoordinates q(model);
        rigidbody::GeneralizedVelocity qdot(model);
        q.setZero();
        for (unsigned int i = 0; i < 9; ++i) qdot[i] = -0.3 + 0.7 * i / 8;
        model.updateMuscles(q, qdot, true);
        const auto& geometry = model.muscle(0).position();
        EXPECT_NEAR(geometry.musculoTendonLength(), references[fixture], 1e-9);
        const auto originalNodes = geometry.pointsInGlobal();
        const utils::Matrix jacobian(geometry.jacobianLength());
        const double speed = geometry.velocity();
        double projectedSpeed = 0;
        for (unsigned int i = 0; i < 9; ++i) projectedSpeed += jacobian(0, i) * qdot[i];
        EXPECT_NEAR(speed, projectedSpeed, 1e-12);
        const double h = 1e-6;
        for (unsigned int coordinate = 0; coordinate < 9; ++coordinate) {
            rigidbody::GeneralizedCoordinates plus(q), minus(q);
            plus[coordinate] += h;
            minus[coordinate] -= h;
            model.updateMuscles(plus, true);
            const double lengthPlus = geometry.musculoTendonLength();
            model.updateMuscles(minus, true);
            const double lengthMinus = geometry.musculoTendonLength();
            EXPECT_NEAR(jacobian(0, coordinate), (lengthPlus - lengthMinus) / (2 * h), 2e-6)
                << "coordinate " << coordinate;
        }
        rigidbody::GeneralizedCoordinates plus(q), minus(q), moved(q);
        for (unsigned int i = 0; i < 9; ++i) {
            plus[i] += h * qdot[i];
            minus[i] -= h * qdot[i];
            moved[i] += 1e-4 * qdot[i];
        }
        model.updateMuscles(plus, true);
        const double lengthPlus = geometry.musculoTendonLength();
        model.updateMuscles(minus, true);
        const double lengthMinus = geometry.musculoTendonLength();
        EXPECT_NEAR(speed, (lengthPlus - lengthMinus) / (2 * h), 2e-6);
        model.updateMuscles(moved, true);
        model.updateMuscles(q, true);
        EXPECT_NEAR(geometry.musculoTendonLength(), references[fixture], 1e-9);
        ASSERT_EQ(geometry.pointsInGlobal().size(), originalNodes.size());
        for (std::size_t i = 0; i < originalNodes.size(); ++i)
            EXPECT_LT((geometry.pointsInGlobal()[i] - originalNodes[i]).norm(), 1e-8);
    }
}
#endif

#ifdef MODULE_MUSCLES
TEST(Wrapping, MixedThreeDimensionalReferenceAndDensePathLength)
{
    Model model("models/wrapping_mixed_3d.bioMod");
    rigidbody::GeneralizedCoordinates q(model);
    q.setZero();
    model.updateMuscles(q, true);
    const auto& geometry = model.muscle(0).position();
    EXPECT_NEAR(geometry.musculoTendonLength(), 0.4999204268042869, 1e-9);
    ASSERT_EQ(geometry.pointsInGlobal().size(), 8u);
    const auto& path = geometry.pathPointsInGlobal();
    ASSERT_GT(path.size(), 8u);
    double polylineLength = 0;
    for (std::size_t i = 1; i < path.size(); ++i)
        polylineLength += (path[i] - path[i - 1]).norm();
    EXPECT_NEAR(polylineLength, geometry.musculoTendonLength(), 1e-7);
}

TEST(Wrapping, TwoCylinderModelReferenceAndDensePathLength)
{
    Model model("models/wrapping_two_cylinders.bioMod");
    rigidbody::GeneralizedCoordinates q(model);
    model.updateMuscles(q, true);
    const auto& geometry = model.muscle(0).position();

    EXPECT_NEAR(geometry.musculoTendonLength(), 0.39645033645390365, 1e-9);
    const auto& path = geometry.pathPointsInGlobal();
    ASSERT_GT(path.size(), 2u);
    double polylineLength = 0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        polylineLength += (path[i] - path[i - 1]).norm();
    }
    EXPECT_NEAR(polylineLength, geometry.musculoTendonLength(), 1e-7);
}

TEST(Wrapping, ThreeIndependentlyTiltedCylindersFormNonplanarPath)
{
    Model model("models/wrapping_three_cylinders_3d.bioMod");
    rigidbody::GeneralizedCoordinates q(model);
    model.updateMuscles(q, true);
    const auto& geometry = model.muscle(0).position();

    EXPECT_NEAR(geometry.musculoTendonLength(), 0.48235866649711817, 1e-9);
    const auto& nodes = geometry.pointsInGlobal();
    ASSERT_EQ(nodes.size(), 8u);
    const auto& path = geometry.pathPointsInGlobal();
    double polylineLength = 0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        polylineLength += (path[i] - path[i - 1]).norm();
    }
    // The sampled display polyline approximates the continuous contact arcs.
    EXPECT_NEAR(polylineLength, geometry.musculoTendonLength(), 1e-7);

    double largestContactTripleProduct = 0;
    for (std::size_t i = 1; i <= 6; ++i) {
        const Eigen::Vector3d a(nodes[i]);
        for (std::size_t j = i + 1; j <= 6; ++j) {
            const Eigen::Vector3d b(nodes[j]);
            for (std::size_t k = j + 1; k <= 6; ++k) {
                const Eigen::Vector3d c(nodes[k]);
                for (std::size_t l = k + 1; l <= 6; ++l) {
                    const Eigen::Vector3d d(nodes[l]);
                    largestContactTripleProduct = std::max(largestContactTripleProduct,
                        std::abs((b - a).dot((c - a).cross(d - a))));
                }
            }
        }
    }
    EXPECT_GT(largestContactTripleProduct, 1e-9);
}

TEST(Wrapping, ViaPointAppearsInOrderedGlobalPath)
{
    Model model("models/wrapping_via.bioMod");
    rigidbody::GeneralizedCoordinates q(model);
    model.updateMuscles(q, true);
    const auto& geometry = model.muscle(0).position();
    const auto& nodes = geometry.pointsInGlobal();
    const auto& path = geometry.pathPointsInGlobal();

    ASSERT_GE(nodes.size(), 3u);
    EXPECT_GT(path.size(), nodes.size());
    EXPECT_LT((path.front() - nodes.front()).norm(), 1e-12);
    EXPECT_LT((path.back() - nodes.back()).norm(), 1e-12);
    const utils::Vector3d expectedVia(0, -0.02, 0);
    EXPECT_TRUE(std::any_of(path.begin(), path.end(), [&](const utils::Vector3d& point) {
        return (point - expectedVia).norm() < 1e-12;
    }));
    double polylineLength = 0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        polylineLength += (path[i] - path[i - 1]).norm();
    }
    EXPECT_NEAR(polylineLength, geometry.musculoTendonLength(), 1e-7);
}

TEST(Wrapping, MovingInsertionLengthJacobianMatchesFiniteDifference)
{
    Model model("models/wrapping_moving_insertion.bioMod");
    rigidbody::GeneralizedCoordinates q(model);
    model.updateMuscles(q, true);
    const auto& initialGeometry = model.muscle(0).position();
    const utils::Matrix jacobianLength(initialGeometry.jacobianLength());
    const double step = 1e-6;

    for (unsigned int coordinate = 0; coordinate < 2; ++coordinate) {
        rigidbody::GeneralizedCoordinates plus(q), minus(q);
        plus[coordinate] += step;
        minus[coordinate] -= step;
        model.updateMuscles(plus, true);
        const double lengthPlus = model.muscle(0).position().musculoTendonLength();
        model.updateMuscles(minus, true);
        const double lengthMinus = model.muscle(0).position().musculoTendonLength();
        const double finiteDifference = (lengthPlus - lengthMinus) / (2 * step);
        EXPECT_NEAR(jacobianLength(0, coordinate), finiteDifference, 2e-7);
    }
}

TEST(Wrapping, MovingObjectsViaAndSpeedMatchLengthJacobian)
{
    Model model("models/wrapping_moving_path.bioMod");
    rigidbody::GeneralizedCoordinates q(model);
    rigidbody::GeneralizedVelocity qdot(model);
    q.setZero();
    qdot.setZero();
    for (unsigned int i = 0; i < model.nbQ(); ++i) {
        qdot[i] = -0.3 + 0.7 * i / std::max(1u, model.nbQ() - 1);
    }
    model.updateMuscles(q, qdot, true);
    const auto& geometry = model.muscle(0).position();
    const utils::Matrix jacobianLength(geometry.jacobianLength());
    double expectedVelocity = 0;
    for (unsigned int i = 0; i < model.nbQ(); ++i) {
        expectedVelocity += jacobianLength(0, i) * qdot[i];
    }
    const double velocity = geometry.velocity();
    EXPECT_NEAR(velocity, expectedVelocity, 1e-12);

    const double velocityStep = 1e-6;
    rigidbody::GeneralizedCoordinates alongPlus(q), alongMinus(q);
    for (unsigned int i = 0; i < model.nbQ(); ++i) {
        alongPlus[i] += velocityStep * qdot[i];
        alongMinus[i] -= velocityStep * qdot[i];
    }
    model.updateMuscles(alongPlus, true);
    const double lengthAlongPlus = model.muscle(0).position().musculoTendonLength();
    model.updateMuscles(alongMinus, true);
    const double lengthAlongMinus = model.muscle(0).position().musculoTendonLength();
    EXPECT_NEAR(velocity,
                (lengthAlongPlus - lengthAlongMinus) / (2 * velocityStep), 2e-7);

    const double step = 1e-6;
    for (unsigned int coordinate = 0; coordinate < model.nbQ(); ++coordinate) {
        rigidbody::GeneralizedCoordinates plus(q), minus(q);
        plus[coordinate] += step;
        minus[coordinate] -= step;
        model.updateMuscles(plus, true);
        const double lengthPlus = model.muscle(0).position().musculoTendonLength();
        model.updateMuscles(minus, true);
        const double lengthMinus = model.muscle(0).position().musculoTendonLength();
        const double finiteDifference = (lengthPlus - lengthMinus) / (2 * step);
        EXPECT_NEAR(jacobianLength(0, coordinate), finiteDifference, 2e-7)
            << "generalized coordinate " << coordinate;
    }
}

TEST(Wrapping, CrossingAnotherSurfaceAcrossViaFailsAndInvalidatesGeometry)
{
    Model model("models/wrapping_cross_via.bioMod");
    rigidbody::GeneralizedCoordinates q(model);
    const auto& geometry = model.muscle(0).position();

    try {
        model.updateMuscles(q, true);
        FAIL() << "Expected the path to collide with the later wrapping surface";
    } catch (const std::runtime_error& error) {
        EXPECT_NE(std::string(error.what()).find("outside its assigned via-point region"),
                  std::string::npos);
    }
    EXPECT_THROW(geometry.length(), std::runtime_error);
    EXPECT_THROW(geometry.musculoTendonLength(), std::runtime_error);
    EXPECT_THROW(geometry.pointsInGlobal(), std::runtime_error);
    EXPECT_THROW(geometry.pathPointsInGlobal(), std::runtime_error);
}

TEST(Wrapping, FailedMovingViaUpdateInvalidatesPreviouslyComputedGeometry)
{
    Model model("models/wrapping_moving_path.bioMod");
    rigidbody::GeneralizedCoordinates q(model);
    q.setZero();
    model.updateMuscles(q, true);
    const auto& geometry = model.muscle(0).position();
    EXPECT_GT(geometry.musculoTendonLength(), 0.0);

    // The final two coordinates translate the via carrier onto the left cylinder axis.
    rigidbody::GeneralizedCoordinates invalid(q);
    invalid[6] = -0.065;
    invalid[7] = -0.005;
    EXPECT_THROW(model.updateMuscles(invalid, true), std::runtime_error);
    EXPECT_THROW(geometry.length(), std::runtime_error);
    EXPECT_THROW(geometry.musculoTendonLength(), std::runtime_error);
    EXPECT_THROW(geometry.pointsInGlobal(), std::runtime_error);
    EXPECT_THROW(geometry.pathPointsInGlobal(), std::runtime_error);
}
#endif  // MODULE_MUSCLES

#else  // BIORBD_USE_EIGEN3_MATH
#include "InternalForces/WrappingCylinder.h"
#include "Utils/RotoTrans.h"
#include "Utils/Vector3d.h"

using namespace BIORBD_NAMESPACE;

TEST(Wrapping, CasadiReportsUnsupportedNumericalSolver)
{
    internal_forces::WrappingCylinder cylinder(utils::RotoTrans(), 0.05, 0.18);
    utils::Vector3d contactA, contactB;
    utils::Scalar arcLength = 0;
    EXPECT_THROW(cylinder.wrapPoints(utils::RotoTrans(),
                     utils::Vector3d(-0.15, -0.01, -0.065),
                     utils::Vector3d(0.15, -0.01, 0.065),
                     contactA, contactB, &arcLength), std::runtime_error);
}

#endif  // BIORBD_USE_EIGEN3_MATH
