#ifndef BIORBD_INTERNAL_WRAPPING_GEODESIC_H
#define BIORBD_INTERNAL_WRAPPING_GEODESIC_H
#include "biorbdConfig.h"
#ifndef BIORBD_USE_CASADI_MATH
#include <Eigen/Core>
#include <vector>
namespace BIORBD_NAMESPACE { namespace internal_forces { namespace wrapping_detail {
/// Internal normalized solver. It checks a stationary connected contact branch
/// to numerical tolerances, not the globally shortest path or a homotopy class.
/// Surface defects and normal reactions are evaluated on integration samples.
struct Surface {
    enum Kind { Ellipsoid, Torus };
    Kind kind;
    Eigen::Vector3d dimensions;
    double scale;
    Surface(Kind kind, const Eigen::Vector3d& dimensions);
    Eigen::Vector3d point(const Eigen::Vector2d& angles) const;
    Eigen::Vector2d angles(const Eigen::Vector3d& point) const;
    void geometry(const Eigen::Vector3d& point, double& level,
        Eigen::Vector3d& gradient, Eigen::Matrix3d& hessian) const;
    double chordLevel(const Eigen::Vector3d& a,const Eigen::Vector3d& b) const;
};
struct Result {
    Eigen::Vector3d parameters; ///< Two contact angles and log normalized arc length.
    std::vector<Eigen::Vector3d> points; ///< Normalized local surface samples.
    double length; ///< Normalized arc length only.
};
/// Each continuation call must use exterior endpoints. Coarse fitting is bounded;
/// finalSolve additionally refines integration and validates physical feasibility.
Result shoot(const Surface& surface, const Eigen::Vector3d& a,
    const Eigen::Vector3d& b, const Eigen::Vector3d& seed, bool finalSolve);
} } }
#endif
#endif
