#ifndef BIORBD_WRAPPING_NUMERICAL_KERNEL_H
#define BIORBD_WRAPPING_NUMERICAL_KERNEL_H
#include <Eigen/Core>
#include <cstddef>
#include <vector>
// Private double-only kernel: independent of biorbd backend, Scalar and RBDL.
namespace biorbd { namespace wrapping {
struct Shape {
    enum Kind { Cylinder, Ellipsoid, Torus };
    Kind kind;
    Eigen::Vector3d dimensions; ///< Cylinder (radius,length,0), ellipse axes, torus (R,r,0).
    int side;
    Shape(Kind kind,const Eigen::Vector3d& dimensions,int side=1);
};
struct Object {
    Shape shape;
    Eigen::Matrix4d pose; ///< Proper rigid local-to-global transform.
    Object(const Shape& shape,const Eigen::Matrix4d& pose);
};
struct Region {
    Eigen::Vector3d start,end; ///< Fixed anatomical anchors in global coordinates.
    std::vector<std::size_t> objects; ///< Ordered indices into the object's array.
};
struct Seed {
    std::vector<double> parameters; ///< Contact angles and log normalized arc length.
    Eigen::Vector3d start,end; ///< Previous local endpoints. State is owned by value.
    Shape::Kind kind;
    int side; ///< Zero means that this legacy seed has no descriptor stamp.
    Eigen::Vector3d dimensions; ///< Descriptor stamp; invalidates seeds on shape changes.
    Seed();
};
struct Contact {
    bool active;
    Eigen::Vector3d entry,exit;
    std::vector<Eigen::Vector3d> samples;
    double length; ///< Surface arc only. Samples/contacts use the caller's frame.
    Contact();
};
enum class RegionSolver { Analytical, NativeGeodesic, Scholz2015, NativeFallback, NativeWarm };
/// Whether this library was built with the optional Simbody Scholz2015 adapter.
bool ngvAvailable();
struct PathResult {
    double length;
    std::vector<RegionSolver> regionSolvers; ///< Accepted route for each Region.
    std::vector<Contact> contacts; ///< One result per Object, including inactive objects.
    std::vector<Eigen::Vector3d> anchorGradients; ///< start0,end0,start1,end1,...
    std::vector<Eigen::Matrix4d> poseGradients; ///< Only the upper 3x4 block is used.
};
/// Stationary connected branch checked numerically; no global minimum claim.
/// Unresolved shooting, disconnected torus contacts and cylinder rims throw.
Contact solveLocal(const Shape& shape,const Eigen::Vector3d& a,
                   const Eigen::Vector3d& b,Seed* seed=nullptr);
/// Bootstrap, coupled sweeps and global collision checks. Every Object occurs exactly
/// once in Regions. Seeds are transactional and committed only on success; absent
/// seeds give cold evaluation with state local to this call.
/// Envelope gradients require stationary frictionless contacts and rigid pose variations.
/// The rotation block specifies its contraction with dR tangent to SO(3), not
/// derivatives for arbitrary independent matrix coefficients.
PathResult solvePath(const std::vector<Object>& objects,const std::vector<Region>& regions,
                     std::vector<Seed>* seeds=nullptr);
/// Minimum normalized implicit level over a global chord (analytic root isolation).
double surfaceLevelOnSegment(const Object& object,const Eigen::Vector3d& a,
                             const Eigen::Vector3d& b);
} }
#endif
