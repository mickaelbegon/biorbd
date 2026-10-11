#ifndef BIORBD_WRAPPING_ELLIPSOID_H
#define BIORBD_WRAPPING_ELLIPSOID_H
#include "InternalForces/WrappingSurface.h"
namespace BIORBD_NAMESPACE { namespace internal_forces {
/// Ellipsoid with local principal semiaxes. Symmetry-plane paths are analytical;
/// other paths use verified unit-speed geodesic shooting with continuation.
/// A stationary branch is not guaranteed to be the global 3D minimum.
/// Unresolved or degenerate contact configurations throw explicitly.
class BIORBD_API WrappingEllipsoid : public WrappingSurface {
public:
    WrappingEllipsoid(const utils::RotoTrans& rt, const utils::Vector3d& radii,
        const utils::String& name = "", const utils::String& parentName = "");
    WrappingEllipsoid DeepCopy() const;
    void DeepCopy(const WrappingEllipsoid& other);
    const utils::Vector3d& radii() const;
protected:
    void solveLocal(const utils::Vector3d& a, const utils::Vector3d& b,
        std::vector<utils::Vector3d>& points, utils::Scalar& arcLength) const override;
    utils::Vector3d m_radii;
};
} }
#endif
