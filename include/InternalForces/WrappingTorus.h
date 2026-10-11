#ifndef BIORBD_WRAPPING_TORUS_H
#define BIORBD_WRAPPING_TORUS_H
#include "InternalForces/WrappingSurface.h"
namespace BIORBD_NAMESPACE { namespace internal_forces {
/// Ring torus about local z, majorRadius > minorRadius > 0.
/// Meridian and exterior equatorial paths are analytical; other paths use
/// verified geodesic shooting with continuation. One connected contact arc
/// is supported. Disconnected or unresolved contacts throw explicitly.
/// A stationary branch is not guaranteed to be the global 3D minimum.
class BIORBD_API WrappingTorus : public WrappingSurface {
public:
    WrappingTorus(const utils::RotoTrans& rt, const utils::Scalar& majorRadius,
        const utils::Scalar& minorRadius, const utils::String& name = "",
        const utils::String& parentName = "");
    WrappingTorus DeepCopy() const;
    void DeepCopy(const WrappingTorus& other);
    utils::Scalar majorRadius() const;
    utils::Scalar minorRadius() const;
protected:
    void solveLocal(const utils::Vector3d& a, const utils::Vector3d& b,
        std::vector<utils::Vector3d>& points, utils::Scalar& arcLength) const override;
    utils::Scalar m_majorRadius, m_minorRadius;
};
} }
#endif
