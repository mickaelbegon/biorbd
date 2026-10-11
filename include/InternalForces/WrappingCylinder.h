#ifndef BIORBD_WRAPPING_CYLINDER_H
#define BIORBD_WRAPPING_CYLINDER_H
#include "InternalForces/WrappingSurface.h"
namespace BIORBD_NAMESPACE { namespace internal_forces {
/// Finite lateral cylinder: local z axis, radius r, z in [-length/2,length/2].
/// Uses the exact developed-surface helix. Rim/cap contact is unsupported.
class BIORBD_API WrappingCylinder : public WrappingSurface {
public:
    WrappingCylinder(const utils::RotoTrans& rt, const utils::Scalar& radius,
        const utils::Scalar& length, const utils::String& name = "",
        const utils::String& parentName = "");
    WrappingCylinder DeepCopy() const;
    void DeepCopy(const WrappingCylinder& other);
    utils::Scalar radius() const;
    const utils::Scalar& length() const;
protected:
    void solveLocal(const utils::Vector3d& a, const utils::Vector3d& b,
        std::vector<utils::Vector3d>& points, utils::Scalar& arcLength) const override;
    utils::Scalar m_radius, m_length;
};
} }
#endif
