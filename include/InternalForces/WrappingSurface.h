#ifndef BIORBD_WRAPPING_SURFACE_H
#define BIORBD_WRAPPING_SURFACE_H

#include "InternalForces/WrappingObject.h"
#include "Utils/RotoTrans.h"
#include "Utils/String.h"
#include <vector>

namespace BIORBD_NAMESPACE { namespace internal_forces {
/// Numerical wrapping surface. Result caches are independent on ordinary copies.
/// Dimensions and paths use the same length unit as the model (normally metres).
class BIORBD_API WrappingSurface : public WrappingObject
{
public:
    WrappingSurface(const utils::RotoTrans& rt,
                    const utils::String& name = "", const utils::String& parentName = "");
    WrappingSurface(const WrappingSurface& other);
    WrappingSurface& operator=(const WrappingSurface& other);
    virtual ~WrappingSurface();
    using WrappingObject::RT;
    /// +1 selects the branch with the greater second local planar coordinate;
    /// -1 selects the opposite branch. XY uses y; XZ and YZ use z.
    /// Cylinder uses y; a torus meridian uses z. In 3D the selected planar
    /// branch is continued to the endpoints. The default is +1.
    void setWrappingSide(int side);
    int wrappingSide() const;
    bool isActive() const;
    /// Global contact curve samples, including both contacts; empty without contact.
    const std::vector<utils::Vector3d>& surfacePoints() const;
    void wrapPoints(const utils::RotoTrans& rt, const utils::Vector3d& a,
                    const utils::Vector3d& b, utils::Vector3d& p1,
                    utils::Vector3d& p2, utils::Scalar* length = nullptr) override;
    void wrapPoints(rigidbody::Joints& model, const rigidbody::GeneralizedCoordinates& Q,
                    const utils::Vector3d& a, const utils::Vector3d& b,
                    utils::Vector3d& p1, utils::Vector3d& p2,
                    utils::Scalar* length = nullptr) override;
    /// Returns cached contacts and the surface arc length only (not free segments).
    void wrapPoints(utils::Vector3d& p1, utils::Vector3d& p2,
                    utils::Scalar* length = nullptr) override;
    const utils::RotoTrans& RT(rigidbody::Joints& model,
                    const rigidbody::GeneralizedCoordinates& Q, bool updateKin = true) override;

protected:
    void DeepCopy(const WrappingSurface& other);
    virtual void solveLocal(const utils::Vector3d& a, const utils::Vector3d& b,
                    std::vector<utils::Vector3d>& points, utils::Scalar& arcLength) const = 0;
#ifndef BIORBD_USE_CASADI_MATH
    struct PlanarArc {
        double entry, sweep, length;
        std::vector<utils::Vector3d> points;
    };
    /// Exact tangent angles and numerically integrated ellipse arc in XY.
    PlanarArc ellipseArc(double ax, double ay, double bx, double by,
                         double rx, double ry, double cx = 0, double cy = 0) const;
    static bool intersectsEllipse(double ax, double ay, double bx, double by,
                         double rx, double ry, double cx = 0, double cy = 0);
    static void validateEndpoints(const utils::Vector3d& a, const utils::Vector3d& b);
#endif
#ifndef BIORBD_USE_CASADI_MATH
    // One seed per surface instance, in its local frame. Copies own their state.
    // Seeds select a continued branch; they never replace feasibility checks.
    mutable std::vector<double> m_geodesicSeed;
    mutable utils::Vector3d m_geodesicStart, m_geodesicEnd;
#endif
    utils::RotoTrans m_RTtoParent;
    int m_side;
    bool m_active;
    utils::Vector3d m_contact1, m_contact2;
    utils::Scalar m_arcLength;
    std::vector<utils::Vector3d> m_surfacePoints;
};
} }
#endif
