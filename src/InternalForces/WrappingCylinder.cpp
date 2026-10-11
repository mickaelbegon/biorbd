#define BIORBD_API_EXPORTS
#include "InternalForces/WrappingCylinder.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
using namespace BIORBD_NAMESPACE;

internal_forces::WrappingCylinder::WrappingCylinder(const utils::RotoTrans& rt,
    const utils::Scalar& radius, const utils::Scalar& length,
    const utils::String& name, const utils::String& parentName) :
    WrappingSurface(rt,name,parentName), m_radius(radius), m_length(length)
{
#ifndef BIORBD_USE_CASADI_MATH
    if(!std::isfinite(radius) || !std::isfinite(length) || radius<=0 || length<=0)
        throw std::invalid_argument("WrappingCylinder requires finite positive radius and length");
#endif
    *m_typeOfNode=utils::NODE_TYPE::WRAPPING_CYLINDER;
}
internal_forces::WrappingCylinder internal_forces::WrappingCylinder::DeepCopy() const
{ WrappingCylinder copy(*this); copy.DeepCopy(*this); return copy; }
void internal_forces::WrappingCylinder::DeepCopy(const WrappingCylinder& other)
{ WrappingSurface::DeepCopy(other); m_radius=other.m_radius; m_length=other.m_length; }
utils::Scalar internal_forces::WrappingCylinder::radius() const { return m_radius; }
const utils::Scalar& internal_forces::WrappingCylinder::length() const { return m_length; }

void internal_forces::WrappingCylinder::solveLocal(const utils::Vector3d& a,
    const utils::Vector3d& b, std::vector<utils::Vector3d>& points, utils::Scalar& arcLength) const
{
#ifdef BIORBD_USE_CASADI_MATH
    (void)a;(void)b;(void)points;(void)arcLength;
    throw std::runtime_error("WrappingCylinder requires the Eigen numerical backend");
#else
    const double r=m_radius, half=m_length/2.;
    // Clip the chord to the cylinder height before checking radial intersection.
    double lo=0,hi=1;
    const double dz=b(2)-a(2);
    if(std::fabs(dz)<1e-15) {
        if(std::fabs(a(2))>half) return;
    } else {
        double t0=(-half-a(2))/dz, t1=(half-a(2))/dz;
        if(t0>t1) std::swap(t0,t1);
        lo=std::max(0.,t0);hi=std::min(1.,t1);
        if(lo>=hi) return;
    }
    if(!intersectsEllipse(a(0)+(b(0)-a(0))*lo,a(1)+(b(1)-a(1))*lo,
        a(0)+(b(0)-a(0))*hi,a(1)+(b(1)-a(1))*hi,r,r)) return;
    if(std::hypot(a(0),a(1))<r*(1.-1e-12) || std::hypot(b(0),b(1))<r*(1.-1e-12))
        throw std::runtime_error("Finite-cylinder wrapping requires exterior radial endpoints; rim/cap contact is unsupported");
    PlanarArc arc=ellipseArc(a(0),a(1),b(0),b(1),r,r);
    const auto& p=arc.points.front(); const auto& q=arc.points.back();
    const double freeA=std::hypot(p(0)-a(0),p(1)-a(1));
    const double freeB=std::hypot(b(0)-q(0),b(1)-q(1));
    const double total=freeA+arc.length+freeB;
    const double z1=a(2)+dz*freeA/total, z2=a(2)+dz*(freeA+arc.length)/total;
    const double tol=1e-10*std::max(r,m_length);
    if(std::fabs(z1)>half+tol || std::fabs(z2)>half+tol)
        throw std::runtime_error("Finite-cylinder contact lies beyond the lateral surface; rim/cap contact is unsupported");
    const int count=static_cast<int>(arc.points.size())-1;
    for(int i=0;i<=count;++i) arc.points[i](2)=z1+(z2-z1)*i/count;
    arcLength=std::hypot(arc.length,z2-z1);
    points.swap(arc.points);
#endif
}
