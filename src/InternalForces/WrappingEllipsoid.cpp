#define BIORBD_API_EXPORTS
#include "InternalForces/WrappingEllipsoid.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "WrappingNumericalKernel.h"
#ifndef BIORBD_USE_CASADI_MATH
#include <Eigen/Geometry>
#endif
using namespace BIORBD_NAMESPACE;
internal_forces::WrappingEllipsoid::WrappingEllipsoid(const utils::RotoTrans& rt,
    const utils::Vector3d& radii, const utils::String& name, const utils::String& parentName) :
    WrappingSurface(rt,name,parentName), m_radii(radii)
{
#ifndef BIORBD_USE_CASADI_MATH
    for(int i=0;i<3;++i) if(!std::isfinite(radii(i)) || radii(i)<=0)
        throw std::invalid_argument("WrappingEllipsoid requires finite positive semiaxes");
#endif
    *m_typeOfNode=utils::NODE_TYPE::WRAPPING_ELLIPSOID;
}
internal_forces::WrappingEllipsoid internal_forces::WrappingEllipsoid::DeepCopy() const
{ WrappingEllipsoid copy(*this); copy.DeepCopy(*this); return copy; }
void internal_forces::WrappingEllipsoid::DeepCopy(const WrappingEllipsoid& other)
{ WrappingSurface::DeepCopy(other); m_radii.DeepCopy(other.m_radii); }
const utils::Vector3d& internal_forces::WrappingEllipsoid::radii() const { return m_radii; }
void internal_forces::WrappingEllipsoid::solveLocal(const utils::Vector3d& a,
    const utils::Vector3d& b, std::vector<utils::Vector3d>& points, utils::Scalar& arcLength) const
{
#ifdef BIORBD_USE_CASADI_MATH
    (void)a;(void)b;(void)points;(void)arcLength;
    throw std::runtime_error("WrappingEllipsoid requires the Eigen numerical backend");
#else
    biorbd::wrapping::Seed seed;
    seed.parameters=m_geodesicSeed;
    if(!seed.parameters.empty()) {seed.start=m_geodesicStart;seed.end=m_geodesicEnd;}
    const biorbd::wrapping::Shape shape(biorbd::wrapping::Shape::Ellipsoid,Eigen::Vector3d(m_radii),m_side);
    const biorbd::wrapping::Contact result=biorbd::wrapping::solveLocal(shape,a,b,&seed);
    points.clear();for(const auto& point:result.samples) points.push_back(utils::Vector3d(point));
    arcLength=result.length;
    m_geodesicSeed=seed.parameters;m_geodesicStart=seed.start;m_geodesicEnd=seed.end;
#endif
}
