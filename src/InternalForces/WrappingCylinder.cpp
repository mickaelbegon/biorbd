#define BIORBD_API_EXPORTS
#include "InternalForces/WrappingCylinder.h"
#include "WrappingNumericalKernel.h"
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
    biorbd::wrapping::Seed seed;
    seed.parameters=m_geodesicSeed;
    if(!seed.parameters.empty()) {seed.start=m_geodesicStart;seed.end=m_geodesicEnd;}
    const biorbd::wrapping::Shape shape(biorbd::wrapping::Shape::Cylinder,Eigen::Vector3d(m_radius,m_length,0),m_side);
    const biorbd::wrapping::Contact result=biorbd::wrapping::solveLocal(shape,a,b,&seed);
    points.clear();for(const auto& point:result.samples) points.push_back(utils::Vector3d(point));
    arcLength=result.length;
    m_geodesicSeed=seed.parameters;m_geodesicStart=seed.start;m_geodesicEnd=seed.end;
#endif
}
