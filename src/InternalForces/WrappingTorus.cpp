#define BIORBD_API_EXPORTS
#include "InternalForces/WrappingTorus.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "WrappingNumericalKernel.h"
using namespace BIORBD_NAMESPACE;

internal_forces::WrappingTorus::WrappingTorus(const utils::RotoTrans& rt,
    const utils::Scalar& majorRadius, const utils::Scalar& minorRadius,
    const utils::String& name, const utils::String& parentName) :
    WrappingSurface(rt,name,parentName), m_majorRadius(majorRadius), m_minorRadius(minorRadius)
{
#ifndef BIORBD_USE_CASADI_MATH
    if(!std::isfinite(majorRadius)||!std::isfinite(minorRadius)||minorRadius<=0||majorRadius<=minorRadius)
        throw std::invalid_argument("WrappingTorus requires finite majorRadius > minorRadius > 0");
#endif
    *m_typeOfNode=utils::NODE_TYPE::WRAPPING_TORUS;
}
internal_forces::WrappingTorus internal_forces::WrappingTorus::DeepCopy() const
{ WrappingTorus copy(*this); copy.DeepCopy(*this); return copy; }
void internal_forces::WrappingTorus::DeepCopy(const WrappingTorus& other)
{ WrappingSurface::DeepCopy(other);m_majorRadius=other.m_majorRadius;m_minorRadius=other.m_minorRadius; }
utils::Scalar internal_forces::WrappingTorus::majorRadius() const { return m_majorRadius; }
utils::Scalar internal_forces::WrappingTorus::minorRadius() const { return m_minorRadius; }

void internal_forces::WrappingTorus::solveLocal(const utils::Vector3d& a,
    const utils::Vector3d& b, std::vector<utils::Vector3d>& points, utils::Scalar& arcLength) const
{
#ifdef BIORBD_USE_CASADI_MATH
    (void)a;(void)b;(void)points;(void)arcLength;
    throw std::runtime_error("WrappingTorus requires the Eigen numerical backend");
#else
    biorbd::wrapping::Seed seed;
    seed.parameters=m_geodesicSeed;
    if(!seed.parameters.empty()) {seed.start=m_geodesicStart;seed.end=m_geodesicEnd;}
    const biorbd::wrapping::Shape shape(biorbd::wrapping::Shape::Torus,Eigen::Vector3d(m_majorRadius,m_minorRadius,0),m_side);
    const biorbd::wrapping::Contact result=biorbd::wrapping::solveLocal(shape,a,b,&seed);
    points.clear();for(const auto& point:result.samples) points.push_back(utils::Vector3d(point));
    arcLength=result.length;
    m_geodesicSeed=seed.parameters;m_geodesicStart=seed.start;m_geodesicEnd=seed.end;
#endif
}
