#define BIORBD_API_EXPORTS
#include "InternalForces/WrappingEllipsoid.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "WrappingGeodesic.h"
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
    double na=0,nb=0,ad=0,dd=0;
    for(int i=0;i<3;++i) {
        const double ai=a(i)/m_radii(i), bi=b(i)/m_radii(i), di=bi-ai;
        na+=ai*ai;nb+=bi*bi;ad+=ai*di;dd+=di*di;
    }
    if(na<1.-1e-12 || nb<1.-1e-12)
        throw std::invalid_argument("WrappingEllipsoid endpoint lies inside the surface");
    const double t=dd>0?std::max(0.,std::min(1.,-ad/dd)):0;
    if(na+2*t*ad+t*t*dd>=1.-1e-12) {m_geodesicSeed.clear();return;}
    double scale=0;
    for(int i=0;i<3;++i) scale=std::max(scale,std::max(m_radii(i),std::max(std::fabs(a(i)),std::fabs(b(i)))));
    int excluded=-1;
    // Prefer XY, then XZ, then YZ when more than one plane is available.
    for(int i=2;i>=0;--i)
        if(std::fabs(a(i))<=1e-10*scale && std::fabs(b(i))<=1e-10*scale) { excluded=i;break; }
    if(excluded<0) {
        namespace detail=wrapping_detail;
        detail::Surface surface(detail::Surface::Ellipsoid,m_radii);
        const Eigen::Vector3d targetA=Eigen::Vector3d(a)/surface.scale;
        const Eigen::Vector3d targetB=Eigen::Vector3d(b)/surface.scale;
        const auto store=[&](const detail::Result& result) {
            points.clear();for(const auto& p:result.points) points.push_back(utils::Vector3d(p*surface.scale));
            arcLength=result.length*surface.scale;
            m_geodesicSeed.assign(result.parameters.data(),result.parameters.data()+3);
            m_geodesicStart.DeepCopy(a);m_geodesicEnd.DeepCopy(b);
        };
        if(m_geodesicSeed.size()==3 && (a-m_geodesicStart).norm()<.5*surface.scale &&
            (b-m_geodesicEnd).norm()<.5*surface.scale) {
            try {store(detail::shoot(surface,targetA,targetB,Eigen::Map<const Eigen::Vector3d>(m_geodesicSeed.data()),true));return;}
            catch(const std::runtime_error&) {m_geodesicSeed.clear();}
        }
        Eigen::Vector3d baselineA=targetA,baselineB=targetB;
        baselineA[2]=baselineB[2]=0;
        Eigen::Quaterniond rotation=Eigen::Quaterniond::Identity();bool rotate=false;
        // If projection puts an endpoint inside, rotate in the scaled-sphere
        // coordinates instead: endpoint radii stay constant throughout continuation.
        if((baselineA.head<2>().cwiseQuotient(surface.dimensions.head<2>())).norm()<1.+1e-8 ||
            (baselineB.head<2>().cwiseQuotient(surface.dimensions.head<2>())).norm()<1.+1e-8) {
            const Eigen::Vector3d sa=targetA.cwiseQuotient(surface.dimensions),sb=targetB.cwiseQuotient(surface.dimensions);
            Eigen::Vector3d normal=sa.cross(sb);
            if(normal.norm()<1e-8) normal=sa.cross(Eigen::Vector3d::UnitY());
            if(normal.norm()<1e-8) normal=sa.cross(Eigen::Vector3d::UnitX());
            normal.normalize();if(normal[2]<0) normal=-normal;
            rotation=Eigen::Quaterniond::FromTwoVectors(normal,Eigen::Vector3d::UnitZ());
            baselineA=(rotation*sa).cwiseProduct(surface.dimensions);
            baselineB=(rotation*sb).cwiseProduct(surface.dimensions);baselineA[2]=baselineB[2]=0;rotate=true;
        }
        const PlanarArc baseline=ellipseArc(baselineA[0],baselineA[1],baselineB[0],baselineB[1],
            surface.dimensions[0],surface.dimensions[1]);
        Eigen::Vector3d seed(baseline.entry,0,std::log(baseline.length));
        const int steps=std::min(64,std::max(12,static_cast<int>(std::ceil(std::max((targetA-baselineA).norm(),(targetB-baselineB).norm())/.1))));
        for(int i=1;i<=steps;++i) {
            const double fraction=static_cast<double>(i)/steps;
            Eigen::Vector3d ca=(1-fraction)*baselineA+fraction*targetA,cb=(1-fraction)*baselineB+fraction*targetB;
            if(rotate) {
                const Eigen::Quaterniond q=rotation.slerp(fraction,Eigen::Quaterniond::Identity());
                ca=(q*targetA.cwiseQuotient(surface.dimensions)).cwiseProduct(surface.dimensions);
                cb=(q*targetB.cwiseQuotient(surface.dimensions)).cwiseProduct(surface.dimensions);
            }
            const detail::Result result=detail::shoot(surface,ca,cb,seed,i==steps);
            seed=result.parameters;if(i==steps) store(result);
        }
        return;
    }
    int u=excluded==0?1:0, v=excluded==2?1:2;
    PlanarArc arc=ellipseArc(a(u),a(v),b(u),b(v),m_radii(u),m_radii(v));
    for(const auto& p:arc.points) {
        utils::Vector3d mapped(0,0,0);mapped(u)=p(0);mapped(v)=p(1);
        points.push_back(mapped);
    }
    m_geodesicSeed.clear();
    arcLength=arc.length;
#endif
}
