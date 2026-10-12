#include "WrappingNgv.h"
#include <cmath>
#include <memory>
#include <stdexcept>
#ifdef BIORBD_WITH_SIMBODY_WRAPPING
#include <Simbody.h>
#endif
namespace biorbd { namespace wrapping {
bool ngvAvailable()
{
#ifdef BIORBD_WITH_SIMBODY_WRAPPING
    return true;
#else
    return false;
#endif
}
#ifdef BIORBD_WITH_SIMBODY_WRAPPING
namespace {
SimTK::Vec3 simVector(const Eigen::Vector3d& p) {return SimTK::Vec3(p[0],p[1],p[2]);}
Eigen::Vector3d eigenVector(const SimTK::Vec3& p) {return Eigen::Vector3d(p[0],p[1],p[2]);}
SimTK::Transform simPose(const Eigen::Matrix4d& p)
{
    SimTK::Mat33 r;for(int i=0;i<3;++i)for(int j=0;j<3;++j)r(i,j)=p(i,j);
    return SimTK::Transform(SimTK::Rotation(r),simVector(p.block<3,1>(0,3)));
}
Eigen::Vector3d frameAxis(const SimTK::Transform& p,int axis)
{return Eigen::Vector3d(p.R().asMat33()(0,axis),p.R().asMat33()(1,axis),p.R().asMat33()(2,axis));}
std::shared_ptr<const SimTK::ContactGeometry> contactGeometry(const Shape& shape)
{
    if(shape.kind==Shape::Cylinder) return std::make_shared<SimTK::ContactGeometry::Cylinder>(shape.dimensions[0]);
    if(shape.kind==Shape::Ellipsoid) return std::make_shared<SimTK::ContactGeometry::Ellipsoid>(simVector(shape.dimensions));
    return std::make_shared<SimTK::ContactGeometry::Torus>(shape.dimensions[0],shape.dimensions[1]);
}
// All Simbody state and caches belong to this one call. In particular no
// automatic discrete variables are promoted between model evaluations.
class ColdRegionAdapter {
    SimTK::MultibodySystem system;
    SimTK::SimbodyMatterSubsystem matter;
    SimTK::CableSubsystem cables;
    std::vector<SimTK::MobilizedBody::Free> bodies;
    SimTK::CableSpan cable;
    std::vector<SimTK::CableSpanObstacleIndex> indices;
public:
    ColdRegionAdapter(const std::vector<Object>& objects,const Region& region,
                      const std::vector<Eigen::Vector3d>& hints):matter(system),cables(system)
    {
        if(hints.size()!=region.objects.size()) throw std::invalid_argument("NGV hint count does not match the region");
        const SimTK::Body::Rigid body(SimTK::MassProperties(1.,SimTK::Vec3(0),SimTK::Inertia(1.)));
        bodies.reserve(region.objects.size());
        cable=SimTK::CableSpan(cables,matter.Ground(),simVector(region.start),matter.Ground(),simVector(region.end));
        for(std::size_t i=0;i<region.objects.size();++i) {
            const Object& object=objects.at(region.objects[i]);
            if(!hints[i].allFinite()) throw std::invalid_argument("NGV hints must be finite");
            bodies.emplace_back(matter.Ground(),body);
            bodies.back().setDefaultTransform(simPose(object.pose));
            indices.push_back(cable.addObstacle(bodies.back(),SimTK::Transform(),
                contactGeometry(object.shape),simVector(hints[i])));
        }
        cable.setAlgorithm(SimTK::CableSpanAlgorithm::Scholz2015);
        cable.setUseWarmStart(false);
        cable.setCurveSegmentAccuracy(1e-12);
        cable.setSmoothnessTolerance(2e-11);
        cable.setSolverMaxIterations(200);
    }
    NgvRegion solve(const Region& region)
    {
        system.realizeTopology();SimTK::State state=system.getDefaultState();
        system.realize(state,SimTK::Stage::Position);
        const double length=cable.calcLength(state);
        NgvRegion result;result.length=length;result.smoothness=cable.getSmoothness(state);
        result.iterations=cable.getNumSolverIterations(state);
        if(!std::isfinite(length)||!std::isfinite(result.smoothness)||result.smoothness>2e-11)
            throw std::runtime_error("Scholz2015 wrapping did not converge to its smoothness tolerance");
        result.contacts.resize(indices.size());
        for(std::size_t i=0;i<indices.size();++i) {
            NgvContact& c=result.contacts[i];c.curve.entry=region.start;c.curve.exit=region.end;
            c.entryTangent.setZero();c.exitTangent.setZero();c.entryNormal.setZero();c.exitNormal.setZero();
            c.curve.active=cable.isInContactWithObstacle(state,indices[i]);
            if(!c.curve.active) continue;
            const SimTK::Transform p=cable.calcCurveSegmentInitialFrenetFrame(state,indices[i]);
            const SimTK::Transform q=cable.calcCurveSegmentFinalFrenetFrame(state,indices[i]);
            c.curve.entry=eigenVector(p.p());c.curve.exit=eigenVector(q.p());
            c.entryTangent=frameAxis(p,0);c.exitTangent=frameAxis(q,0);
            c.entryNormal=frameAxis(p,1);c.exitNormal=frameAxis(q,1);
            c.curve.length=cable.calcCurveSegmentArcLength(state,indices[i]);
            cable.calcCurveSegmentResampledPoints(state,indices[i],1025,
                [&](const SimTK::Vec3& point){c.curve.samples.push_back(eigenVector(point));});
        }
        return result;
    }
};
}
#endif
NgvRegion solveNgvRegion(const std::vector<Object>& objects,const Region& region,
                        const std::vector<Eigen::Vector3d>& hints)
{
#ifdef BIORBD_WITH_SIMBODY_WRAPPING
    return ColdRegionAdapter(objects,region,hints).solve(region);
#else
    (void)objects;(void)region;(void)hints;
    throw std::runtime_error("Simbody wrapping was not enabled at build time");
#endif
}
} }
