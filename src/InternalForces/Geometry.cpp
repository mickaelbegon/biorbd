#define BIORBD_API_EXPORTS

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include "Utils/Error.h"
#include "Utils/Matrix.h"
#include "Utils/RotoTrans.h"
#include "RigidBody/NodeSegment.h"
#include "RigidBody/Joints.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "InternalForces/WrappingObject.h"
#include "InternalForces/WrappingSurface.h"
#include "InternalForces/WrappingCylinder.h"
#include "InternalForces/WrappingEllipsoid.h"
#include "InternalForces/WrappingTorus.h"
#include "InternalForces/PathModifiers.h"
#include "InternalForces/ViaPoint.h"
#include "InternalForces/Geometry.h"
#ifdef BIORBD_USE_CASADI_MATH
#include "WrappingCasadi.h"
#endif

using namespace BIORBD_NAMESPACE;

#ifndef BIORBD_USE_CASADI_MATH
namespace {
// Minimize an implicit surface level on a line segment. A negative result
// detects penetration, including the two disconnected lobes of a ring torus.
double surfaceLevelOnSegment(const internal_forces::WrappingSurface& surface,
    const utils::RotoTrans& rt, const utils::Vector3d& a, const utils::Vector3d& b)
{
    Eigen::Vector3d p = rt.block(0,0,3,3).transpose() *
        (a - rt.block(0,3,3,1));
    Eigen::Vector3d d = rt.block(0,0,3,3).transpose() * (b-a);
    if (const internal_forces::WrappingEllipsoid* ellipsoid =
        dynamic_cast<const internal_forces::WrappingEllipsoid*>(&surface)) {
        p = p.cwiseQuotient(ellipsoid->radii());
        d = d.cwiseQuotient(ellipsoid->radii());
        const double t = d.squaredNorm() == 0 ? 0 :
            std::max(0.,std::min(1.,-p.dot(d)/d.squaredNorm()));
        return (p+t*d).squaredNorm()-1.;
    }
    if (const internal_forces::WrappingCylinder* cylinder =
        dynamic_cast<const internal_forces::WrappingCylinder*>(&surface)) {
        double lo=0., hi=1.;
        const double half=cylinder->length()/2.;
        if (std::abs(d[2]) < 1e-15) {
            if (std::abs(p[2]) > half) return 1.;
        } else {
            double t1=(-half-p[2])/d[2], t2=(half-p[2])/d[2];
            if(t1>t2) std::swap(t1,t2);
            lo=std::max(lo,t1); hi=std::min(hi,t2);
            if(lo>hi) return 1.;
        }
        const double dd=d.head<2>().squaredNorm();
        const double t=dd == 0 ? lo :
            std::max(lo,std::min(hi,-p.head<2>().dot(d.head<2>())/dd));
        return (p+t*d).head<2>().squaredNorm()/
            (cylinder->radius()*cylinder->radius())-1.;
    }
    const internal_forces::WrappingTorus* torus =
        dynamic_cast<const internal_forces::WrappingTorus*>(&surface);
    utils::Error::check(torus != nullptr,"Unknown wrapping surface");
    const double R=torus->majorRadius(), r=torus->minorRadius();
    const double aa=d.squaredNorm(), bb=2*p.dot(d);
    const double cc=p.squaredNorm()+R*R-r*r;
    const double c[5]={cc*cc-4*R*R*p.head<2>().squaredNorm(),
        2*bb*cc-8*R*R*p.head<2>().dot(d.head<2>()),
        bb*bb+2*aa*cc-4*R*R*d.head<2>().squaredNorm(),
        2*aa*bb,aa*aa};
    const auto value=[&](double t) {
        return ((((c[4]*t+c[3])*t+c[2])*t+c[1])*t+c[0]);
    };
    const auto derivative=[&](double t) {
        return (((4*c[4]*t+3*c[3])*t+2*c[2])*t+c[1]);
    };
    std::vector<double> cuts{0.,1.};
    // Turning points of the cubic derivative split it into monotone intervals.
    const double qa=12*c[4], qb=6*c[3], qc=2*c[2];
    if(qa != 0) {
        const double disc=qb*qb-4*qa*qc;
        if(disc>=0) {
            const double delta=std::sqrt(disc);
            for(double t : {(-qb-delta)/(2*qa),(-qb+delta)/(2*qa)})
                if(t>0 && t<1) cuts.push_back(t);
        }
    } else if(qb != 0) {
        const double t=-qc/qb;
        if(t>0 && t<1) cuts.push_back(t);
    }
    std::sort(cuts.begin(),cuts.end());
    double result=std::min(value(0),value(1));
    for(size_t i=0;i<cuts.size();++i) {
        result=std::min(result,value(cuts[i]));
        if(i+1==cuts.size()) break;
        double lo=cuts[i],hi=cuts[i+1],flo=derivative(lo),fhi=derivative(hi);
        if((flo<0 && fhi>0) || (flo>0 && fhi<0)) {
            for(int j=0;j<70;++j) {
                const double mid=(lo+hi)/2.,fm=derivative(mid);
                if((flo<=0 && fm<=0) || (flo>=0 && fm>=0)) {lo=mid;flo=fm;}
                else hi=mid;
            }
            result=std::min(result,value((lo+hi)/2.));
        }
    }
    return result/std::pow(R+r,4);
}

struct SurfaceContact {
    internal_forces::WrappingSurface* surface;
    utils::RotoTrans rt;
    utils::Vector3d entry,exit;
    double length;
    bool active;
    std::vector<utils::Vector3d> samples;
};

void solveSurfaceRegion(std::vector<SurfaceContact>& contacts,
    const utils::Vector3d& start,const utils::Vector3d& end)
{
    for(SurfaceContact& c : contacts) {
        c.entry=start;c.exit=end;c.length=0;c.active=false;c.samples.clear();
    }
    // Bootstrap only multi-object regions. These provisional planar contacts
    // supply neighbours before the first true solve; they are never delivered.
    // In particular, a standalone geodesic branch can lose continuation while
    // the shorter coupled branch remains well defined.
    if(contacts.size()>1) for(SurfaceContact& c : contacts) {
        utils::Vector3d a=start,b=end;
        const internal_forces::WrappingEllipsoid* ellipsoid=
            dynamic_cast<const internal_forces::WrappingEllipsoid*>(c.surface);
        const internal_forces::WrappingTorus* torus=
            dynamic_cast<const internal_forces::WrappingTorus*>(c.surface);
        if(ellipsoid || torus) {
            a.applyRT(c.rt.transpose());b.applyRT(c.rt.transpose());
            a(2)=b(2)=0.;
            if(ellipsoid) {
                const auto& r=ellipsoid->radii();
                if(std::hypot(a(0)/r(0),a(1)/r(1))<1. ||
                    std::hypot(b(0)/r(0),b(1)/r(1))<1.) continue;
            } else {
                const double outer=torus->majorRadius()+torus->minorRadius();
                if(std::hypot(a(0),a(1))<outer || std::hypot(b(0),b(1))<outer) continue;
            }
            a.applyRT(c.rt);b.applyRT(c.rt);
        }
        try {
            utils::Vector3d p,q;utils::Scalar length=0;
            c.surface->wrapPoints(c.rt,a,b,p,q,&length);
            if(c.surface->isActive()) {
                c.entry=p;c.exit=q;c.length=length;c.active=true;
                c.samples=c.surface->surfacePoints();
            }
        } catch(const std::invalid_argument&) {
            // An inadmissible optional guess leaves this object inactive.
        } catch(const std::runtime_error&) {
            // The strict solve below still propagates every true solve failure.
        }
    }
    bool converged=false;
    const double tolerance=1e-10*std::max(1.,(end-start).norm());
    for(unsigned int iteration=0;iteration<200;++iteration) {
        double change=0.;bool changedActive=false;
        for(size_t i=0;i<contacts.size();++i) {
            utils::Vector3d a=start,b=end;
            for(size_t j=i;j>0;--j) if(contacts[j-1].active) {a=contacts[j-1].exit;break;}
            for(size_t j=i+1;j<contacts.size();++j) if(contacts[j].active) {b=contacts[j].entry;break;}
            SurfaceContact& c=contacts[i];
            utils::Vector3d p,q;utils::Scalar length=0;
            c.surface->wrapPoints(c.rt,a,b,p,q,&length);
            const bool active=c.surface->isActive();
            changedActive=changedActive || active!=c.active;
            if(active) change=std::max(change,std::max((p-c.entry).norm(),(q-c.exit).norm()));
            change=std::max(change,std::abs(length-c.length));
            c.entry=p;c.exit=q;c.length=length;c.active=active;
            c.samples=c.surface->surfacePoints();
        }
        if(!changedActive && change<=tolerance) {converged=true;break;}
    }
    utils::Error::check(converged,"Coupled wrapping contacts did not converge");
    // Verify free segments against every associated solid. Verify sampled
    // contact curves against the other objects; intersecting configurations
    // are rejected rather than silently accepted as feasible trajectories.
    utils::Vector3d previous=start;
    const auto checkLine=[&](const utils::Vector3d& a,const utils::Vector3d& b,
                            const internal_forces::WrappingSurface* owner) {
        for(const SurfaceContact& other : contacts) if(other.surface!=owner)
            utils::Error::check(surfaceLevelOnSegment(*other.surface,other.rt,a,b)>=-1e-8,
                "Wrapping trajectory intersects another object; this contact configuration is unsupported");
    };
    for(const SurfaceContact& c : contacts) if(c.active) {
        checkLine(previous,c.entry,nullptr);
        for(size_t i=1;i<c.samples.size();++i)
            checkLine(c.samples[i-1],c.samples[i],c.surface);
        previous=c.exit;
    }
    checkLine(previous,end,nullptr);
}
}
#endif

internal_forces::Geometry::Geometry() :
    m_origin(std::make_shared<utils::Vector3d>()),
    m_insertion(std::make_shared<utils::Vector3d>()),
    m_originInGlobal(std::make_shared<utils::Vector3d>
                     (utils::Vector3d::Zero())),
    m_insertionInGlobal(std::make_shared<utils::Vector3d>
                        (utils::Vector3d::Zero())),
    m_pointsInGlobal(std::make_shared<std::vector<utils::Vector3d>>()),
    m_pointsInLocal(std::make_shared<std::vector<utils::Vector3d>>()),
    m_pathPointsInGlobal(std::make_shared<std::vector<utils::Vector3d>>()),
    m_surfaceSegmentLengths(std::make_shared<std::vector<utils::Scalar>>()),
    m_surfaceSegments(std::make_shared<std::vector<bool>>()),
    m_surfacePath(std::make_shared<bool>(false)),
    m_jacobian(std::make_shared<utils::Matrix>()),
    m_G(std::make_shared<utils::Matrix>()),
    m_jacobianLength(std::make_shared<utils::Matrix>()),
    m_length(std::make_shared<utils::Scalar>(0)),
    m_velocity(std::make_shared<utils::Scalar>(0)),
    m_isGeometryComputed(std::make_shared<bool>(false)),
    m_isVelocityComputed(std::make_shared<bool>(false)),
    m_posAndJacoWereForced(std::make_shared<bool>(false))
{

}

internal_forces::Geometry::Geometry(
    const utils::Vector3d &origin,
    const utils::Vector3d &insertion) :
    m_origin(std::make_shared<utils::Vector3d>(origin)),
    m_insertion(std::make_shared<utils::Vector3d>(insertion)),
    m_originInGlobal(std::make_shared<utils::Vector3d>
                     (utils::Vector3d::Zero())),
    m_insertionInGlobal(std::make_shared<utils::Vector3d>
                        (utils::Vector3d::Zero())),
    m_pointsInGlobal(std::make_shared<std::vector<utils::Vector3d>>()),
    m_pointsInLocal(std::make_shared<std::vector<utils::Vector3d>>()),
    m_pathPointsInGlobal(std::make_shared<std::vector<utils::Vector3d>>()),
    m_surfaceSegmentLengths(std::make_shared<std::vector<utils::Scalar>>()),
    m_surfaceSegments(std::make_shared<std::vector<bool>>()),
    m_surfacePath(std::make_shared<bool>(false)),
    m_jacobian(std::make_shared<utils::Matrix>()),
    m_G(std::make_shared<utils::Matrix>()),
    m_jacobianLength(std::make_shared<utils::Matrix>()),
    m_length(std::make_shared<utils::Scalar>(0)),
    m_velocity(std::make_shared<utils::Scalar>(0)),
    m_isGeometryComputed(std::make_shared<bool>(false)),
    m_isVelocityComputed(std::make_shared<bool>(false)),
    m_posAndJacoWereForced(std::make_shared<bool>(false))
{

}

internal_forces::Geometry internal_forces::Geometry::DeepCopy() const
{
    internal_forces::Geometry copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::Geometry::DeepCopy(const internal_forces::Geometry &other)
{
    *m_origin = other.m_origin->DeepCopy();
    *m_insertion = other.m_insertion->DeepCopy();
    *m_originInGlobal = other.m_originInGlobal->DeepCopy();
    *m_insertionInGlobal = other.m_insertionInGlobal->DeepCopy();
    m_pointsInGlobal->resize(other.m_pointsInGlobal->size());
    for (unsigned int i=0; i<other.m_pointsInGlobal->size(); ++i) {
        (*m_pointsInGlobal)[i] = (*other.m_pointsInGlobal)[i].DeepCopy();
    }
    m_pointsInLocal->resize(other.m_pointsInLocal->size());
    for (unsigned int i=0; i<other.m_pointsInLocal->size(); ++i) {
        (*m_pointsInLocal)[i] = (*other.m_pointsInLocal)[i].DeepCopy();
    }
    *m_jacobian = *other.m_jacobian;
    *m_pathPointsInGlobal = *other.m_pathPointsInGlobal;
    *m_surfaceSegmentLengths = *other.m_surfaceSegmentLengths;
    *m_surfaceSegments = *other.m_surfaceSegments;
    *m_surfacePath = *other.m_surfacePath;
    *m_G = *other.m_G;
    *m_jacobianLength = *other.m_jacobianLength;
    *m_length = *other.m_length;
    *m_velocity = *other.m_velocity;
    *m_isGeometryComputed = *other.m_isGeometryComputed;
    *m_isVelocityComputed = *other.m_isVelocityComputed;
    *m_posAndJacoWereForced = *other.m_posAndJacoWereForced;
}


// ------ PUBLIC FUNCTIONS ------ //
void internal_forces::Geometry::updateKinematics(
    rigidbody::Joints &model,
    const rigidbody::GeneralizedCoordinates *Q,
    const rigidbody::GeneralizedVelocity *Qdot,
    int updateKin)
{
    if (*m_posAndJacoWereForced) {
        utils::Error::warning(
            false,
            "Warning, using updateKinematics overrides the"
            " previously sent position and jacobian");
        *m_posAndJacoWereForced = false;
    }

    // Make sure the model is in the right configuration
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = 2;
#endif
    if (updateKin > 1) {
        model.UpdateKinematicsCustom(Q, Qdot, nullptr);
    }

    // Position of the points in space
    setPointsInGlobal(model, *Q);

    // Compute the Jacobian of the muscle points
    jacobian(model, *Q);

    // Complete the update
    _updateKinematics(Qdot, nullptr);
}
void internal_forces::Geometry::updateKinematics(rigidbody::Joints
        &model,
        internal_forces::PathModifiers &pathModifiers,
        const rigidbody::GeneralizedCoordinates *Q,
        const rigidbody::GeneralizedVelocity *Qdot,
        int updateKin)
{
    if (*m_posAndJacoWereForced) {
        utils::Error::warning(
            false, "Warning, using updateKinematics overrides the"
            " previously sent position and jacobian");
        *m_posAndJacoWereForced = false;
    }
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = 2;
#endif

    // Ensure the model is in the right configuration
    if (updateKin > 1) {
        model.UpdateKinematicsCustom(Q, Qdot);
    }

    // Position of the points in space
    setPointsInGlobal(model, *Q, &pathModifiers);

    // Compute the Jacobian of the muscle points
    jacobian(model, *Q);

    // Complete the update
    _updateKinematics(Qdot, &pathModifiers);
}

void internal_forces::Geometry::updateKinematics(
    std::vector<utils::Vector3d>& pointsInGlobal,
    utils::Matrix& jacoPointsInGlobal,
    const rigidbody::GeneralizedVelocity* Qdot)
{
    *m_posAndJacoWereForced = true;

    // Position of the points in space
    setPointsInGlobal(pointsInGlobal);

    // Compute the Jacobian of the muscle points
    jacobian(jacoPointsInGlobal);

    // Complete the update
    _updateKinematics(Qdot, nullptr);
}

// Get and set the positions of the origins and insertions
void internal_forces::Geometry::setOrigin(
    const utils::Vector3d &position)
{
    if (dynamic_cast<const rigidbody::NodeSegment*>(&position)) {
        *m_origin = position;
    } else {
        // Preserve the Node information
        m_origin->RigidBodyDynamics::Math::Vector3d::operator=(position);
    }
}
const utils::Vector3d& internal_forces::Geometry::originInLocal() const
{
    return *m_origin;
}

void internal_forces::Geometry::setInsertionInLocal(
    const utils::Vector3d &position)
{
    if (dynamic_cast<const rigidbody::NodeSegment*>(&position)) {
        *m_insertion = position;
    } else {
        // Preserve the Node information
        m_insertion->RigidBodyDynamics::Math::Vector3d::operator=(position);
    }
}
const utils::Vector3d &internal_forces::Geometry::insertionInLocal()
const
{
    return *m_insertion;
}

// Position of the muscles in space
const utils::Vector3d &internal_forces::Geometry::originInGlobal() const
{
    utils::Error::check(*m_isGeometryComputed,
                                "Geometry must be computed at least once before calling originInLocal()");
    return *m_originInGlobal;
}
const utils::Vector3d &internal_forces::Geometry::insertionInGlobal()
const
{
    utils::Error::check(*m_isGeometryComputed,
                                "Geometry must be computed at least once before calling insertionInGlobal()");
    return *m_insertionInGlobal;
}
const std::vector<utils::Vector3d>
&internal_forces::Geometry::pointsInGlobal() const
{
    utils::Error::check(*m_isGeometryComputed,
                                "Geometry must be computed at least once before calling musclesPointsInGlobal()");
    return *m_pointsInGlobal;
}

// Return the length and muscular velocity
const std::vector<utils::Vector3d>& internal_forces::Geometry::pathPointsInGlobal() const
{
    utils::Error::check(*m_isGeometryComputed,"Geometry must be computed before reading the trajectory");
    return *m_surfacePath ? *m_pathPointsInGlobal : *m_pointsInGlobal;
}

const utils::Scalar& internal_forces::Geometry::length() const
{
    utils::Error::check(*m_isGeometryComputed,
                                "Geometry must be computed at least before calling length()");
    return *m_length;
}

const utils::Scalar& internal_forces::Geometry::velocity() const
{
    utils::Error::check(*m_isVelocityComputed,
                                "Geometry must be computed before calling velocity()");
    return *m_velocity;
}

// Return the Jacobian
const utils::Matrix& internal_forces::Geometry::jacobian() const
{
    utils::Error::check(*m_isGeometryComputed,
                                "Geometry must be computed before calling jacobian()");
    return *m_jacobian;
} // Return the last Jacobian
utils::Matrix internal_forces::Geometry::jacobianOrigin() const
{
    utils::Error::check(*m_isGeometryComputed,
                                "Geometry must be computed before calling jacobianOrigin()");
    return m_jacobian->block(0,0,3,m_jacobian->cols());
}
utils::Matrix internal_forces::Geometry::jacobianInsertion() const
{
    utils::Error::check(*m_isGeometryComputed,
                                "Geometry must be computed before calling jacobianInsertion()");
    return m_jacobian->block(m_jacobian->rows()-3,0,3,m_jacobian->cols());
}
utils::Matrix internal_forces::Geometry::jacobian(
    unsigned int idxViaPoint) const
{
    utils::Error::check(*m_isGeometryComputed,
                                "Geometry must be computed before calling jacobian(i)");
    return m_jacobian->block(3*idxViaPoint,0,3,m_jacobian->cols());
}

const utils::Matrix &internal_forces::Geometry::jacobianLength() const
{
    utils::Error::check(*m_isGeometryComputed,
                                "Geometry must be computed before calling jacobianLength()");
    return *m_jacobianLength;
}

// --------------------------------------- //

void internal_forces::Geometry::_updateKinematics(
    const rigidbody::GeneralizedVelocity* Qdot,
    internal_forces::PathModifiers *pathModifiers)
{
    // Compute the length and velocities
    length(pathModifiers);
    *m_isGeometryComputed = true;

    // Compute the jacobian of the lengths
    computeJacobianLength();
    if (Qdot != nullptr) {
        velocity(*Qdot);
        *m_isVelocityComputed = true;
    } else {
        *m_isVelocityComputed = false;
    }
}

const utils::Vector3d &internal_forces::Geometry::originInGlobal(
    rigidbody::Joints &model,
    const rigidbody::GeneralizedCoordinates &Q)
{
    // Return the position of the marker in function of the given position
    m_originInGlobal->block(0,0,3,
                            1) = RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q,
                                    model.GetBodyId(m_origin->parent().c_str()), *m_origin,false);
    return *m_originInGlobal;
}

const utils::Vector3d &internal_forces::Geometry::insertionInGlobal(
    rigidbody::Joints &model,
    const rigidbody::GeneralizedCoordinates &Q)
{
    // Return the position of the marker in function of the given position
    m_insertionInGlobal->block(0,0,3,1) = RigidBodyDynamics::CalcBodyToBaseCoordinates(
                model, Q, model.GetBodyId(m_insertion->parent().c_str()), *m_insertion,false);
    return *m_insertionInGlobal;
}

void internal_forces::Geometry::setPointsInGlobal(
    std::vector<utils::Vector3d> &ptsInGlobal)
{
    *m_isGeometryComputed = false;
    *m_isVelocityComputed = false;
    *m_surfacePath = false;
    m_surfaceSegments->clear();
    m_surfaceSegmentLengths->clear();
    m_pathPointsInGlobal->clear();
    utils::Error::check(ptsInGlobal.size() >= 2,
                                "ptsInGlobal must at least have an origin and an insertion");
    m_pointsInLocal->clear(); // In this mode, we don't need the local, because the Jacobian of the points has to be given as well
    *m_pointsInGlobal = ptsInGlobal;
}

void internal_forces::Geometry::setPointsInGlobal(
    rigidbody::Joints &model,
    const rigidbody::GeneralizedCoordinates &Q,
    internal_forces::PathModifiers *pathModifiers)
{
    *m_isGeometryComputed = false;
    *m_isVelocityComputed = false;
    // Output varible (reset to zero)
    m_pointsInLocal->clear();
    m_pointsInGlobal->clear();

    *m_surfacePath = false;
    m_pathPointsInGlobal->clear();
    m_surfaceSegments->clear();
    m_surfaceSegmentLengths->clear();
    if(pathModifiers==nullptr || pathModifiers->nbObjects()==0) {
        m_pointsInLocal->push_back(originInLocal());
        m_pointsInLocal->push_back(insertionInLocal());
        m_pointsInGlobal->push_back(originInGlobal(model,Q));
        m_pointsInGlobal->push_back(insertionInGlobal(model,Q));
        setJacobianDimension(model);
        return;
    }
    bool hasSurface=false;
    for(unsigned int i=0;i<pathModifiers->nbObjects();++i)
        hasSurface=hasSurface || dynamic_cast<WrappingSurface*>(&pathModifiers->object(i))!=nullptr;
    if(hasSurface) {
#ifdef BIORBD_USE_CASADI_MATH
        utils::Error::check(model.nbQ()==model.dof_count,
            "CasADi new wrapping surfaces require Q_size == dof_count; quaternion configurations are unsupported");
        // Build the immutable kinematic snapshot with an independent pure Q.
        // This also accepts a caller's constant Q or a slice of a larger graph.
        rigidbody::Joints snapshot=model.DeepCopy();
        const casadi::MX input=casadi::MX::sym("wrapping_q",model.nbQ());
        const rigidbody::GeneralizedCoordinates coordinates(input);
        snapshot.UpdateKinematicsCustom(&coordinates);
        std::vector<casadi::MX> anchors;
        std::vector<utils::Vector3d> localAnchors;
        std::vector<casadi::MX> poses;
        std::vector<WrappingSurface*> surfaces;
        std::vector<::biorbd::wrapping::Shape> shapes;
        std::vector<std::vector<size_t>> regions(1);
        const auto anchor=[&](const utils::Vector3d& node) {
            const utils::Vector3d global(RigidBodyDynamics::CalcBodyToBaseCoordinates(
                snapshot,coordinates,snapshot.GetBodyId(node.parent().c_str()),node,false));
            anchors.push_back(global);localAnchors.push_back(node);
        };
        const auto dimension=[](const casadi::MX& value) {
            utils::Error::check(casadi::MX::symvar(value).empty(),
                "CasADi wrapping dimensions must be constant numerical values");
            return static_cast<double>(casadi::MX::evalf(value));
        };
        anchor(originInLocal());
        for(unsigned int i=0;i<pathModifiers->nbObjects();++i) {
            utils::Vector3d& object=pathModifiers->object(i);
            if(object.typeOfNode()==utils::NODE_TYPE::VIA_POINT) {
                anchor(object);regions.emplace_back();
                continue;
            }
            WrappingSurface* surface=dynamic_cast<WrappingSurface*>(&object);
            utils::Error::check(surface!=nullptr,
                "Legacy wrapping objects cannot yet be combined with the new surface solver");
            using Shape=::biorbd::wrapping::Shape;
            std::unique_ptr<WrappingSurface> copy;
            if(const WrappingCylinder* cylinder=dynamic_cast<const WrappingCylinder*>(surface)) {
                shapes.emplace_back(Shape::Cylinder,Eigen::Vector3d(
                    dimension(cylinder->radius()),dimension(cylinder->length()),0),surface->wrappingSide());
                copy.reset(new WrappingCylinder(*cylinder));
            } else if(const WrappingEllipsoid* ellipsoid=dynamic_cast<const WrappingEllipsoid*>(surface)) {
                const auto& radii=ellipsoid->radii();
                shapes.emplace_back(Shape::Ellipsoid,Eigen::Vector3d(
                    dimension(radii(0)),dimension(radii(1)),dimension(radii(2))),surface->wrappingSide());
                copy.reset(new WrappingEllipsoid(*ellipsoid));
            } else if(const WrappingTorus* torus=dynamic_cast<const WrappingTorus*>(surface)) {
                shapes.emplace_back(Shape::Torus,Eigen::Vector3d(
                    dimension(torus->majorRadius()),dimension(torus->minorRadius()),0),surface->wrappingSide());
                copy.reset(new WrappingTorus(*torus));
            } else utils::Error::raise("Unknown wrapping surface");
            const casadi::MX pose=copy->RT(snapshot,coordinates,false);
            poses.push_back(casadi::MX::reshape(pose,16,1));
            regions.back().push_back(surfaces.size());surfaces.push_back(surface);
        }
        anchor(insertionInLocal());
        std::vector<casadi::MX> values=anchors;
        values.insert(values.end(),poses.begin(),poses.end());
        const casadi::MX kinematicVector=casadi::MX::densify(casadi::MX::vertcat(values));
        const casadi::Function kinematics("wrapping_kinematics",{input},
            {kinematicVector,casadi::MX::densify(casadi::MX::jacobian(kinematicVector,input))});
        const CasadiWrappingExpressions expressions=makeCasadiWrapping(
            kinematics,static_cast<const casadi::MX&>(Q),shapes,regions,anchors.size());
        *m_surfacePath=true;
        *m_length=expressions.length;
        *m_jacobianLength=utils::Matrix(expressions.gradient);
        const auto append=[&](const utils::Vector3d& local) {
            const size_t i=m_pointsInGlobal->size();
            m_pointsInGlobal->push_back(utils::Vector3d(RigidBodyDynamics::Math::VectorNd(casadi::MX(expressions.points(casadi::Slice(static_cast<casadi_int>(3*i),static_cast<casadi_int>(3*i+3)))))));
            m_pointsInLocal->push_back(local);
        };
        append(originInLocal());
        for(size_t r=0;r<regions.size();++r) {
            for(size_t index : regions[r]) {
                WrappingSurface* surface=surfaces[index];
                const unsigned int body=model.GetBodyId(surface->parent().c_str());
                for(int contact=0;contact<2;++contact) {
                    const size_t i=m_pointsInGlobal->size();
                    const utils::Vector3d global(RigidBodyDynamics::Math::VectorNd(casadi::MX(expressions.points(casadi::Slice(static_cast<casadi_int>(3*i),static_cast<casadi_int>(3*i+3))))));
                    const utils::Vector3d local(RigidBodyDynamics::CalcBaseToBodyCoordinates(
                        model,Q,body,global,false),contact==0 ? "wrap_entry" : "wrap_exit",surface->parent());
                    append(local);
                }
            }
            append(localAnchors[r+1]);
        }
        *m_originInGlobal=m_pointsInGlobal->front();
        *m_insertionInGlobal=m_pointsInGlobal->back();
        *m_pathPointsInGlobal=*m_pointsInGlobal;
        setJacobianDimension(model);
        return;
#else
        *m_surfacePath=true;
        const utils::Vector3d start=originInGlobal(model,Q);
        const utils::Vector3d finish=insertionInGlobal(model,Q);
        m_pointsInLocal->push_back(originInLocal());
        m_pointsInGlobal->push_back(start);
        m_pathPointsInGlobal->push_back(start);
        std::vector<SurfaceContact> region, allContacts;
        utils::Vector3d regionStart=start;
        const auto appendNode=[&](const utils::Vector3d& global,
            const utils::Vector3d& local,bool arc,double arcLength) {
            m_pointsInGlobal->push_back(global);
            m_pointsInLocal->push_back(local);
            m_surfaceSegments->push_back(arc);
            m_surfaceSegmentLengths->push_back(arcLength);
        };
        const auto appendRegion=[&](const utils::Vector3d& end,const utils::Vector3d& localEnd) {
            solveSurfaceRegion(region,regionStart,end);
            for(const SurfaceContact& c : region) if(c.active) {
                const unsigned int body=model.GetBodyId(c.surface->parent().c_str());
                const utils::Vector3d localEntry(RigidBodyDynamics::CalcBaseToBodyCoordinates(
                    model,Q,body,c.entry,false),"wrap_entry",c.surface->parent());
                const utils::Vector3d localExit(RigidBodyDynamics::CalcBaseToBodyCoordinates(
                    model,Q,body,c.exit,false),"wrap_exit",c.surface->parent());
                appendNode(c.entry,localEntry,false,0.);
                appendNode(c.exit,localExit,true,c.length);
                m_pathPointsInGlobal->insert(m_pathPointsInGlobal->end(),c.samples.begin(),c.samples.end());
            }
            appendNode(end,localEnd,false,0.);
            m_pathPointsInGlobal->push_back(end);
            allContacts.insert(allContacts.end(),region.begin(),region.end());
            region.clear();regionStart=end;
        };
        for(unsigned int i=0;i<pathModifiers->nbObjects();++i) {
            utils::Vector3d& object=pathModifiers->object(i);
            if(object.typeOfNode()==utils::NODE_TYPE::VIA_POINT) {
                const utils::Vector3d global(RigidBodyDynamics::CalcBodyToBaseCoordinates(
                    model,Q,model.GetBodyId(object.parent().c_str()),object,false));
                appendRegion(global,object);
            } else {
                WrappingSurface* surface=dynamic_cast<WrappingSurface*>(&object);
                utils::Error::check(surface!=nullptr,
                    "Legacy wrapping objects cannot yet be combined with the new surface solver");
                SurfaceContact contact;
                contact.surface=surface;contact.rt=surface->RT(model,Q,false);
                contact.active=false;contact.length=0.;
                region.push_back(contact);
            }
        }
        appendRegion(finish,insertionInLocal());
        // Via points fix the anatomical order, but every solid constrains the
        // complete path, including the regions on either side of a via point.
        const auto checkAllObjects=[&](const utils::Vector3d& a,const utils::Vector3d& b,
            const WrappingSurface* owner) {
            for(const SurfaceContact& c : allContacts) if(c.surface!=owner)
                utils::Error::check(surfaceLevelOnSegment(*c.surface,c.rt,a,b)>=-1e-8,
                    "Wrapping trajectory crosses an object outside its assigned via-point region");
        };
        for(size_t i=0;i<m_surfaceSegments->size();++i)
            if(!(*m_surfaceSegments)[i])
                checkAllObjects((*m_pointsInGlobal)[i],(*m_pointsInGlobal)[i+1],nullptr);
        for(const SurfaceContact& c : allContacts) if(c.active)
            for(size_t i=1;i<c.samples.size();++i)
                checkAllObjects(c.samples[i-1],c.samples[i],c.surface);
        setJacobianDimension(model);
        return;
#endif
    }

    // Do not apply on wrapping objects
    if (pathModifiers->nbWraps()!=0) {
        // CHECK TO MODIFY BEFOR GOING FORWARD WITH PROJECTS
        utils::Error::check(pathModifiers->nbVia() == 0,
                                    "Cannot mix wrapping and via points yet") ;
        utils::Error::check(pathModifiers->nbWraps() < 2,
                                    "Cannot compute more than one wrapping yet");

        // Get the matrix of Rt of the wrap
        internal_forces::WrappingObject& w =
            static_cast<internal_forces::WrappingObject&>(pathModifiers->object(0));
        const utils::RotoTrans& RT = w.RT(model,Q);

        // Alias
        const utils::Vector3d& po_mus = originInGlobal(model,
                                                Q);  // Origin on bone
        const utils::Vector3d& pi_mus = insertionInGlobal(model,
                                                Q); // Insertion on bone

        utils::Vector3d pi_wrap(0, 0,
                                        0); // point on the wrapping related to insertion
        utils::Vector3d po_wrap(0, 0,
                                        0); // point on the wrapping related to origin

        utils::Scalar a; // Force the computation of the length
        w.wrapPoints(RT,po_mus,pi_mus,po_wrap, pi_wrap, &a);

        // Store the points in local
        m_pointsInLocal->push_back(originInLocal());
        m_pointsInLocal->push_back(
            utils::Vector3d(RigidBodyDynamics::CalcBodyToBaseCoordinates(
                                        model, Q, model.GetBodyId(w.parent().c_str()),po_wrap, false),
                                    "wrap_o", w.parent()));
        m_pointsInLocal->push_back(
            utils::Vector3d(RigidBodyDynamics::CalcBodyToBaseCoordinates(
                                        model, Q, model.GetBodyId(w.parent().c_str()),pi_wrap, false),
                                    "wrap_i", w.parent()));
        m_pointsInLocal->push_back(insertionInLocal());

        // Store the points in global
        m_pointsInGlobal->push_back(po_mus);
        m_pointsInGlobal->push_back(po_wrap);
        m_pointsInGlobal->push_back(pi_wrap);
        m_pointsInGlobal->push_back(pi_mus);

    }

    else if (pathModifiers->nbObjects()!=0
             && pathModifiers->object(0).typeOfNode() == utils::NODE_TYPE::VIA_POINT) {
        m_pointsInLocal->push_back(originInLocal());
        m_pointsInGlobal->push_back(originInGlobal(model, Q));
        for (unsigned int i=0; i<pathModifiers->nbObjects(); ++i) {
            const internal_forces::ViaPoint& node(static_cast<internal_forces::ViaPoint&>
                                                  (pathModifiers->object(i)));
            m_pointsInLocal->push_back(node);
            m_pointsInGlobal->push_back(RigidBodyDynamics::CalcBodyToBaseCoordinates(model,
                                        Q,
                                        model.GetBodyId(node.parent().c_str()), node, false));
        }
        m_pointsInLocal->push_back(insertionInLocal());
        m_pointsInGlobal->push_back(insertionInGlobal(model,Q));

    } else if (pathModifiers->nbObjects()==0) {
        m_pointsInLocal->push_back(originInLocal());
        m_pointsInLocal->push_back(insertionInLocal());
        m_pointsInGlobal->push_back(originInGlobal(model, Q));
        m_pointsInGlobal->push_back(insertionInGlobal(model,Q));
    } else {
        utils::Error::raise("Length for this type of object was not implemented");
    }

    // Set the dimension of jacobian
    setJacobianDimension(model);
}

const utils::Scalar& internal_forces::Geometry::length(
    internal_forces::PathModifiers *pathModifiers)
{
#ifdef BIORBD_USE_CASADI_MATH
    if(*m_surfacePath) return *m_length;
#endif
    *m_length = 0;

    if(*m_surfacePath) {
        for(size_t i=0;i<m_surfaceSegments->size();++i)
            *m_length += (*m_surfaceSegments)[i] ? (*m_surfaceSegmentLengths)[i] :
                ((*m_pointsInGlobal)[i+1]-(*m_pointsInGlobal)[i]).norm();
        return *m_length;
    }

    // because we can't combine, test the first (0) will let us know all the types if more than one
    if (pathModifiers != nullptr && pathModifiers->nbWraps()!=0) {
        utils::Error::check(pathModifiers->nbVia() == 0,
                                    "Cannot mix wrapping and via points yet" ) ;
        utils::Error::check(pathModifiers->nbWraps() < 2,
                                    "Cannot compute more than one wrapping yet");

        utils::Vector3d pi_wrap(0, 0,
                                        0); // point on the wrapping related to insertion
        utils::Vector3d po_wrap(0, 0,
                                        0); // point on the wrapping related to origin
        utils::Scalar lengthWrap(0);
        static_cast<internal_forces::WrappingObject&>(
            pathModifiers->object(0)).wrapPoints(po_wrap, pi_wrap, &lengthWrap);
        *m_length = ((*m_pointsInGlobal)[0] - po_wrap).norm()
                                + // length before the wrap
                                lengthWrap                 + // length on the wrap
                                ((*m_pointsInGlobal)[3] - pi_wrap).norm();   // length after the wrap

    } else {
        for (unsigned int i=0; i<m_pointsInGlobal->size()-1; ++i) {
            *m_length += ((*m_pointsInGlobal)[i+1] - (*m_pointsInGlobal)[i]).norm();
        }
    }
    return *m_length;
}

const utils::Scalar& internal_forces::Geometry::velocity(
    const rigidbody::GeneralizedVelocity &Qdot)
{
    // Compute the velocity of the muscular elongation
    *m_velocity = (jacobianLength()*Qdot)[0];
    return *m_velocity;
}

void internal_forces::Geometry::setJacobianDimension(rigidbody::Joints
        &model)
{
    *m_jacobian = utils::Matrix::Zero(static_cast<unsigned int>
                  (m_pointsInLocal->size()*3), model.dof_count);
    *m_G = utils::Matrix::Zero(3, model.dof_count);
}

void internal_forces::Geometry::jacobian(const utils::Matrix &jaco)
{
    utils::Error::check(jaco.rows()/3 == static_cast<int>
                                (m_pointsInGlobal->size()), "Jacobian is the wrong size");
    *m_jacobian = jaco;
}

void internal_forces::Geometry::jacobian(
    rigidbody::Joints &model,
    const rigidbody::GeneralizedCoordinates &Q)
{
    for (unsigned int i=0; i<m_pointsInLocal->size(); ++i) {
        m_G->setZero();
        RigidBodyDynamics::CalcPointJacobian(model, Q,
                                             model.GetBodyId((*m_pointsInLocal)[i].parent().c_str()),
                                             (*m_pointsInLocal)[i], *m_G, false); // False for speed
        m_jacobian->block(3*i,0,3,model.dof_count) = *m_G;
    }
}

void internal_forces::Geometry::computeJacobianLength()
{
#ifdef BIORBD_USE_CASADI_MATH
    if(*m_surfacePath) return; // Exact stationary gradient supplied by the callback.
#endif
    *m_jacobianLength = utils::Matrix::Zero(1, m_jacobian->cols());

    // jacobian approximates as if there were no wrapping object
    const std::vector<utils::Vector3d>& p = *m_pointsInGlobal;
    for (unsigned int i=0; i<p.size()-1 ; ++i) {
        // At a stationary frictionless contact path, sliding-contact terms
        // cancel by tangency. Surface arc lengths are invariant under rigid
        // object motion, so only free-segment material velocities contribute.
        if(*m_surfacePath && (*m_surfaceSegments)[i]) continue;
#ifndef BIORBD_USE_CASADI_MATH
        if((p[i+1]-p[i]).norm() <= 1e-14) continue;
#endif
        *m_jacobianLength += (( p[i+1] - p[i] ).transpose() * (jacobian(i+1) - jacobian(
                                  i)))
                             /
                             ( p[i+1] - p[i] ).norm();
    }
}
