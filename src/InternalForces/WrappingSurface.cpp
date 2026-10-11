#define BIORBD_API_EXPORTS
#include "InternalForces/WrappingSurface.h"
#include "RigidBody/Joints.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace BIORBD_NAMESPACE;

internal_forces::WrappingSurface::WrappingSurface(const utils::RotoTrans& rt,
    const utils::String& name, const utils::String& parentName) :
    WrappingObject(rt.trans(), name, parentName), m_RTtoParent(rt), m_side(1),
    m_active(false), m_contact1(0,0,0), m_contact2(0,0,0), m_arcLength(0)
{}

internal_forces::WrappingSurface::WrappingSurface(const WrappingSurface& other) :
    WrappingObject(), m_side(1), m_active(false), m_arcLength(0)
{ DeepCopy(other); }

internal_forces::WrappingSurface& internal_forces::WrappingSurface::operator=(
    const WrappingSurface& other)
{
    if (this != &other) DeepCopy(other);
    return *this;
}
internal_forces::WrappingSurface::~WrappingSurface() {}

void internal_forces::WrappingSurface::DeepCopy(const WrappingSurface& other)
{
    if (this == &other) return;
    // Node metadata and RT in the legacy base are shared pointers. Detach them
    // before copying: new surfaces must never share a mutable result cache.
    m_name = std::make_shared<utils::String>(other.utils::Node::name());
    m_parentName = std::make_shared<utils::String>(other.parent());
    m_typeOfNode = std::make_shared<utils::NODE_TYPE>(other.typeOfNode());
    m_RT = std::make_shared<utils::RotoTrans>(*other.m_RT);
    WrappingObject::DeepCopy(other);
#ifndef BIORBD_USE_CASADI_MATH
    m_geodesicSeed = other.m_geodesicSeed;
    m_geodesicStart.DeepCopy(other.m_geodesicStart);
    m_geodesicEnd.DeepCopy(other.m_geodesicEnd);
#endif
    m_RTtoParent = other.m_RTtoParent;
    m_side = other.m_side;
    m_active = other.m_active;
    m_contact1.DeepCopy(other.m_contact1);
    m_contact2.DeepCopy(other.m_contact2);
    m_arcLength = other.m_arcLength;
    m_surfacePoints.clear();
    for (const auto& point : other.m_surfacePoints) {
        utils::Vector3d copy;
        copy.DeepCopy(point);
        m_surfacePoints.push_back(copy);
    }
}

void internal_forces::WrappingSurface::setWrappingSide(int side)
{
    if (side != 1 && side != -1)
        throw std::invalid_argument("Wrapping side must be +1 or -1");
    m_side = side;
#ifndef BIORBD_USE_CASADI_MATH
    m_geodesicSeed.clear();
#endif
    m_active = false;
    m_surfacePoints.clear();
    m_arcLength = 0;
}
int internal_forces::WrappingSurface::wrappingSide() const { return m_side; }
bool internal_forces::WrappingSurface::isActive() const { return m_active; }
const std::vector<utils::Vector3d>& internal_forces::WrappingSurface::surfacePoints() const
{ return m_surfacePoints; }

void internal_forces::WrappingSurface::wrapPoints(const utils::RotoTrans& rt,
    const utils::Vector3d& a, const utils::Vector3d& b,
    utils::Vector3d& p1, utils::Vector3d& p2, utils::Scalar* length)
{
    // Invalidate first so a failed solve cannot expose stale contacts.
    m_active = false;
    m_surfacePoints.clear();
    m_arcLength = 0;
    m_contact1.DeepCopy(a);
    m_contact2.DeepCopy(b);
#ifdef BIORBD_USE_CASADI_MATH
    (void)rt; (void)p1; (void)p2; (void)length;
    throw std::runtime_error("New wrapping surfaces require the Eigen numerical backend; CasADi MX wrapping is not supported");
#else
    validateEndpoints(a,b);
    utils::Vector3d localA(a), localB(b);
    localA.applyRT(rt.transpose());
    localB.applyRT(rt.transpose());
    std::vector<utils::Vector3d> points;
    utils::Scalar arcLength = 0;
    solveLocal(localA, localB, points, arcLength);
    if (!points.empty()) {
        if (points.size() < 2 || !std::isfinite(arcLength) || arcLength <= 0)
            throw std::runtime_error("Invalid wrapping contact curve");
        for (auto& point : points) point.applyRT(rt);
        m_surfacePoints.swap(points);
        m_contact1.DeepCopy(m_surfacePoints.front());
        m_contact2.DeepCopy(m_surfacePoints.back());
        m_arcLength = arcLength;
        m_active = true;
    }
    *m_RT = rt;
    wrapPoints(p1,p2,length);
#endif
}

void internal_forces::WrappingSurface::wrapPoints(rigidbody::Joints& model,
    const rigidbody::GeneralizedCoordinates& Q, const utils::Vector3d& a,
    const utils::Vector3d& b, utils::Vector3d& p1, utils::Vector3d& p2,
    utils::Scalar* length)
{ wrapPoints(RT(model,Q), a,b,p1,p2,length); }

void internal_forces::WrappingSurface::wrapPoints(utils::Vector3d& p1,
    utils::Vector3d& p2, utils::Scalar* length)
{
    p1.DeepCopy(m_contact1);
    p2.DeepCopy(m_contact2);
    if (length) *length = m_arcLength;
}

const utils::RotoTrans& internal_forces::WrappingSurface::RT(rigidbody::Joints& model,
    const rigidbody::GeneralizedCoordinates& Q, bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    if (updateKin) model.UpdateKinematicsCustom(&Q);
    *m_RT = model.globalJCS(*m_parentName) * m_RTtoParent;
    return *m_RT;
}

#ifndef BIORBD_USE_CASADI_MATH
void internal_forces::WrappingSurface::validateEndpoints(const utils::Vector3d& a,
    const utils::Vector3d& b)
{
    for (int i=0; i<3; ++i)
        if (!std::isfinite(a(i)) || !std::isfinite(b(i)))
            throw std::invalid_argument("Wrapping endpoints must be finite");
}

bool internal_forces::WrappingSurface::intersectsEllipse(double ax, double ay,
    double bx, double by, double rx, double ry, double cx, double cy)
{
    ax=(ax-cx)/rx; ay=(ay-cy)/ry;
    bx=(bx-cx)/rx; by=(by-cy)/ry;
    const double dx=bx-ax, dy=by-ay, n=dx*dx+dy*dy;
    const double t=n > 0 ? std::max(0.,std::min(1.,-(ax*dx+ay*dy)/n)) : 0;
    return std::hypot(ax+t*dx,ay+t*dy) < 1.-1e-12;
}

internal_forces::WrappingSurface::PlanarArc internal_forces::WrappingSurface::ellipseArc(
    double ax, double ay, double bx, double by,
    double rx, double ry, double cx, double cy) const
{
    const double pi=std::acos(-1.);
    const double na=std::hypot((ax-cx)/rx,(ay-cy)/ry);
    const double nb=std::hypot((bx-cx)/rx,(by-cy)/ry);
    if (na < 1.-1e-12 || nb < 1.-1e-12)
        throw std::invalid_argument("Wrapping endpoint lies inside the surface");
    const double pha=std::atan2((ay-cy)/ry,(ax-cx)/rx);
    const double phb=std::atan2((by-cy)/ry,(bx-cx)/rx);
    const double da=std::acos(std::min(1.,1./na));
    const double db=std::acos(std::min(1.,1./nb));
    PlanarArc best;
    double bestScore=-std::numeric_limits<double>::infinity();
    bool found=false;
    for (int sign=-1; sign<=1; sign+=2) {
        // Select contacts whose tangent velocity agrees with traversal.
        const double entry=pha+sign*da, exit=phb-sign*db;
        double sweep=exit-entry;
        if (sign>0) { while(sweep<0) sweep+=2*pi; }
        else { while(sweep>0) sweep-=2*pi; }
        if (std::fabs(sweep)<1e-12) continue;
        const double ex=cx+rx*std::cos(entry), ey=cy+ry*std::sin(entry);
        const double qx=cx+rx*std::cos(exit), qy=cy+ry*std::sin(exit);
        const double vinx=ex-ax, viny=ey-ay, voutx=bx-qx, vouty=by-qy;
        const double dotIn=sign*(-rx*std::sin(entry)*vinx+ry*std::cos(entry)*viny);
        const double dotOut=sign*(-rx*std::sin(exit)*voutx+ry*std::cos(exit)*vouty);
        if (dotIn < -1e-12 || dotOut < -1e-12) continue;
        // Side is defined by the midpoint of the contact arc in the local frame.
        const double score=m_side*ry*std::sin(entry+sweep/2.);
        if (found && score<=bestScore) continue;
        best.entry=entry; best.sweep=sweep;
        best.points.clear();
        // At most 0.001 rad per segment; samples are exact surface points.
        const int count=std::max(32,static_cast<int>(std::ceil(std::fabs(sweep)/.001)));
        for (int i=0;i<=count;++i) {
            const double t=entry+sweep*i/count;
            best.points.push_back(utils::Vector3d(cx+rx*std::cos(t),cy+ry*std::sin(t),0));
        }
        // Composite Simpson with convergence check, independent of display samples.
        auto speed=[rx,ry](double t) { return std::hypot(rx*std::sin(t),ry*std::cos(t)); };
        double previous=0, integral=0;
        bool converged=false;
        for(int n=32;n<=65536;n*=2) {
            const double h=sweep/n;
            double sum=speed(entry)+speed(entry+sweep);
            for(int i=1;i<n;++i) sum+=(i%2?4:2)*speed(entry+h*i);
            integral=std::fabs(h)*sum/3.;
            if(n>32 && std::fabs(integral-previous)<=1e-12*std::max(rx,ry)) {
                converged=true; break;
            }
            previous=integral;
        }
        if(!converged) throw std::runtime_error("Wrapping ellipse arc integration did not converge");
        best.length=integral;
        bestScore=score; found=true;
    }
    if(!found) throw std::runtime_error("No consistent tangent branch for wrapping surface");
    return best;
}
#endif
