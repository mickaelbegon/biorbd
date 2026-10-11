#define BIORBD_API_EXPORTS
#include "InternalForces/WrappingTorus.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "WrappingGeodesic.h"
using namespace BIORBD_NAMESPACE;

#ifndef BIORBD_USE_CASADI_MATH
namespace {
double evaluate(const std::vector<double>& coefficients, double t)
{
    double value=0;
    for(auto i=coefficients.rbegin();i!=coefficients.rend();++i) value=value*t+*i;
    return value;
}
// Isolate all roots in [0,1] by the derivative's monotonic intervals.
// The torus chord is quartic; no fixed sampling can miss a thin penetration.
std::vector<double> rootsOnChord(std::vector<double> coefficients)
{
    while(coefficients.size()>1 && coefficients.back()==0) coefficients.pop_back();
    if(coefficients.size()<2) return {};
    std::vector<double> derivative;
    for(size_t i=1;i<coefficients.size();++i) derivative.push_back(i*coefficients[i]);
    std::vector<double> cuts=rootsOnChord(derivative);
    cuts.push_back(0);cuts.push_back(1);
    std::sort(cuts.begin(),cuts.end());
    std::vector<double> roots;
    for(size_t i=1;i<cuts.size();++i) {
        double lo=cuts[i-1],hi=cuts[i], flo=evaluate(coefficients,lo), fhi=evaluate(coefficients,hi);
        if(flo==0) roots.push_back(lo);
        if(fhi==0) roots.push_back(hi);
        if((flo<0 && fhi>0)||(flo>0 && fhi<0)) {
            for(int iteration=0;iteration<64;++iteration) {
                const double mid=(lo+hi)/2., fm=evaluate(coefficients,mid);
                if((flo<0 && fm<=0)||(flo>0 && fm>=0)) { lo=mid;flo=fm; }
                else hi=mid;
            }
            roots.push_back((lo+hi)/2.);
        }
    }
    std::sort(roots.begin(),roots.end());
    roots.erase(std::unique(roots.begin(),roots.end()),roots.end());
    return roots;
}
bool torusChordIntersects(const utils::Vector3d& a, const utils::Vector3d& b,
    double major, double minor)
{
    double aa=0, ad=0, dd=0, radialA=0,radialAD=0,radialDD=0;
    for(int i=0;i<3;++i) {
        const double ai=a(i)/major, di=(b(i)-a(i))/major;
        aa+=ai*ai;ad+=ai*di;dd+=di*di;
        if(i<2) { radialA+=ai*ai;radialAD+=ai*di;radialDD+=di*di; }
    }
    const double ratio=minor/major,q0=aa+1.-ratio*ratio,q1=2*ad,q2=dd;
    const std::vector<double> coefficients={q0*q0-4*radialA,
        2*q0*q1-8*radialAD,q1*q1+2*q0*q2-4*radialDD,2*q1*q2,q2*q2};
    std::vector<double> derivative;
    for(size_t i=1;i<coefficients.size();++i) derivative.push_back(i*coefficients[i]);
    auto roots=rootsOnChord(derivative);
    roots.push_back(0);roots.push_back(1);
    double minimum=evaluate(coefficients,0);
    for(double t:roots) minimum=std::min(minimum,evaluate(coefficients,t));
    return minimum < -1e-12*ratio*ratio;
}
}
#endif

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
    const double major=m_majorRadius,minor=m_minorRadius;
    for(const auto* endpoint : {&a,&b})
        if(std::hypot(std::hypot((*endpoint)(0),(*endpoint)(1))-major,(*endpoint)(2))<minor*(1.-1e-12))
            throw std::invalid_argument("WrappingTorus endpoint lies inside the tube");
    if(!torusChordIntersects(a,b,major,minor)) {m_geodesicSeed.clear();return;}
    const double scale=std::max(major,std::max(a.norm(),b.norm())), tol=1e-10*scale;
    if(std::fabs(a(2))<=tol && std::fabs(b(2))<=tol &&
        std::hypot(a(0),a(1))>=major+minor-tol && std::hypot(b(0),b(1))>=major+minor-tol) {
        PlanarArc arc=ellipseArc(a(0),a(1),b(0),b(1),major+minor,major+minor);
        m_geodesicSeed.clear();arcLength=arc.length;points.swap(arc.points);return;
    }
    const double ra=std::hypot(a(0),a(1)),rb=std::hypot(b(0),b(1));
    double dx=1,dy=0;
    if(ra>tol) {dx=a(0)/ra;dy=a(1)/ra;}
    else if(rb>tol) {dx=b(0)/rb;dy=b(1)/rb;}
    // Give the meridian a canonical signed radial axis independent of endpoint order.
    if(dx < -1e-12 || (std::fabs(dx)<=1e-12 && dy<0)) {dx=-dx;dy=-dy;}
    if(std::fabs(a(0)*dy-a(1)*dx)>tol || std::fabs(b(0)*dy-b(1)*dx)>tol) {
        namespace detail=wrapping_detail;
        detail::Surface surface(detail::Surface::Torus,Eigen::Vector3d(major,minor,0));
        const Eigen::Vector3d targetA=Eigen::Vector3d(a)/surface.scale,targetB=Eigen::Vector3d(b)/surface.scale;
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
        Eigen::Vector3d baselineA=targetA,baselineB=targetB,seed;
        const double outer=surface.dimensions[0]+surface.dimensions[1];
        bool equatorial=ra>=major+minor && rb>=major+minor;
        double baseThetaA=0,baseThetaB=0,deltaThetaA=0,deltaThetaB=0;
        if(equatorial) {
            baselineA[2]=baselineB[2]=0;
            const PlanarArc arc=ellipseArc(baselineA[0],baselineA[1],baselineB[0],baselineB[1],outer,outer);
            seed=Eigen::Vector3d(arc.entry,0,std::log(arc.length));
        } else {
            // Rotate radial endpoints into a common meridian, preserving rho and z.
            // This keeps endpoints outside the tube at every continuation step.
            double ux=rb>tol?b(0)/rb:(ra>tol?a(0)/ra:1),uy=rb>tol?b(1)/rb:(ra>tol?a(1)/ra:0);
            if(ux < -1e-12 || (std::fabs(ux)<=1e-12 && uy<0)) {ux=-ux;uy=-uy;}
            const double signA=a(0)*ux+a(1)*uy>=0?1:-1,signB=b(0)*ux+b(1)*uy>=0?1:-1;
            const double au=signA*ra/surface.scale,bu=signB*rb/surface.scale;
            const double theta=std::atan2(uy,ux),pi=std::acos(-1.);
            baseThetaA=theta+(signA<0?pi:0);baseThetaB=theta+(signB<0?pi:0);
            deltaThetaA=std::atan2(a(1),a(0))-baseThetaA;deltaThetaB=std::atan2(b(1),b(0))-baseThetaB;
            while(deltaThetaA>pi) deltaThetaA-=2*pi;while(deltaThetaA<-pi) deltaThetaA+=2*pi;
            while(deltaThetaB>pi) deltaThetaB-=2*pi;while(deltaThetaB<-pi) deltaThetaB+=2*pi;
            const double R=surface.dimensions[0],r=surface.dimensions[1];
            const bool positive=intersectsEllipse(au,targetA[2],bu,targetB[2],r,r,R,0);
            const bool negative=intersectsEllipse(au,targetA[2],bu,targetB[2],r,r,-R,0);
            if(positive && negative) throw std::runtime_error("No connected torus meridian initialization: the baseline crosses both tubes");
            const double center=positive?R:(negative?-R:(au+bu>=0?R:-R));
            const PlanarArc arc=ellipseArc(au,targetA[2],bu,targetB[2],r,r,center,0);
            const utils::Vector3d p=arc.points.front();
            const Eigen::Vector3d entry(ux*p(0),uy*p(0),p(1));
            seed.head<2>()=surface.angles(entry);seed[2]=std::log(arc.length);
            baselineA=Eigen::Vector3d(std::cos(baseThetaA)*ra/surface.scale,std::sin(baseThetaA)*ra/surface.scale,targetA[2]);
            baselineB=Eigen::Vector3d(std::cos(baseThetaB)*rb/surface.scale,std::sin(baseThetaB)*rb/surface.scale,targetB[2]);
        }
        const int steps=std::min(64,std::max(16,static_cast<int>(std::ceil(std::max((targetA-baselineA).norm(),(targetB-baselineB).norm())/.1))));
        for(int i=1;i<=steps;++i) {
            const double fraction=static_cast<double>(i)/steps;
            Eigen::Vector3d ca=(1-fraction)*baselineA+fraction*targetA,cb=(1-fraction)*baselineB+fraction*targetB;
            if(!equatorial) {
                ca=Eigen::Vector3d(std::cos(baseThetaA+fraction*deltaThetaA)*ra/surface.scale,std::sin(baseThetaA+fraction*deltaThetaA)*ra/surface.scale,targetA[2]);
                cb=Eigen::Vector3d(std::cos(baseThetaB+fraction*deltaThetaB)*rb/surface.scale,std::sin(baseThetaB+fraction*deltaThetaB)*rb/surface.scale,targetB[2]);
            }
            const detail::Result result=detail::shoot(surface,ca,cb,seed,i==steps);
            seed=result.parameters;if(i==steps) store(result);
        }
        return;
    }
    const double au=a(0)*dx+a(1)*dy,bu=b(0)*dx+b(1)*dy;
    const bool positive=intersectsEllipse(au,a(2),bu,b(2),minor,minor,major,0);
    const bool negative=intersectsEllipse(au,a(2),bu,b(2),minor,minor,-major,0);
    if(positive && negative)
        throw std::runtime_error("WrappingTorus meridian requires multiple disconnected tube contacts, which are unsupported");
    if(!positive && !negative)
        throw std::runtime_error("WrappingTorus chord intersection is inconsistent with its symmetry plane");
    const double center=positive?major:-major;
    PlanarArc arc=ellipseArc(au,a(2),bu,b(2),minor,minor,center,0);
    const auto& p=arc.points.front();const auto& q=arc.points.back();
    if(intersectsEllipse(au,a(2),p(0),p(1),minor,minor,-center,0) ||
        intersectsEllipse(q(0),q(1),bu,b(2),minor,minor,-center,0))
        throw std::runtime_error("WrappingTorus branch intersects the opposite tube; multiple disconnected contacts are unsupported");
    for(const auto& point:arc.points)
        points.push_back(utils::Vector3d(dx*point(0),dy*point(0),point(1)));
    m_geodesicSeed.clear();
    arcLength=arc.length;
#endif
}
