#include "WrappingNumericalKernel.h"
#include "WrappingGeodesic.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
using namespace biorbd::wrapping;
namespace biorbd { namespace wrapping {
namespace {
class LocalSolver {
public:
    LocalSolver(const Shape& shape,Seed& seed):m_side(shape.side),
        m_radius(shape.dimensions[0]),m_length(shape.dimensions[1]),m_radii(shape.dimensions),
        m_majorRadius(shape.dimensions[0]),m_minorRadius(shape.dimensions[1]),
        m_geodesicSeed(seed.parameters),m_geodesicStart(seed.start),m_geodesicEnd(seed.end) {}
    struct PlanarArc {double entry,sweep,length;std::vector<Eigen::Vector3d> points;};
    PlanarArc ellipseArc(double ax,double ay,double bx,double by,double rx,double ry,
                        double cx=0,double cy=0) const;
    static bool intersectsEllipse(double ax,double ay,double bx,double by,double rx,double ry,
                                  double cx=0,double cy=0);
    void solveCylinder(const Eigen::Vector3d&,const Eigen::Vector3d&,std::vector<Eigen::Vector3d>&,double&) const;
    void solveEllipsoid(const Eigen::Vector3d&,const Eigen::Vector3d&,std::vector<Eigen::Vector3d>&,double&) const;
    void solveTorus(const Eigen::Vector3d&,const Eigen::Vector3d&,std::vector<Eigen::Vector3d>&,double&) const;
private:
    int m_side;
    double m_radius,m_length;
    Eigen::Vector3d m_radii;
    double m_majorRadius,m_minorRadius;
    std::vector<double>& m_geodesicSeed;
    Eigen::Vector3d& m_geodesicStart;
    Eigen::Vector3d& m_geodesicEnd;
};
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
bool torusChordIntersects(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
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
bool LocalSolver::intersectsEllipse(double ax, double ay,
    double bx, double by, double rx, double ry, double cx, double cy)
{
    ax=(ax-cx)/rx; ay=(ay-cy)/ry;
    bx=(bx-cx)/rx; by=(by-cy)/ry;
    const double dx=bx-ax, dy=by-ay, n=dx*dx+dy*dy;
    const double t=n > 0 ? std::max(0.,std::min(1.,-(ax*dx+ay*dy)/n)) : 0;
    return std::hypot(ax+t*dx,ay+t*dy) < 1.-1e-12;
}

LocalSolver::PlanarArc LocalSolver::ellipseArc(
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
            best.points.push_back(Eigen::Vector3d(cx+rx*std::cos(t),cy+ry*std::sin(t),0));
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

void LocalSolver::solveCylinder(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
    std::vector<Eigen::Vector3d>& points,double& arcLength) const
{
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
}
void LocalSolver::solveEllipsoid(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
    std::vector<Eigen::Vector3d>& points,double& arcLength) const
{
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
        namespace detail=::biorbd::wrapping;
        detail::Surface surface(detail::Surface::Ellipsoid,m_radii);
        const Eigen::Vector3d targetA=Eigen::Vector3d(a)/surface.scale;
        const Eigen::Vector3d targetB=Eigen::Vector3d(b)/surface.scale;
        const auto store=[&](const detail::Result& result) {
            points.clear();for(const auto& p:result.points) points.push_back(Eigen::Vector3d(p*surface.scale));
            arcLength=result.length*surface.scale;
            m_geodesicSeed.assign(result.parameters.data(),result.parameters.data()+3);
            m_geodesicStart=a;m_geodesicEnd=b;
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
        Eigen::Vector3d mapped(0,0,0);mapped(u)=p(0);mapped(v)=p(1);
        points.push_back(mapped);
    }
    m_geodesicSeed.clear();
    arcLength=arc.length;
}
void LocalSolver::solveTorus(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
    std::vector<Eigen::Vector3d>& points,double& arcLength) const
{
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
        namespace detail=::biorbd::wrapping;
        detail::Surface surface(detail::Surface::Torus,Eigen::Vector3d(major,minor,0));
        const Eigen::Vector3d targetA=Eigen::Vector3d(a)/surface.scale,targetB=Eigen::Vector3d(b)/surface.scale;
        const auto store=[&](const detail::Result& result) {
            points.clear();for(const auto& p:result.points) points.push_back(Eigen::Vector3d(p*surface.scale));
            arcLength=result.length*surface.scale;
            m_geodesicSeed.assign(result.parameters.data(),result.parameters.data()+3);
            m_geodesicStart=a;m_geodesicEnd=b;
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
            const Eigen::Vector3d p=arc.points.front();
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
        points.push_back(Eigen::Vector3d(dx*point(0),dy*point(0),point(1)));
    m_geodesicSeed.clear();
    arcLength=arc.length;
}

} // anonymous namespace

double surfaceLevelOnSegment(const Object& object, const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    Eigen::Vector3d p = object.pose.block(0,0,3,3).transpose() *
        (a - object.pose.block(0,3,3,1));
    Eigen::Vector3d d = object.pose.block(0,0,3,3).transpose() * (b-a);
    if (object.shape.kind==Shape::Ellipsoid) {
        p = p.cwiseQuotient(object.shape.dimensions);
        d = d.cwiseQuotient(object.shape.dimensions);
        const double t = d.squaredNorm() == 0 ? 0 :
            std::max(0.,std::min(1.,-p.dot(d)/d.squaredNorm()));
        return (p+t*d).squaredNorm()-1.;
    }
    if (object.shape.kind==Shape::Cylinder) {
        double lo=0., hi=1.;
        const double half=object.shape.dimensions[1]/2.;
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
            (object.shape.dimensions[0]*object.shape.dimensions[0])-1.;
    }
    const double R=object.shape.dimensions[0], r=object.shape.dimensions[1];
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


Shape::Shape(Kind kind_,const Eigen::Vector3d& dimensions_,int side_):
    kind(kind_),dimensions(dimensions_),side(side_)
{
    if(side!=1 && side!=-1) throw std::invalid_argument("Wrapping side must be +1 or -1");
    if(!dimensions.allFinite()) throw std::invalid_argument("Wrapping dimensions must be finite");
    if(kind==Cylinder) {
        if(dimensions[0]<=0 || dimensions[1]<=0)
            throw std::invalid_argument("WrappingCylinder requires positive radius and length");
    } else if(kind==Ellipsoid) {
        if((dimensions.array()<=0).any())
            throw std::invalid_argument("WrappingEllipsoid requires positive semiaxes");
    } else if(kind==Torus) {
        if(dimensions[1]<=0 || dimensions[0]<=dimensions[1])
            throw std::invalid_argument("WrappingTorus requires majorRadius > minorRadius > 0");
    } else throw std::invalid_argument("Unknown wrapping surface kind");
}
Object::Object(const Shape& shape_,const Eigen::Matrix4d& pose_):shape(shape_),pose(pose_)
{
    const Eigen::Matrix3d r=pose.block<3,3>(0,0);
    if(!pose.allFinite() || (r.transpose()*r-Eigen::Matrix3d::Identity()).norm()>1e-9 ||
        std::abs(r.determinant()-1.)>1e-9 ||
        (pose.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()>1e-12)
        throw std::invalid_argument("Wrapping pose must be a finite proper rigid transform");
}
Seed::Seed():start(Eigen::Vector3d::Zero()),end(Eigen::Vector3d::Zero()) {}
Contact::Contact():active(false),entry(Eigen::Vector3d::Zero()),exit(Eigen::Vector3d::Zero()),length(0) {}
Contact solveLocal(const Shape& shape,const Eigen::Vector3d& a,const Eigen::Vector3d& b,Seed* seed)
{
    // Revalidate because a caller can alter the descriptor's public fields.
    const Shape checked(shape.kind,shape.dimensions,shape.side);
    if(!a.allFinite() || !b.allFinite()) throw std::invalid_argument("Wrapping endpoints must be finite");
    Seed local=seed?*seed:Seed();
    LocalSolver solver(checked,local);
    Contact result;result.entry=a;result.exit=b;
    if(shape.kind==Shape::Cylinder) {
        local.parameters.clear();solver.solveCylinder(a,b,result.samples,result.length);
    } else if(shape.kind==Shape::Ellipsoid) solver.solveEllipsoid(a,b,result.samples,result.length);
    else solver.solveTorus(a,b,result.samples,result.length);
    if(!result.samples.empty()) {
        if(result.samples.size()<2 || !std::isfinite(result.length) || result.length<=0)
            throw std::runtime_error("Invalid wrapping contact curve");
        result.active=true;result.entry=result.samples.front();result.exit=result.samples.back();
    } else {result.length=0;local.parameters.clear();}
    if(seed) *seed=local;
    return result;
}
namespace {
Eigen::Vector3d toLocal(const Object& object,const Eigen::Vector3d& p)
{ return object.pose.block<3,3>(0,0).transpose()*(p-object.pose.block<3,1>(0,3)); }
Eigen::Vector3d toGlobal(const Object& object,const Eigen::Vector3d& p)
{ return object.pose.block<3,3>(0,0)*p+object.pose.block<3,1>(0,3); }
Contact solveObject(const Object& object,const Eigen::Vector3d& a,const Eigen::Vector3d& b,Seed& seed)
{
    Contact result=solveLocal(object.shape,toLocal(object,a),toLocal(object,b),&seed);
    result.entry=toGlobal(object,result.entry);result.exit=toGlobal(object,result.exit);
    for(auto& p:result.samples) p=toGlobal(object,p);
    return result;
}
void solveRegion(const std::vector<Object>& objects,const Region& region,
                 std::vector<Contact>& contacts,std::vector<Seed>& seeds)
{
    for(std::size_t index:region.objects) {
        contacts[index]=Contact();contacts[index].entry=region.start;contacts[index].exit=region.end;
    }
    if(region.objects.size()>1) for(std::size_t index:region.objects) {
        const Object& object=objects[index];
        Eigen::Vector3d a=region.start,b=region.end;
        if(object.shape.kind!=Shape::Cylinder) {
            a=toLocal(object,a);b=toLocal(object,b);a[2]=b[2]=0;
            if(object.shape.kind==Shape::Ellipsoid) {
                const Eigen::Vector3d& r=object.shape.dimensions;
                if(std::hypot(a[0]/r[0],a[1]/r[1])<1. || std::hypot(b[0]/r[0],b[1]/r[1])<1.) continue;
            } else {
                const double outer=object.shape.dimensions[0]+object.shape.dimensions[1];
                if(std::hypot(a[0],a[1])<outer || std::hypot(b[0],b[1])<outer) continue;
            }
            a=toGlobal(object,a);b=toGlobal(object,b);
        }
        try {
            const Contact guess=solveObject(object,a,b,seeds[index]);
            if(guess.active) contacts[index]=guess;
        } catch(const std::invalid_argument&) {} catch(const std::runtime_error&) {}
    }
    bool converged=false;
    const double tolerance=1e-10*std::max(1.,(region.end-region.start).norm());
    for(unsigned int iteration=0;iteration<200;++iteration) {
        double change=0;bool changedActive=false;
        for(std::size_t i=0;i<region.objects.size();++i) {
            Eigen::Vector3d a=region.start,b=region.end;
            for(std::size_t j=i;j>0;--j) if(contacts[region.objects[j-1]].active) {a=contacts[region.objects[j-1]].exit;break;}
            for(std::size_t j=i+1;j<region.objects.size();++j) if(contacts[region.objects[j]].active) {b=contacts[region.objects[j]].entry;break;}
            const std::size_t index=region.objects[i];const Contact& previous=contacts[index];
            const Contact current=solveObject(objects[index],a,b,seeds[index]);
            changedActive=changedActive || current.active!=previous.active;
            if(current.active) change=std::max(change,std::max((current.entry-previous.entry).norm(),(current.exit-previous.exit).norm()));
            change=std::max(change,std::abs(current.length-previous.length));
            contacts[index]=current;
        }
        if(!changedActive && change<=tolerance) {converged=true;break;}
    }
    if(!converged) throw std::runtime_error("Coupled wrapping contacts did not converge");
}
}
PathResult solvePath(const std::vector<Object>& objects,const std::vector<Region>& regions,std::vector<Seed>* seeds)
{
    std::vector<unsigned int> occurrences(objects.size(),0);
    for(const Object& object:objects) {const Object checked(object.shape,object.pose);(void)checked;}
    for(const Region& region:regions) {
        if(!region.start.allFinite() || !region.end.allFinite()) throw std::invalid_argument("Wrapping anchors must be finite");
        for(std::size_t index:region.objects) {
            if(index>=objects.size() || ++occurrences[index]!=1)
                throw std::invalid_argument("Wrapping object indices must be valid and unique across regions");
        }
    }
    for(unsigned int count:occurrences) if(count!=1)
        throw std::invalid_argument("Every wrapping object must occur once in the path regions");
    std::vector<Seed> localSeeds=seeds?*seeds:std::vector<Seed>();localSeeds.resize(objects.size());
    PathResult result;result.length=0;result.contacts.resize(objects.size());
    result.anchorGradients.assign(2*regions.size(),Eigen::Vector3d::Zero());
    result.poseGradients.assign(objects.size(),Eigen::Matrix4d::Zero());
    for(const Region& region:regions) solveRegion(objects,region,result.contacts,localSeeds);
    const auto checkLine=[&](const Eigen::Vector3d& a,const Eigen::Vector3d& b,std::size_t owner) {
        for(std::size_t j=0;j<objects.size();++j) if(j!=owner)
            if(surfaceLevelOnSegment(objects[j],a,b)<-1e-8)
                throw std::runtime_error("Wrapping trajectory intersects another object or a different via-point region; this contact configuration is unsupported");
    };
    const std::size_t noObject=objects.size();
    // Stationarity cancels sliding contact velocities. Accumulate only free-piece
    // material velocities, exactly as Geometry::computeJacobianLength does.
    for(std::size_t k=0;k<regions.size();++k) {
        const Region& region=regions[k];Eigen::Vector3d previous=region.start;
        std::size_t previousObject=noObject;
        const auto freePiece=[&](const Eigen::Vector3d& end,std::size_t nextObject) {
            checkLine(previous,end,noObject);
            const Eigen::Vector3d d=end-previous;const double length=d.norm();result.length+=length;
            if(length<=1e-14) return;
            const Eigen::Vector3d direction=d/length;
            if(previousObject==noObject) result.anchorGradients[2*k]-=direction;
            else {
                result.poseGradients[previousObject].block<3,3>(0,0)-=direction*toLocal(objects[previousObject],previous).transpose();
                result.poseGradients[previousObject].block<3,1>(0,3)-=direction;
            }
            if(nextObject==noObject) result.anchorGradients[2*k+1]+=direction;
            else {
                result.poseGradients[nextObject].block<3,3>(0,0)+=direction*toLocal(objects[nextObject],end).transpose();
                result.poseGradients[nextObject].block<3,1>(0,3)+=direction;
            }
        };
        for(std::size_t index:region.objects) {
            const Contact& c=result.contacts[index];if(!c.active) continue;
            freePiece(c.entry,index);result.length+=c.length;
            for(std::size_t i=1;i<c.samples.size();++i) checkLine(c.samples[i-1],c.samples[i],index);
            previous=c.exit;previousObject=index;
        }
        freePiece(region.end,noObject);
    }
    if(seeds) *seeds=localSeeds;
    return result;
}
} } // biorbd::wrapping
