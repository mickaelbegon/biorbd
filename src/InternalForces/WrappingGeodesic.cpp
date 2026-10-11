#include "WrappingGeodesic.h"
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <sstream>
using namespace biorbd::wrapping;
namespace {
typedef Eigen::Matrix<double,6,1> State;
typedef Eigen::Matrix<double,4,1> Residual;
const double pi=std::acos(-1.);
double polynomial(const std::vector<double>& c,double x) {
    double result=0;for(auto i=c.rbegin();i!=c.rend();++i) result=result*x+*i;return result;
}
std::vector<double> roots(std::vector<double> c) {
    while(c.size()>1 && c.back()==0) c.pop_back();
    if(c.size()<2) return {};
    std::vector<double> derivative;for(size_t i=1;i<c.size();++i) derivative.push_back(i*c[i]);
    auto cuts=roots(derivative);cuts.push_back(0);cuts.push_back(1);std::sort(cuts.begin(),cuts.end());
    std::vector<double> out;
    for(size_t i=1;i<cuts.size();++i) {
        double lo=cuts[i-1],hi=cuts[i],flo=polynomial(c,lo),fhi=polynomial(c,hi);
        if(flo==0) out.push_back(lo);if(fhi==0) out.push_back(hi);
        if((flo<0 && fhi>0)||(flo>0 && fhi<0)) {
            for(int j=0;j<64;++j) {double mid=(lo+hi)/2,fm=polynomial(c,mid);
                if((flo<0 && fm<=0)||(flo>0 && fm>=0)) {lo=mid;flo=fm;} else hi=mid;}
            out.push_back((lo+hi)/2);
        }
    }
    return out;
}
State rhs(const Surface& s,const State& x) {
    double f;Eigen::Vector3d g;Eigen::Matrix3d h;s.geometry(x.head<3>(),f,g,h);
    State out;out.head<3>()=x.tail<3>();out.tail<3>()=-x.tail<3>().dot(h*x.tail<3>())/g.squaredNorm()*g;
    return out;
}
State initial(const Surface& s,const Eigen::Vector3d& a,const Eigen::Vector3d& u,double& entryResidual) {
    State x;x.head<3>()=s.point(u.head<2>());
    double f;Eigen::Vector3d g;Eigen::Matrix3d h;s.geometry(x.head<3>(),f,g,h);
    Eigen::Vector3d incoming=x.head<3>()-a;
    if(incoming.norm()<1e-10) throw std::runtime_error("Geodesic shooting does not support an endpoint coinciding with its contact");
    incoming.normalize();entryResidual=incoming.dot(g)/g.norm();
    Eigen::Vector3d tangent=incoming-incoming.dot(g)/g.squaredNorm()*g;
    if(tangent.norm()<1e-8) throw std::runtime_error("Degenerate geodesic tangent initialization");
    x.tail<3>()=tangent.normalized();return x;
}
State integrate(const Surface& s,const State& start,double length,int steps,std::vector<State>* samples=nullptr) {
    if(!std::isfinite(length)||length<=1e-7||length>20*pi) throw std::runtime_error("Geodesic arc length is outside the bounded shooting range");
    State x=start;const double dt=length/steps;
    if(samples) {samples->clear();samples->push_back(x);}
    for(int i=0;i<steps;++i) {
        State k1=rhs(s,x),k2=rhs(s,x+dt*k1/2),k3=rhs(s,x+dt*k2/2),k4=rhs(s,x+dt*k3);
        x+=dt*(k1+2*k2+2*k3+k4)/6;
        if(!x.allFinite()||x.norm()>1e5) throw std::runtime_error("Geodesic integration diverged");
        if(samples) samples->push_back(x);
    }
    return x;
}
Residual residual(const Surface& s,const Eigen::Vector3d& a,const Eigen::Vector3d& b,const Eigen::Vector3d& u,int steps) {
    double entry;State start=initial(s,a,u,entry);
    State end=integrate(s,start,std::exp(u[2]),steps);
    Eigen::Vector3d outgoing=b-end.head<3>();
    if(outgoing.norm()<1e-10) throw std::runtime_error("Geodesic shooting does not support an endpoint coinciding with its contact");
    Residual r;r[0]=entry;r.tail<3>()=end.tail<3>()-outgoing.normalized();return r;
}
Eigen::Vector3d fit(const Surface& s,const Eigen::Vector3d& a,const Eigen::Vector3d& b,Eigen::Vector3d u,int steps,double tolerance) {
    double damping=1e-4;
    Residual r=residual(s,a,b,u,steps);
    for(int iteration=0;iteration<70;++iteration) {
        if(r.cwiseAbs().maxCoeff()<tolerance) return u;
        Eigen::Matrix<double,4,3> j;
        for(int k=0;k<3;++k) {
            Eigen::Vector3d plus=u,minus=u;const double delta=2e-5;
            plus[k]+=delta;minus[k]-=delta;
            j.col(k)=(residual(s,a,b,plus,steps)-residual(s,a,b,minus,steps))/(2*delta);
        }
        const Eigen::Matrix3d normal=j.transpose()*j;
        bool improved=false;
        for(int trial=0;trial<12;++trial) {
            Eigen::Matrix3d regularized=normal;
            for(int k=0;k<3;++k) regularized(k,k)+=damping*std::max(1e-6,normal(k,k));
            Eigen::Vector3d step=regularized.ldlt().solve(-j.transpose()*r);
            const double norm=step.cwiseAbs().maxCoeff();if(norm>.45) step*=.45/norm;
            Eigen::Vector3d candidate=u+step;
            if(s.kind==Surface::Ellipsoid) candidate[1]=std::max(-pi/2+1e-6,std::min(pi/2-1e-6,candidate[1]));
            bool valid=true;Residual proposed;
            try {proposed=residual(s,a,b,candidate,steps);} catch(const std::runtime_error&) {valid=false;}
            if(valid && proposed.squaredNorm()<r.squaredNorm()) {
                u=candidate;r=proposed;damping=std::max(1e-12,damping*.2);improved=true;break;
            }
            damping*=10;
        }
        if(!improved) break;
    }
    std::ostringstream message;message<<"Geodesic shooting did not converge to tangent contact directions (bounded LM): kind="<<s.kind<<", RK4 steps="<<steps<<", residual="<<r.cwiseAbs().maxCoeff()<<", tolerance="<<tolerance;
    throw std::runtime_error(message.str());
}
}
Surface::Surface(Kind type,const Eigen::Vector3d& d):kind(type),dimensions(d),scale(d.maxCoeff()) {dimensions/=scale;}
Eigen::Vector3d Surface::point(const Eigen::Vector2d& uv) const {
    const double u=uv[0],v=uv[1];
    if(kind==Ellipsoid) return dimensions.cwiseProduct(Eigen::Vector3d(std::cos(u)*std::cos(v),std::sin(u)*std::cos(v),std::sin(v)));
    return Eigen::Vector3d((dimensions[0]+dimensions[1]*std::cos(v))*std::cos(u),
        (dimensions[0]+dimensions[1]*std::cos(v))*std::sin(u),dimensions[1]*std::sin(v));
}
Eigen::Vector2d Surface::angles(const Eigen::Vector3d& x) const {
    if(kind==Ellipsoid) {Eigen::Vector3d p=x.cwiseQuotient(dimensions);
        return Eigen::Vector2d(std::atan2(p[1],p[0]),std::asin(std::max(-1.,std::min(1.,p[2]))));}
    return Eigen::Vector2d(std::atan2(x[1],x[0]),std::atan2(x[2],std::hypot(x[0],x[1])-dimensions[0]));
}
void Surface::geometry(const Eigen::Vector3d& x,double& f,Eigen::Vector3d& g,Eigen::Matrix3d& h) const {
    if(kind==Ellipsoid) {
        Eigen::Vector3d inverse=dimensions.cwiseProduct(dimensions).cwiseInverse();
        f=x.cwiseProduct(x).dot(inverse)-1;g=2*x.cwiseProduct(inverse);h=(2*inverse).asDiagonal();return;
    }
    const double rho=std::hypot(x[0],x[1]),major=dimensions[0],minor=dimensions[1],q=rho-major;
    if(rho<1e-10) throw std::runtime_error("Torus geodesic reached its singular symmetry axis");
    f=q*q+x[2]*x[2]-minor*minor;
    g=Eigen::Vector3d(2*q*x[0]/rho,2*q*x[1]/rho,2*x[2]);h.setZero();
    h.block<2,2>(0,0)=2*(q/rho*Eigen::Matrix2d::Identity()+major/(rho*rho*rho)*x.head<2>()*x.head<2>().transpose());h(2,2)=2;
    if(g.squaredNorm()<1e-18) throw std::runtime_error("Torus geodesic reached a degenerate normal");
}
double Surface::chordLevel(const Eigen::Vector3d& a,const Eigen::Vector3d& b) const {
    Eigen::Vector3d d=b-a;
    if(kind==Ellipsoid) {
        Eigen::Vector3d p=a.cwiseQuotient(dimensions),v=d.cwiseQuotient(dimensions);
        double t=v.squaredNorm()?std::max(0.,std::min(1.,-p.dot(v)/v.squaredNorm())):0;
        return (p+t*v).squaredNorm()-1;
    }
    const double major=dimensions[0],minor=dimensions[1],q0=a.squaredNorm()+major*major-minor*minor,q1=2*a.dot(d),q2=d.squaredNorm();
    std::vector<double> c={q0*q0-4*major*major*a.head<2>().squaredNorm(),
        2*q0*q1-8*major*major*a.head<2>().dot(d.head<2>()),q1*q1+2*q0*q2-4*major*major*d.head<2>().squaredNorm(),2*q1*q2,q2*q2};
    std::vector<double> derivative;for(size_t i=1;i<c.size();++i) derivative.push_back(i*c[i]);
    auto ts=roots(derivative);ts.push_back(0);ts.push_back(1);double minimum=polynomial(c,0);
    for(double t:ts) minimum=std::min(minimum,polynomial(c,t));
    return minimum/std::pow(major+minor,4);
}
Result biorbd::wrapping::shoot(const Surface& s,const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,const Eigen::Vector3d& seed,bool finalSolve) {
    Eigen::Vector3d u=fit(s,a,b,seed,128,2e-8);
    if(!finalSolve) {Result result;result.parameters=u;result.length=std::exp(u[2]);return result;}
    Eigen::Vector3d previous=fit(s,a,b,u,512,2e-11);
    u=fit(s,a,b,previous,1024,2e-11);
    double entry;State start=initial(s,a,u,entry);std::vector<State> states;
    State end=integrate(s,start,std::exp(u[2]),1024,&states);
    double unused;State previousEnd=integrate(s,initial(s,a,previous,unused),std::exp(previous[2]),512);
    if((end-previousEnd).norm()>2e-9 || (start.head<3>()-s.point(previous.head<2>())).norm()>2e-9 ||
        std::fabs(std::exp(u[2])-std::exp(previous[2]))>2e-9)
        throw std::runtime_error("Geodesic integration refinement did not converge");
    const Residual r=residual(s,a,b,u,1024);
    if(r.cwiseAbs().maxCoeff()>2e-9) throw std::runtime_error("Geodesic tangent-direction residual exceeds tolerance");
    for(const State& state:states) {
        double f;Eigen::Vector3d g;Eigen::Matrix3d h;s.geometry(state.head<3>(),f,g,h);
        if(std::fabs(f)>2e-9 || std::fabs(state.tail<3>().norm()-1)>2e-9 ||
            std::fabs(state.tail<3>().dot(g))/g.norm()>2e-9)
            throw std::runtime_error("Geodesic surface or unit-speed defect exceeds tolerance");
        if(state.tail<3>().dot(h*state.tail<3>())/g.norm() < -2e-9)
            throw std::runtime_error("Geodesic requires a negative normal contact reaction");
    }
    if(s.chordLevel(a,start.head<3>()) < -2e-9 || s.chordLevel(end.head<3>(),b) < -2e-9)
        throw std::runtime_error("Geodesic free segment penetrates the wrapping solid; disconnected contact is unsupported");
    Result result;result.parameters=u;result.length=std::exp(u[2]);
    for(const State& state:states) result.points.push_back(state.head<3>());
    return result;
}
