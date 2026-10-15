#define BIORBD_API_EXPORTS
#include "WrappingCasadi.h"
#include <algorithm>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>

using namespace BIORBD_NAMESPACE;
namespace {
namespace kernel = ::biorbd::wrapping;

struct Evaluation {
    kernel::PathResult path;
    std::vector<Eigen::Vector3d> anchors;
    Eigen::MatrixXd kinematicJacobian;
};

// An immutable snapshot, never a pointer to the caller's mutable Model. The
// solver state is local to each evaluation, including all continuation seeds.
struct Descriptor {
    casadi::Function kinematics;
    std::vector<kernel::Shape> shapes;
    std::vector<std::vector<size_t>> regions;
    size_t anchors;
    casadi_int nq, nodes;

    Evaluation evaluate(const double* q) const {
        const std::vector<casadi::DM> values = kinematics(
            std::vector<casadi::DM>{casadi::DM(nq ? std::vector<double>(q,q+nq) : std::vector<double>())});
        const std::vector<double>& coordinates=values[0].nonzeros();
        Evaluation result;
        result.anchors.resize(anchors);
        for(size_t i=0;i<anchors;++i)
            result.anchors[i]=Eigen::Map<const Eigen::Vector3d>(&coordinates[3*i]);
        std::vector<kernel::Object> objects;
        objects.reserve(shapes.size());
        for(size_t i=0;i<shapes.size();++i) {
            objects.emplace_back(shapes[i],Eigen::Map<const Eigen::Matrix4d>(
                &coordinates[3*anchors+16*i]));
        }
        std::vector<kernel::Region> parts(regions.size());
        for(size_t i=0;i<parts.size();++i) {
            parts[i].start=result.anchors[i];parts[i].end=result.anchors[i+1];
            parts[i].objects=regions[i];
        }
        result.path=kernel::solvePath(objects,parts);
        const std::vector<double>& jac=values[1].nonzeros();
        result.kinematicJacobian=Eigen::Map<const Eigen::MatrixXd>(
            jac.data(),coordinates.size(),nq);
        return result;
    }

    Eigen::RowVectorXd gradient(const Evaluation& result) const {
        Eigen::VectorXd material=Eigen::VectorXd::Zero(3*anchors+16*shapes.size());
        for(size_t i=0;i<regions.size();++i) {
            material.segment<3>(3*i)+=result.path.anchorGradients[2*i];
            material.segment<3>(3*(i+1))+=result.path.anchorGradients[2*i+1];
        }
        for(size_t i=0;i<shapes.size();++i)
            material.segment(3*anchors+16*i,16)=Eigen::Map<const Eigen::VectorXd>(
                result.path.poseGradients[i].data(),16);
        return material.transpose()*result.kinematicJacobian;
    }

    std::vector<double> points(const Evaluation& result) const {
        std::vector<double> values;
        values.reserve(3*nodes);
        const auto append=[&](const Eigen::Vector3d& point) {
            for(int j=0;j<3;++j) values.push_back(point[j]);
        };
        append(result.anchors.front());
        for(size_t i=0;i<regions.size();++i) {
            Eigen::Vector3d previous=result.anchors[i];
            for(size_t index : regions[i]) {
                const kernel::Contact& contact=result.path.contacts[index];
                append(contact.active ? contact.entry : previous);
                append(contact.active ? contact.exit : previous);
                if(contact.active) previous=contact.exit;
            }
            append(result.anchors[i+1]);
        }
        return values;
    }
};

casadi::Dict finiteDifferenceOptions() {
    casadi::Dict options;
    options["enable_fd"]=true;
    options["fd_method"]="central";
    // Physical Q perturbations retain valid rotations. Higher derivatives
    // are numerical approximations of the stationary first-order gradient.
    casadi::Dict fd;
    fd["h"]=1e-6;
    fd["h_iter"]=0;
    options["fd_options"]=fd;
    return options;
}

class GradientCallback : public casadi::Callback {
    std::shared_ptr<const Descriptor> m_descriptor;
public:
    GradientCallback(const std::string& name,std::shared_ptr<const Descriptor> descriptor) :
        m_descriptor(std::move(descriptor)) { construct(name,finiteDifferenceOptions()); }
    casadi_int get_n_in() override { return 1; }
    casadi_int get_n_out() override { return 1; }
    casadi::Sparsity get_sparsity_in(casadi_int) override {
        return casadi::Sparsity::dense(m_descriptor->nq,1);
    }
    casadi::Sparsity get_sparsity_out(casadi_int) override {
        return casadi::Sparsity::dense(1,m_descriptor->nq);
    }
    bool has_eval_buffer() const override { return true; }
    int eval_buffer(const double** arg,const std::vector<casadi_int>&,
        double** res,const std::vector<casadi_int>&) const override {
        if(res[0]) {
            const Eigen::RowVectorXd gradient=m_descriptor->gradient(m_descriptor->evaluate(arg[0]));
            std::copy(gradient.data(),gradient.data()+gradient.size(),res[0]);
        }
        return 0;
    }
};

class LengthCallback : public casadi::Callback {
    std::shared_ptr<const Descriptor> m_descriptor;
    std::shared_ptr<GradientCallback> m_gradient;
public:
    LengthCallback(const std::string& name,std::shared_ptr<const Descriptor> descriptor,
        std::shared_ptr<GradientCallback> gradient) : m_descriptor(std::move(descriptor)),
        m_gradient(std::move(gradient)) { construct(name); }
    casadi_int get_n_in() override { return 1; }
    casadi_int get_n_out() override { return 1; }
    casadi::Sparsity get_sparsity_in(casadi_int) override {
        return casadi::Sparsity::dense(m_descriptor->nq,1);
    }
    casadi::Sparsity get_sparsity_out(casadi_int) override {
        return casadi::Sparsity::dense(1,1);
    }
    bool has_eval_buffer() const override { return true; }
    int eval_buffer(const double** arg,const std::vector<casadi_int>&,
        double** res,const std::vector<casadi_int>&) const override {
        if(res[0]) res[0][0]=m_descriptor->evaluate(arg[0]).path.length;
        return 0;
    }
    bool has_jacobian() const override { return true; }
    casadi::Function get_jacobian(const std::string& name,
        const std::vector<std::string>& inputs,const std::vector<std::string>& outputs,
        const casadi::Dict& options) const override {
        casadi::MX q=casadi::MX::sym("q",m_descriptor->nq);
        casadi::MX length=casadi::MX::sym("length");
        return casadi::Function(name,{q,length},(*m_gradient)(std::vector<casadi::MX>{q}),
            inputs,outputs,options);
    }
};

class PointsCallback : public casadi::Callback {
    std::shared_ptr<const Descriptor> m_descriptor;
public:
    PointsCallback(const std::string& name,std::shared_ptr<const Descriptor> descriptor) :
        m_descriptor(std::move(descriptor)) { construct(name,finiteDifferenceOptions()); }
    casadi_int get_n_in() override { return 1; }
    casadi_int get_n_out() override { return 1; }
    casadi::Sparsity get_sparsity_in(casadi_int) override {
        return casadi::Sparsity::dense(m_descriptor->nq,1);
    }
    casadi::Sparsity get_sparsity_out(casadi_int) override {
        return casadi::Sparsity::dense(3*m_descriptor->nodes,1);
    }
    bool has_eval_buffer() const override { return true; }
    int eval_buffer(const double** arg,const std::vector<casadi_int>&,
        double** res,const std::vector<casadi_int>&) const override {
        if(res[0]) {
            const std::vector<double> points=m_descriptor->points(m_descriptor->evaluate(arg[0]));
            std::copy(points.begin(),points.end(),res[0]);
        }
        return 0;
    }
};

struct Graph {
    std::shared_ptr<GradientCallback> gradient;
    std::shared_ptr<LengthCallback> length;
    std::shared_ptr<PointsCallback> points;
};

// CallbackInternal does not own its C++ Callback. Retain one immutable graph
// per distinct model/path definition for process lifetime, so exported
// Functions survive Model destruction. Numerical evaluations add no entries.
std::mutex graphMutex;
std::map<std::string,std::shared_ptr<Graph>> graphs;
}

internal_forces::CasadiWrappingExpressions internal_forces::makeCasadiWrapping(
    const casadi::Function& kinematics,const casadi::MX& q,
    const std::vector<kernel::Shape>& shapes,
    const std::vector<std::vector<size_t>>& regions,size_t anchorCount)
{
    std::ostringstream key;
    key << kinematics.serialize() << ' ' << anchorCount << std::setprecision(17);
    for(const kernel::Shape& shape : shapes)
        key << ' ' << static_cast<int>(shape.kind) << ' ' << shape.side << ' '
            << shape.dimensions.transpose();
    for(const auto& region : regions) {
        key << " region";
        for(size_t index : region) key << ' ' << index;
    }
    std::shared_ptr<Graph> graph;
    {
        std::lock_guard<std::mutex> lock(graphMutex);
        auto existing=graphs.find(key.str());
        if(existing!=graphs.end()) graph=existing->second;
        else {
            auto descriptor=std::make_shared<Descriptor>();
            descriptor->kinematics=kinematics;descriptor->shapes=shapes;
            descriptor->regions=regions;descriptor->anchors=anchorCount;
            descriptor->nq=kinematics.size1_in(0);
            descriptor->nodes=anchorCount+2*shapes.size();
            const std::string suffix=std::to_string(graphs.size());
            graph=std::make_shared<Graph>();
            graph->gradient=std::make_shared<GradientCallback>("wrapping_gradient_"+suffix,descriptor);
            graph->length=std::make_shared<LengthCallback>("wrapping_length_"+suffix,descriptor,graph->gradient);
            graph->points=std::make_shared<PointsCallback>("wrapping_points_"+suffix,descriptor);
            graphs.emplace(key.str(),graph);
        }
    }
    CasadiWrappingExpressions result;
    result.length=(*graph->length)(std::vector<casadi::MX>{q})[0];
    result.gradient=(*graph->gradient)(std::vector<casadi::MX>{q})[0];
    result.points=(*graph->points)(std::vector<casadi::MX>{q})[0];
    return result;
}
