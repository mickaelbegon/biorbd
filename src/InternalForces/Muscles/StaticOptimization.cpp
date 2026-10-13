#define BIORBD_API_EXPORTS

#include "InternalForces/Muscles/StaticOptimization.h"

#include <IpIpoptApplication.hpp>
#include <cmath>
#include <string>

#include "BiorbdModel.h"
#include "InternalForces/Muscles/StateDynamics.h"
#include "InternalForces/Muscles/StaticOptimizationIpoptLinearized.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "Utils/Error.h"
#include "Utils/String.h"
#include "Utils/Vector.h"

using namespace BIORBD_NAMESPACE;
using namespace internal_forces;

internal_forces::muscles::StaticOptimization::StaticOptimization(
    Model& model,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    const rigidbody::GeneralizedTorque& torqueTarget,
    double initialActivationGuess,
    unsigned int pNormFactor,
    bool useResidualTorque,
    int verbose)
    : m_model(model),
      m_useResidualTorque(useResidualTorque),
      m_initialActivationGuess(
          std::make_shared<utils::Vector>(m_model.nbMuscles())),
      m_pNormFactor(pNormFactor),
      m_verbose(verbose),
      m_staticOptimProblem(),
      m_alreadyRun(false) {
  m_allQ.push_back(Q);
  m_allQdot.push_back(Qdot);
  m_allTorqueTarget.push_back(torqueTarget);
  for (unsigned int i = 0; i < m_model.nbMuscles(); ++i) {
    (*m_initialActivationGuess)[i] = initialActivationGuess;
  }
}

internal_forces::muscles::StaticOptimization::StaticOptimization(
    Model& model,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    const rigidbody::GeneralizedTorque& torqueTarget,
    const utils::Vector& initialActivationGuess,
    unsigned int pNormFactor,
    bool useResidualTorque,
    int verbose)
    : m_model(model),
      m_useResidualTorque(useResidualTorque),
      m_initialActivationGuess(
          std::make_shared<utils::Vector>(m_model.nbMuscles())),
      m_pNormFactor(pNormFactor),
      m_verbose(verbose),
      m_staticOptimProblem(),
      m_alreadyRun(false) {
  m_allQ.push_back(Q);
  m_allQdot.push_back(Qdot);
  m_allTorqueTarget.push_back(torqueTarget);

  if (initialActivationGuess.size() != m_model.nbMuscles()) {
    utils::Error::raise(
        "Initial guess must either be a single value or a vector "
        "of dimension nbMuscles");
  }
  *m_initialActivationGuess = initialActivationGuess;
}

internal_forces::muscles::StaticOptimization::StaticOptimization(
    Model& model,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    const rigidbody::GeneralizedTorque& torqueTarget,
    const std::vector<internal_forces::muscles::StateDynamics>&
        initialActivationGuess,
    unsigned int pNormFactor,
    bool useResidualTorque,
    int verbose)
    : m_model(model),
      m_useResidualTorque(useResidualTorque),
      m_initialActivationGuess(
          std::make_shared<utils::Vector>(m_model.nbMuscles())),
      m_pNormFactor(pNormFactor),
      m_verbose(verbose),
      m_alreadyRun(false) {
  m_allQ.push_back(Q);
  m_allQdot.push_back(Qdot);
  m_allTorqueTarget.push_back(torqueTarget);

  if (initialActivationGuess.size() != m_model.nbMuscles()) {
    utils::Error::raise(
        "Initial guess must either be a single value or a vector "
        "of dimension nbMuscles");
  }

  for (unsigned int i = 0; i < m_model.nbMuscles(); ++i) {
    (*m_initialActivationGuess)[i] = initialActivationGuess[i].activation();
  }
}

internal_forces::muscles::StaticOptimization::StaticOptimization(
    Model& model,
    const std::vector<rigidbody::GeneralizedCoordinates>& allQ,
    const std::vector<rigidbody::GeneralizedVelocity>& allQdot,
    const std::vector<rigidbody::GeneralizedTorque>& allTorqueTarget,
    double initialActivationGuess,
    unsigned int pNormFactor,
    bool useResidualTorque,
    int verbose)
    : m_model(model),
      m_useResidualTorque(useResidualTorque),
      m_allQ(allQ),
      m_allQdot(allQdot),
      m_allTorqueTarget(allTorqueTarget),
      m_initialActivationGuess(
          std::make_shared<utils::Vector>(m_model.nbMuscles())),
      m_pNormFactor(pNormFactor),
      m_verbose(verbose),
      m_alreadyRun(false) {
  for (unsigned int i = 0; i < m_model.nbMuscles(); ++i) {
    (*m_initialActivationGuess)[i] = initialActivationGuess;
  }
}

internal_forces::muscles::StaticOptimization::StaticOptimization(
    Model& model,
    const std::vector<rigidbody::GeneralizedCoordinates>& allQ,
    const std::vector<rigidbody::GeneralizedVelocity>& allQdot,
    const std::vector<rigidbody::GeneralizedTorque>& allTorqueTarget,
    const utils::Vector& initialActivationGuess,
    unsigned int pNormFactor,
    bool useResidualTorque,
    int verbose)
    : m_model(model),
      m_useResidualTorque(useResidualTorque),
      m_allQ(allQ),
      m_allQdot(allQdot),
      m_allTorqueTarget(allTorqueTarget),
      m_initialActivationGuess(
          std::make_shared<utils::Vector>(m_model.nbMuscles())),
      m_pNormFactor(pNormFactor),
      m_verbose(verbose),
      m_alreadyRun(false) {
  if (initialActivationGuess.size() != m_model.nbMuscles()) {
    utils::Error::raise(
        "Initial guess must either be a single value or a vector "
        "of dimension nbMuscles");
  }
  *m_initialActivationGuess = initialActivationGuess;
}

internal_forces::muscles::StaticOptimization::StaticOptimization(
    Model& model,
    const std::vector<rigidbody::GeneralizedCoordinates>& allQ,
    const std::vector<rigidbody::GeneralizedVelocity>& allQdot,
    const std::vector<rigidbody::GeneralizedTorque>& allTorqueTarget,
    const std::vector<internal_forces::muscles::StateDynamics>&
        initialActivationGuess,
    unsigned int pNormFactor,
    bool useResidualTorque,
    int verbose)
    : m_model(model),
      m_useResidualTorque(useResidualTorque),
      m_allQ(allQ),
      m_allQdot(allQdot),
      m_allTorqueTarget(allTorqueTarget),
      m_initialActivationGuess(
          std::make_shared<utils::Vector>(m_model.nbMuscles())),
      m_pNormFactor(pNormFactor),
      m_verbose(verbose),
      m_alreadyRun(false) {
  if (initialActivationGuess.size() != m_model.nbMuscles()) {
    utils::Error::raise(
        "Initial guess must either be a single value or a vector "
        "of dimension nbMuscles");
  }

  for (unsigned int i = 0; i < m_model.nbMuscles(); ++i) {
    (*m_initialActivationGuess)[i] = initialActivationGuess[i].activation();
  }
}

void internal_forces::muscles::StaticOptimization::run(
    bool useLinearizedState) {
  m_alreadyRun = false;
  m_staticOptimProblem.clear();
  m_solverStatuses.clear();
  m_finalTorqueErrors.clear();
  utils::Error::check(
      !m_allQ.empty() && m_allQ.size() == m_allQdot.size() &&
          m_allQ.size() == m_allTorqueTarget.size(),
      "Static optimization frame counts must match and be nonempty");
  // Setup the Ipopt problem
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  app->Options()->SetNumericValue("tol", 1e-7);
  app->Options()->SetStringValue("mu_strategy", "adaptive");
  // app->Options()->SetStringValue("output_file", "ipopt.out");
  app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  app->Options()->SetStringValue("derivative_test", "first-order");
  app->Options()->SetIntegerValue("max_iter", 10000);
  app->Options()->SetIntegerValue("print_level", m_verbose > 0 ? 5 : 0);

  Ipopt::ApplicationReturnStatus status;
  status = app->Initialize();
  utils::Error::check(
      status == Ipopt::Solve_Succeeded, "Ipopt initialization failed");

  for (unsigned int i = 0; i < m_allQ.size(); ++i) {
    if (useLinearizedState)
      m_staticOptimProblem.push_back(
          new internal_forces::muscles::StaticOptimizationIpoptLinearized(
              m_model,
              m_allQ[i],
              m_allQdot[i],
              m_allTorqueTarget[i],
              *m_initialActivationGuess,
              m_useResidualTorque,
              m_pNormFactor,
              m_verbose));
    else
      m_staticOptimProblem.push_back(
          new internal_forces::muscles::StaticOptimizationIpopt(
              m_model,
              m_allQ[i],
              m_allQdot[i],
              m_allTorqueTarget[i],
              *m_initialActivationGuess,
              m_useResidualTorque,
              m_pNormFactor,
              m_verbose));
    // Optimize!
    status = app->OptimizeTNLP(m_staticOptimProblem[i]);
    m_solverStatuses.push_back(static_cast<int>(status));
    utils::Error::check(
        status == Ipopt::Solve_Succeeded ||
            status == Ipopt::Solved_To_Acceptable_Level,
        "Static optimization failed at frame " + std::to_string(i) +
            " (IPOPT status " + std::to_string(static_cast<int>(status)) + ")");

    auto* problem =
        static_cast<internal_forces::muscles::StaticOptimizationIpopt*>(
            Ipopt::GetRawPtr(m_staticOptimProblem[i]));
    const utils::Vector activation = problem->finalSolution();
    std::vector<std::shared_ptr<internal_forces::muscles::State>> states;
    for (unsigned int j = 0; j < m_model.nbMuscles(); ++j) {
      states.push_back(
          std::make_shared<internal_forces::muscles::State>(0, activation[j]));
    }
    utils::Vector error =
        m_model.muscularJointTorque(states, m_allQ[i], m_allQdot[i]) +
        problem->finalResidual() - m_allTorqueTarget[i];
    for (unsigned int j = 0; j < error.size(); ++j) {
      const double tolerance = 1e-6 * (1 + std::fabs(m_allTorqueTarget[i][j]));
      utils::Error::check(
          std::isfinite(error[j]) && std::fabs(error[j]) <= tolerance,
          "Static optimization physical torque balance failed at frame " +
              std::to_string(i));
    }
    m_finalTorqueErrors.push_back(error);

    // Take the solution of the previous optimization as the solution for the
    // next optimization
    *m_initialActivationGuess =
        static_cast<internal_forces::muscles::StaticOptimizationIpopt*>(
            Ipopt::GetRawPtr(m_staticOptimProblem[i]))
            ->finalSolution();
  }
  m_alreadyRun = true;
}

std::vector<utils::Vector>
internal_forces::muscles::StaticOptimization::finalSolution() {
  std::vector<utils::Vector> res;
  if (!m_alreadyRun) {
    utils::Error::raise(
        "Problem has not been ran through the optimization process "
        "yet, you should optimize it first to get "
        "the optimized solution");
  } else {
    for (unsigned int i = 0; i < m_allQ.size(); ++i) {
      res.push_back(
          static_cast<internal_forces::muscles::StaticOptimizationIpopt*>(
              Ipopt::GetRawPtr(m_staticOptimProblem[i]))
              ->finalSolution());
    }
  }

  return res;
}

utils::Vector internal_forces::muscles::StaticOptimization::finalSolution(
    unsigned int index) {
  utils::Vector res;
  if (!m_alreadyRun) {
    utils::Error::raise(
        "Problem has not been ran through the optimization process "
        "yet, you should optimize it first to get "
        "the optimized solution");
  } else {
    res = static_cast<internal_forces::muscles::StaticOptimizationIpopt*>(
              Ipopt::GetRawPtr(m_staticOptimProblem[index]))
              ->finalSolution();
  }
  return res;
}

std::vector<int> internal_forces::muscles::StaticOptimization::solverStatuses()
    const {
  return m_solverStatuses;
}

utils::Vector internal_forces::muscles::StaticOptimization::finalResidual(
    unsigned int index) const {
  utils::Error::check(
      m_alreadyRun, "Static optimization has no validated final solution");
  return static_cast<internal_forces::muscles::StaticOptimizationIpopt*>(
             Ipopt::GetRawPtr(m_staticOptimProblem.at(index)))
      ->finalResidual();
}

utils::Vector internal_forces::muscles::StaticOptimization::finalTorqueError(
    unsigned int index) const {
  utils::Error::check(
      m_alreadyRun, "Static optimization has no validated final solution");
  return m_finalTorqueErrors.at(index);
}
