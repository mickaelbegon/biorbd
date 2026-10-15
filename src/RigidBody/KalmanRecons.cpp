#define BIORBD_API_EXPORTS
#include "RigidBody/KalmanRecons.h"

#include "BiorbdModel.h"
#include "RigidBody/GeneralizedAcceleration.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "Utils/Matrix.h"
#include "Utils/Vector.h"

using namespace BIORBD_NAMESPACE;

rigidbody::KalmanRecons::KalmanRecons()
    : m_params(std::make_shared<KalmanParam>()),
      m_Te(std::make_shared<double>(1.0 / (m_params->acquisitionFrequency()))),
      m_nbDof(std::make_shared<size_t>()),
      m_nMeasure(std::make_shared<size_t>()),
      m_xp(std::make_shared<utils::Vector>()),
      m_A(std::make_shared<utils::Matrix>()),
      m_Q(std::make_shared<utils::Matrix>()),
      m_R(std::make_shared<utils::Matrix>()),
      m_Pp(std::make_shared<utils::Matrix>()) {}

rigidbody::KalmanRecons::KalmanRecons(
    Model &model,
    size_t nbMeasure,
    KalmanParam params)
    : m_params(std::make_shared<KalmanParam>(params)),
      m_Te(std::make_shared<double>(1.0 / (m_params->acquisitionFrequency()))),
      m_nbDof(std::make_shared<size_t>(model.dof_count)),
      m_nMeasure(std::make_shared<size_t>(nbMeasure)),
      m_xp(std::make_shared<utils::Vector>()),
      m_A(std::make_shared<utils::Matrix>()),
      m_Q(std::make_shared<utils::Matrix>()),
      m_R(std::make_shared<utils::Matrix>()),
      m_Pp(std::make_shared<utils::Matrix>()) {}

rigidbody::KalmanRecons::~KalmanRecons() {}

void rigidbody::KalmanRecons::DeepCopy(const rigidbody::KalmanRecons &other) {
  *m_params = *other.m_params;
  *m_Te = *other.m_Te;
  *m_nbDof = *other.m_nbDof;
  *m_nMeasure = *other.m_nMeasure;
  *m_xp = *other.m_xp;
  *m_A = *other.m_A;
  *m_Q = *other.m_Q;
  *m_R = *other.m_R;
  *m_Pp = *other.m_Pp;
}

void rigidbody::KalmanRecons::iteration(
    utils::Vector measure,
    const utils::Vector &projectedMeasure,
    const utils::Matrix &Hessian,
    const std::vector<size_t> &occlusion) {
  // The measurement matrix has the structure H = [J 0 0] (only Q is measured)
  // and R is diagonal. Therefore, the m x m innovation matrix never needs to
  // be inverted: with the Woodbury identity, the gain only requires the
  // solution of a (nQ x nQ) system (m, the number of measures, is usually
  // larger than nQ, the number of degrees-of-freedom).
  const Eigen::Index nQ = static_cast<Eigen::Index>(*m_nbDof);

  // Prediction
  const utils::Vector &xkm(*m_A * *m_xp);
  const Eigen::MatrixXd Pkm(*m_A * *m_Pp * m_A->transpose() + *m_Q);

  // Occluded measures do not contribute (their rows of H are null)
  manageOcclusionDuringIteration(measure, occlusion);

  // Correction
  const Eigen::MatrixXd J(Hessian.leftCols(nQ));  // m x nQ
  const Eigen::VectorXd Rdiag(m_R->diagonal());
  const Eigen::VectorXd RdiagInv(Rdiag.cwiseInverse());
  const Eigen::MatrixXd Pq(Pkm.leftCols(nQ));  // n x nQ
  const Eigen::MatrixXd JtRinv(
      J.transpose() * RdiagInv.asDiagonal());  // nQ x m
  // K = Pkm H' (H Pkm H' + R)^-1 = Pq (I + J' R^-1 J Pqq)^-1 J' R^-1
  Eigen::MatrixXd G(
      Eigen::MatrixXd::Identity(nQ, nQ) +
      JtRinv * J * Pq.topRows(nQ));
  const Eigen::MatrixXd X(G.partialPivLu().solve(JtRinv));  // nQ x m
  const Eigen::MatrixXd K(Pq * X);                          // Gain, n x m

  *m_xp = xkm + K * (measure - projectedMeasure);  // New estimated state

  // Joseph form: P = (I - KH) Pkm (I - KH)' + K R K', where KH = [KJ 0 0]
  const Eigen::MatrixXd KJ(K * J);  // n x nQ
  const Eigen::MatrixXd Y(Pkm - KJ * Pkm.topRows(nQ));
  Eigen::MatrixXd Pnew(Y - Y.leftCols(nQ) * KJ.transpose());
  const Eigen::MatrixXd Z(
      X * Rdiag.asDiagonal() * X.transpose());  // nQ x nQ
  Pnew.noalias() += Pq * Z * Pq.transpose();
  // Keep P symmetric against round-off errors
  *m_Pp = 0.5 * (Pnew + Pnew.transpose());
}

void rigidbody::KalmanRecons::manageOcclusionDuringIteration(
    utils::Vector &measure,
    const std::vector<size_t> &occlusion) {
  for (size_t i = 0; i < occlusion.size(); ++i)
    for (size_t j = occlusion[i]; j < occlusion[i] + 1; ++j) {
      measure(j) = 0;
    }
}

void rigidbody::KalmanRecons::getState(
    rigidbody::GeneralizedCoordinates *Q,
    rigidbody::GeneralizedVelocity *Qdot,
    rigidbody::GeneralizedAcceleration *Qddot) {
  if (Q != nullptr) {
    *Q = m_xp->block(0, 0, *m_nbDof, 1);
  }

  if (Qdot != nullptr) {
    *Qdot = m_xp->block(*m_nbDof, 0, *m_nbDof, 1);
  }

  if (Qddot != nullptr) {
    *Qddot = m_xp->block(2 * *m_nbDof, 0, *m_nbDof, 1);
  }
}

utils::Matrix
rigidbody::KalmanRecons::evolutionMatrix(const size_t nQ, size_t n, double Te) {
  // m  : number of degrees of freedom
  // n  : order of Taylor development
  // Te : 1 / (acquisition frequency)

  n += 1;
  utils::Matrix A(utils::Matrix::Identity(nQ * n, nQ * n));
  double c = 1;
  for (size_t i = 2; i < n + 1; ++i) {
    size_t j = (i - 1) * nQ;
    c = c / (i - 1);

    for (size_t cmp = 0; cmp < nQ * n - j; ++cmp) {
      A(0 + cmp, j + cmp) +=
          c * static_cast<double>(std::pow(Te, (static_cast<double>(i) - 1.0)));
    }
  }

  return A;
}

utils::Matrix rigidbody::KalmanRecons::processNoiseMatrix(
    const size_t nbQ,
    double Te) {
  // Find the coefficients values
  double c1 = 1.0 / 20.0 * pow(Te, 5);
  double c2 = 1.0 / 8.0 * pow(Te, 4);
  double c3 = 1.0 / 6.0 * pow(Te, 3);
  double c4 = 1.0 / 3.0 * pow(Te, 3);
  double c5 = 1.0 / 2.0 * pow(Te, 2);
  double c6 = Te;

  // Ouput matrix
  utils::Matrix Q(utils::Matrix::Zero(3 * nbQ, 3 * nbQ));
  for (size_t j = 0; j < nbQ; ++j) {
    Q(j, j) = c1;
    Q(j, nbQ + j) = c2;
    Q(j, 2 * nbQ + j) = c3;
    Q(nbQ + j, j) = c2;
    Q(nbQ + j, nbQ + j) = c4;
    Q(nbQ + j, 2 * nbQ + j) = c5;
    Q(2 * nbQ + j, j) = c3;
    Q(2 * nbQ + j, nbQ + j) = c5;
    Q(2 * nbQ + j, 2 * nbQ + j) = c6;
  }

  return Q;
}

// Methods during the initialization
void rigidbody::KalmanRecons::initialize() {
  // Declaration of the evolution matrix
  *m_A = evolutionMatrix(*m_nbDof, 2, *m_Te);

  // Process Noise Matrix
  *m_Q = processNoiseMatrix(*m_nbDof, *m_Te);

  // Matrix of the noise on the measurement
  *m_R = measurementNoiseMatrix(*m_nMeasure, m_params->noiseFactor());

  // Initialize the state
  *m_xp = initState(*m_nbDof);

  // Matrix Pp
  *m_Pp = initCovariance(*m_nbDof, m_params->errorFactor());
}

utils::Matrix rigidbody::KalmanRecons::measurementNoiseMatrix(
    const size_t nbT,
    double val) {
  utils::Matrix R(utils::Matrix::Zero(nbT, nbT));
  for (size_t i = 0; i < nbT; ++i) {
    R(i, i) = val;
  }
  return R;
}

rigidbody::GeneralizedCoordinates rigidbody::KalmanRecons::initState(
    const size_t nbQ) {
  return rigidbody::GeneralizedCoordinates::Zero(3 * nbQ);  // Q, Qdot, Qddot
}

void rigidbody::KalmanRecons::setInitState(
    const rigidbody::GeneralizedCoordinates *Q,
    const rigidbody::GeneralizedVelocity *Qdot,
    const rigidbody::GeneralizedAcceleration *Qddot) {
  if (Q != nullptr) {
    m_xp->block(0, 0, *m_nbDof, 1) = *Q;
  }

  if (Qdot != nullptr) {
    m_xp->block(*m_nbDof, 0, *m_nbDof, 1) = *Qdot;
  }

  if (Qddot != nullptr) {
    m_xp->block(2 * *m_nbDof, 0, *m_nbDof, 1) = *Qddot;
  }
}

utils::Matrix rigidbody::KalmanRecons::initCovariance(
    const size_t nbQ,
    double val) {
  utils::Matrix Pp(utils::Matrix::Zero(3 * nbQ, 3 * nbQ));
  for (size_t i = 0; i < 3 * nbQ; ++i) {
    Pp(i, i) = val;
  }
  return Pp;
}

rigidbody::KalmanParam::KalmanParam(
    double frequency,
    double noiseFactor,
    double errorFactor)
    : m_acquisitionFrequency(frequency),
      m_noiseFactor(noiseFactor),
      m_errorFactor(errorFactor) {}

double rigidbody::KalmanParam::acquisitionFrequency() const {
  return m_acquisitionFrequency;
}

double rigidbody::KalmanParam::noiseFactor() const { return m_noiseFactor; }

double rigidbody::KalmanParam::errorFactor() const { return m_errorFactor; }
