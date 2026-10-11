#ifndef BIORBD_WRAPPING_CASADI_PRIVATE_H
#define BIORBD_WRAPPING_CASADI_PRIVATE_H

#include "biorbdConfig.h"
#include "WrappingNumericalKernel.h"
#include <casadi/casadi.hpp>

namespace BIORBD_NAMESPACE { namespace internal_forces {
struct CasadiWrappingExpressions {
    casadi::MX length, gradient, points;
};

// The kinematic Function maps Q to a dense vector containing all anchors (3
// values each), then each object-to-global RT (16 column-major values), and
// to that vector's Jacobian. Region i connects anchors i and i+1.
CasadiWrappingExpressions makeCasadiWrapping(
    const casadi::Function& kinematics, const casadi::MX& q,
    const std::vector<::biorbd::wrapping::Shape>& shapes,
    const std::vector<std::vector<size_t>>& regions, size_t anchorCount);
} }
#endif
