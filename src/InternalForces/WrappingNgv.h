#ifndef BIORBD_WRAPPING_NGV_PRIVATE_H
#define BIORBD_WRAPPING_NGV_PRIVATE_H
#include "WrappingNumericalKernel.h"
namespace biorbd { namespace wrapping {
// Private candidate data. Contact frames are retained for the common kernel's
// full tangent-direction and outward-normal checks before acceptance.
struct NgvContact {
    Contact curve;
    Eigen::Vector3d entryTangent,exitTangent,entryNormal,exitNormal;
};
struct NgvRegion {
    std::vector<NgvContact> contacts; ///< In Region::objects order.
    double length,smoothness;
    int iterations;
};
// Stateless cold Scholz2015 solve. Hints are supplied in each object's local
// frame by the native geometric initializer; no model or fixture references.
NgvRegion solveNgvRegion(const std::vector<Object>& objects,const Region& region,
                        const std::vector<Eigen::Vector3d>& hints);
} }
#endif
