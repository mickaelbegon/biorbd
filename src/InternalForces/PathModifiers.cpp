#define BIORBD_API_EXPORTS
#include "InternalForces/PathModifiers.h"

#include "Utils/Error.h"
#include "Utils/Vector3d.h"
#include "InternalForces/ViaPoint.h"
#include "InternalForces/WrappingSphere.h"
#include "InternalForces/WrappingHalfCylinder.h"
#include "InternalForces/WrappingCylinder.h"
#include "InternalForces/WrappingEllipsoid.h"
#include "InternalForces/WrappingTorus.h"

using namespace BIORBD_NAMESPACE;

internal_forces::PathModifiers::PathModifiers() :
    m_obj(std::make_shared<std::vector<std::shared_ptr<utils::Vector3d>>>()),
    m_nbWraps(std::make_shared<unsigned int>(0)),
    m_nbVia(std::make_shared<unsigned int>(0)),
    m_totalObjects(std::make_shared<unsigned int>(0))
{

}

internal_forces::PathModifiers internal_forces::PathModifiers::DeepCopy() const
{
    internal_forces::PathModifiers copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::PathModifiers::DeepCopy(const
        internal_forces::PathModifiers &other)
{
    m_obj->resize(other.m_obj->size());
    for (unsigned int i=0; i<other.m_obj->size(); ++i) {
        const utils::Vector3d& object = *(*other.m_obj)[i];
        switch (object.typeOfNode()) {
        case utils::NODE_TYPE::WRAPPING_CYLINDER:
            (*m_obj)[i] = std::make_shared<WrappingCylinder>(
                static_cast<const WrappingCylinder&>(object).DeepCopy());
            break;
        case utils::NODE_TYPE::WRAPPING_ELLIPSOID:
            (*m_obj)[i] = std::make_shared<WrappingEllipsoid>(
                static_cast<const WrappingEllipsoid&>(object).DeepCopy());
            break;
        case utils::NODE_TYPE::WRAPPING_TORUS:
            (*m_obj)[i] = std::make_shared<WrappingTorus>(
                static_cast<const WrappingTorus&>(object).DeepCopy());
            break;
        case utils::NODE_TYPE::WRAPPING_HALF_CYLINDER:
            (*m_obj)[i] = std::make_shared<WrappingHalfCylinder>(
                static_cast<const WrappingHalfCylinder&>(object).DeepCopy());
            break;
        case utils::NODE_TYPE::WRAPPING_SPHERE:
            (*m_obj)[i] = std::make_shared<WrappingSphere>(
                static_cast<const WrappingSphere&>(object).DeepCopy());
            break;
        case utils::NODE_TYPE::VIA_POINT:
            (*m_obj)[i] = std::make_shared<ViaPoint>(
                static_cast<const ViaPoint&>(object).DeepCopy());
            break;
        default:
            utils::Error::raise("Unknown path modifier in DeepCopy");
        }
    }
    *m_nbWraps = *other.m_nbWraps;
    *m_nbVia = *other.m_nbVia;
    *m_totalObjects = *other.m_totalObjects;
}

// Private method to assing values
void internal_forces::PathModifiers::addPathChanger(
    utils::Vector3d &object)
{

    // Add a muscle to the pool of muscle depending on type
    if (object.typeOfNode() == utils::NODE_TYPE::WRAPPING_CYLINDER) {
        m_obj->push_back(std::make_shared<WrappingCylinder>(
            dynamic_cast<WrappingCylinder&>(object)));
        ++*m_nbWraps;
    } else if (object.typeOfNode() == utils::NODE_TYPE::WRAPPING_ELLIPSOID) {
        m_obj->push_back(std::make_shared<WrappingEllipsoid>(
            dynamic_cast<WrappingEllipsoid&>(object)));
        ++*m_nbWraps;
    } else if (object.typeOfNode() == utils::NODE_TYPE::WRAPPING_TORUS) {
        m_obj->push_back(std::make_shared<WrappingTorus>(
            dynamic_cast<WrappingTorus&>(object)));
        ++*m_nbWraps;
    } else if (object.typeOfNode() == utils::NODE_TYPE::WRAPPING_SPHERE) {
        utils::Error::check(*m_nbVia == 0,
                                    "Cannot mix via points and wrapping objects yet");
        m_obj->push_back(std::make_shared<internal_forces::WrappingSphere>(
                             static_cast<internal_forces::WrappingSphere&> (object)));
        ++*m_nbWraps;
    } else if (object.typeOfNode() ==
               utils::NODE_TYPE::WRAPPING_HALF_CYLINDER) {
        utils::Error::check(*m_nbVia == 0,
                                    "Cannot mix via points and wrapping objects yet");
        m_obj->push_back(std::make_shared<internal_forces::WrappingHalfCylinder>(
                             dynamic_cast <internal_forces::WrappingHalfCylinder&> (object)));
        ++*m_nbWraps;
    } else if (object.typeOfNode() == utils::NODE_TYPE::VIA_POINT) {
        for (const auto& existing : *m_obj) {
            utils::Error::check(existing->typeOfNode() != utils::NODE_TYPE::WRAPPING_SPHERE &&
                existing->typeOfNode() != utils::NODE_TYPE::WRAPPING_HALF_CYLINDER,
                "Cannot mix legacy wrapping and via points yet");
        }
        m_obj->push_back(std::make_shared<internal_forces::ViaPoint>(
                             dynamic_cast <internal_forces::ViaPoint&> (object)));
        ++*m_nbVia;
    } else {
        utils::Error::raise("Wrapping type not found");
    }
    ++*m_totalObjects;
}

unsigned int internal_forces::PathModifiers::nbWraps() const
{
    return *m_nbWraps;
}

unsigned int internal_forces::PathModifiers::nbVia() const
{
    return *m_nbVia;
}

unsigned int internal_forces::PathModifiers::nbObjects() const
{
    return *m_totalObjects;
}

utils::Vector3d& internal_forces::PathModifiers::object(
    unsigned int idx)
{
    utils::Error::check(idx<nbObjects(),
                                "Idx asked is higher than number of wrapping objects");
    return *(*m_obj)[idx];
}


const utils::Vector3d& internal_forces::PathModifiers::object(
    unsigned int idx) const
{
    utils::Error::check(idx<nbObjects(),
                                "Idx asked is higher than number of wrapping objects");
    return *(*m_obj)[idx];
}


