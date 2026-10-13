#ifndef BIORBD_MUSCLE_AUDIT_TEST_UTILS_H
#define BIORBD_MUSCLE_AUDIT_TEST_UTILS_H
#include <cmath>
#include <cstdio>
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>

#include "BiorbdModel.h"
#include "InternalForces/Muscles/all.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "Utils/Matrix.h"
#include "Utils/String.h"
#include "Utils/Vector.h"
using namespace BIORBD_NAMESPACE;
namespace muscle_audit {
inline double value(const utils::Scalar& scalar) {
  SCALAR_TO_DOUBLE(result, scalar);
  return result;
}
struct TemporaryModel {
  std::string path;
  TemporaryModel(
      const std::string& name,
      const std::string& oldText,
      const std::string& newText)
      : path("models/" + name + ".bioMod") {
    std::ifstream input("models/muscle_audit.bioMod");
    std::stringstream buffer;
    buffer << input.rdbuf();
    std::string text = buffer.str();
    size_t offset = text.find(oldText);
    if (offset == std::string::npos)
      throw std::runtime_error("Missing fixture text");
    text.replace(offset, oldText.size(), newText);
    std::ofstream output(path);
    output << text;
  }
  ~TemporaryModel() { std::remove(path.c_str()); }
};
}  // namespace muscle_audit
#endif
