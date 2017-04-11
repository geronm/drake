#include "drake/multibody/constraint/rigid_body_dynamic_constraint.h"


// TODO(#2274) NOTE This file has so many cpplint errors that we have
// whitelisted it in its entirety.  When the file is next rewritten or updates,
// we should re-enable cpplint accordingly.

#include <map>
#include <stdexcept>

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/gradient.h"
#include "drake/math/quaternion.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/util/drakeGeometryUtil.h"

using Eigen::AutoDiffScalar;
using Eigen::Isometry3d;
using Eigen::Map;
using Eigen::Matrix3Xd;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::SparseMatrix;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::math::autoDiffToGradientMatrix;
using drake::math::autoDiffToValueMatrix;
using drake::math::gradientMatrixToAutoDiff;
using drake::math::initializeAutoDiffTuple;
using drake::math::quatDiff;
using drake::math::quatDiffAxisInvar;

namespace DrakeRigidBodyDynamicConstraint {
  // constants go here
}

template <typename T>
RigidBodyDynamicConstraint<T>::RigidBodyDynamicConstraint(
    int category, RigidBodyTree<T>* robot)
    : category_(category), type_(0), robot_(robot) {
  if (category >= 0 || category <= -7) {
    throw std::runtime_error(
        "Drake:RigidBodyDynamicConstraint:Unsupported constraint category");
  }
  //if (tspan(0) > tspan(1)) {
  //  throw std::runtime_error(
  //      "Drake:RigidBodyDynamicConstraint:tspan(0) should be no larger than tspan(1)");
  //}
  // tspan_[0] = tspan(0);
  // tspan_[1] = tspan(1);
}

template <typename T>
RigidBodyDynamicConstraint<T>::~RigidBodyDynamicConstraint(void) {}

namespace {
const int QuasiStaticDefaultRobotNum[1] = {0};
}

// Explicitly instantiate the most common scalar types.
template class RigidBodyDynamicConstraint<double>;