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

template <typename T>
CableDynamicConstraint<T>::CableDynamicConstraint(RigidBodyTree<T>* robot,
    T cable_length,
    const std::vector<int>& pulley_frames,
    const std::vector<Eigen::Vector3d>& pulley_xyz_offsets,
    const std::vector<Eigen::Vector3d>& pulley_axes,
    const std::vector<T>& pulley_radii,
    const std::vector<int>& pulley_num_wraps)
    // const std::set<int>& model_instance_id_set)
    :   robot_(robot),
        cable_length_(cable_length),
        pulley_frames_(pulley_frames),
        pulley_xyz_offsets_(pulley_xyz_offsets),
        pulley_axes_(pulley_axes),
        pulley_radii_(pulley_radii),
        pulley_num_wraps_(pulley_num_wraps) {
  // pass
}

template <typename T>
CableDynamicConstraint<T>::~CableDynamicConstraint() {}

template <typename T>
template <typename Scalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, 1> CableDynamicConstraint<T>::positionConstraints(
      const KinematicsCache<Scalar>& cache) const {
  this->robot_->CheckCacheValidity(cache);
  
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> ret(1, 1); // default is 1-by-1, one constraint

  Scalar phi = 0.0;

  // Need at least 2 anchor points in order for there to be any length to phi
  if (pulley_frames_.size() >= 2) {
    auto last_p = this->robot_->transformPoints(cache, pulley_xyz_offsets_[0],
                                      pulley_frames_[0],
                                      0);
    // TODO deleteme VVVV
    // std::cout << "Teehee: " << std::endl << last_p << std::endl;
    // Every pulley adds some length to phi
    for (size_t i = 1; i < pulley_frames_.size(); ++i) {
      {  // position constraint
        auto cur_p = this->robot_->transformPoints(cache, pulley_xyz_offsets_[i],
                                        pulley_frames_[i],
                                        0);
        phi += (cur_p - last_p).norm();

        last_p = cur_p;

        // TODO deleteme VVVV
        // std::cout << "Teehee: " << std::endl << last_p << std::endl;
      }
    }
  }

  phi -= cable_length_;

  ret(0, 0) = phi;
  return ret;
}

template <typename T>
template <typename Scalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> CableDynamicConstraint<T>::positionConstraintsJacobian(
                            const KinematicsCache<Scalar>& cache,
                            bool in_terms_of_qdot) const {
  this->robot_->CheckCacheValidity(cache);

  // Checked with Twan; in_terms_of_qdot will change the structures of the vectors returned
  // by RigidBodyTree::transformPoints and RigidBodyTree::transformPointsJacobian, but
  // this code should still just sort of work?
  
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> ret(
    1, in_terms_of_qdot ? this->robot_->get_num_velocities() : this->robot_->get_num_positions());
         // 1-by-dim(q[dot]), one constraint
  ret.setZero();

  // Eigen::Vector<Scalar> dphi(in_terms_of_qdot ? robot_->get_num_velocities() : robot_->get_num_positions());
  // dphi.setZero();

  // Need at least 2 anchor points in order for there to be any length to phi
  if (pulley_frames_.size() >= 2) {
    auto last_p = this->robot_->transformPoints(cache, pulley_xyz_offsets_[0],
                                      pulley_frames_[0],
                                      0);
    auto d_last_p = this->robot_->transformPointsJacobian(cache, pulley_xyz_offsets_[0],
                                      pulley_frames_[0],
                                      0,
                                      in_terms_of_qdot);

    // TODO deleteme VVVV
    // std::cout << "Teehee 1: " << std::endl << last_p << std::endl;

    // Every pulley adds some length to phi
    for (size_t i = 1; i < pulley_frames_.size(); ++i) {
      {  // position constraint
        auto cur_p = this->robot_->transformPoints(cache, pulley_xyz_offsets_[i],
                                        pulley_frames_[i],
                                        0);
        auto d_cur_p = this->robot_->transformPointsJacobian(cache, pulley_xyz_offsets_[i],
                                        pulley_frames_[i],
                                        0,
                                        in_terms_of_qdot);

        // MATLAB Reference:
        //
        // vec = pt-last_pt;
        // C = sqrt(vec'*vec);
        //
        // if nargout>1
        //   dvec = dpt-last_dpt;
        //   dC = vec'*dvec/C;

        auto vec = (cur_p - last_p);
        auto C = vec.norm(); // C represents distance from previous pulley to current pulley

        // (TODO(#XXXX) handle C=0 silently, shouldn't just be doomed to divide-by-zero)
        auto dvec = d_cur_p - d_last_p;
        auto dC = (vec.transpose() * dvec) / (C + EPSILON);

        // MATLAB Reference:
        //
        // length = length+C;
        // last_attachment_pt = pt;
        // if nargout>1
        //   dlength = dlength+dC;
        //   last_attachment_dpt = dpt;
        // end

        ret.row(0) += dC;

        last_p = cur_p;
        d_last_p = d_cur_p;

        // TODO deleteme VVVV
        // std::cout << "Teehee 2: " << std::endl << last_p << std::endl;
      }
    }
  }

  return ret;
}

template <typename T>
template <typename Scalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, 1> CableDynamicConstraint<T>::positionConstraintsJacDotTimesV(
    const KinematicsCache<Scalar>& cache) const {

  // This one is complicated. Put the following into LaTeX for a glimpse of the formula:
  //
  //
  /*
   \begin{align*}
     \dot{J}_v v = & \\
       &\frac{(p_{i+1}-p_i)^{\top}}{|p_{i+1}-p_i|}(\dot{J}_{v,i+1}v-\dot{J}_{v,i}v)  \\ + &\\
       &               \left(\left[\frac{1}{|p_{i+1}-p_i|}-\frac{(p_{i+1}-p_i)(p_{i+1}-p_i)^{\top}}{|p_{i+1}-p_i|^3}\right](J_{v,i+1}v-J_{v,i}v)\right)^{\top}
                    (J_{v,i+1}-J_{v,i})v
   \end{align*}
  */
  //

  this->robot_->CheckCacheValidity(cache);

  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> ret(1,1);
  ret.setZero();

  auto q = cache.getQ();
  auto v = cache.getV();

  // Need at least 2 anchor points in order for there to be any length to phi
  if (pulley_frames_.size() >= 2) {
    auto last_p = this->robot_->transformPoints(cache, pulley_xyz_offsets_[0],
                                      pulley_frames_[0],
                                      0);
    auto d_last_p = this->robot_->transformPointsJacobian(cache, pulley_xyz_offsets_[0],
                                      pulley_frames_[0],
                                      0,
                                      false);
    auto jdv_last_p = this->robot_->transformPointsJacobianDotTimesV(cache, pulley_xyz_offsets_[0],
                                      pulley_frames_[0],
                                      0);

    // TODO deleteme VVVV
    // std::cout << "Teehee 1: " << std::endl << last_p << std::endl;

    // Every pulley adds some length to phi
    for (size_t i = 1; i < pulley_frames_.size(); ++i) {
      {  // position constraint
        auto cur_p = this->robot_->transformPoints(cache, pulley_xyz_offsets_[i],
                                        pulley_frames_[i],
                                        0);
        auto d_cur_p = this->robot_->transformPointsJacobian(cache, pulley_xyz_offsets_[i],
                                        pulley_frames_[i],
                                        0,
                                        false);
        auto jdv_cur_p = this->robot_->transformPointsJacobianDotTimesV(cache, pulley_xyz_offsets_[i],
                                        pulley_frames_[i],
                                        0);

        auto vec = (cur_p - last_p);
        auto C = vec.norm(); // C represents distance from previous pulley to current pulley

        // (TODO(#XXXX) handle C=0 silently, shouldn't just be doomed to divide-by-zero)

        // Term: 1/|p2-p1| * (p2-p1)^T * (J2'q' - J1'q')
        ret += (vec.transpose() * (jdv_cur_p - jdv_last_p)) / (C + EPSILON);


        // Term: ([1/|p2-p1| - (p2-p1)(p2-p1)^T/(|p2-p1|^3)] * (p2'-p1'))^T * (p2'-p1')
        auto dvec_dt = (d_cur_p - d_last_p) * v;  // (p2'-p1')
        ret +=  ( (  dvec_dt  -  vec*(vec.transpose())*dvec_dt/(C + EPSILON)/(C + EPSILON)  ) / (C + EPSILON) ).transpose() * dvec_dt;

        last_p = cur_p;
        d_last_p = d_cur_p;
        jdv_last_p = jdv_cur_p;
      }
    }
  }

  return ret;
}

template <typename T>
size_t CableDynamicConstraint<T>::getNumPositionConstraints() const {
  return 1; // every pulley imposes 1 constraint on the system.
}

template <typename T>
void CableDynamicConstraint<T>::updateRobot(RigidBodyTree<T>* robot) {
  robot_ = robot;
}

template <typename T>
void CableDynamicConstraint<T>::updatePulleyRadii(std::vector<T>& pulley_radii) {
  pulley_radii_ = pulley_radii;
}


// Explicitly instantiate the most common scalar types.
template class RigidBodyDynamicConstraint<double>;

template class CableDynamicConstraint<double>;

template Eigen::Matrix<double, Eigen::Dynamic, 1> CableDynamicConstraint<double>::positionConstraints(
      const KinematicsCache<double>& cache) const;
template Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> CableDynamicConstraint<double>::positionConstraintsJacobian(
                            const KinematicsCache<double>& cache,
                            bool in_terms_of_qdot) const;
template Eigen::Matrix<double, Eigen::Dynamic, 1> CableDynamicConstraint<double>::positionConstraintsJacDotTimesV(
  const KinematicsCache<double>& cache) const;