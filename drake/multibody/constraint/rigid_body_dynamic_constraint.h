#pragma once

// TODO(#2274) NOTE This file has so many cpplint errors that we have
// whitelisted it in its entirety.  When the file is next rewritten or updates,
// we should re-enable cpplint accordingly.

#include <set>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SparseCore>

#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/rigid_body_tree.h"

namespace DrakeRigidBodyDynamicConstraint {
extern Eigen::Vector2d default_tspan;
}

/**
 * @class RigidBodyDynamicConstraint       The abstract base class. All constraints
 * used in the inverse kinematics problem are inherited from
 * RigidBodyDynamicConstraint. There are 6 main categories of the RigidBodyDynamicConstraint,
 * each category has its own interface
 */
template <typename T>
class RigidBodyDynamicConstraint {
 public:
  /* In each category, constraint classes share the same function interface.*/
  static const int PositionConstraintCategory = -1;
  /* Each non-abstrac RigidBodyDynamicConstraint class has a unique type.*/
  static const int CableDynamicConstraintType = 1;

  RigidBodyDynamicConstraint(
      int category, RigidBodyTree<T>* robot);
  int getType() const { return type_; }
  int getCategory() const { return category_; }
  RigidBodyTree<T>* getRobotPointer() const { return robot_; }
  virtual ~RigidBodyDynamicConstraint(void) = 0;

  /** template <typename Scalar> 
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> positionConstraints(
      const KinematicsCache<Scalar>& cache) const; */

  /* the critical methods: these will allow RBT to hook into these constraints at sim time.
   *  */
//  template <typename Scalar>
//  virtual Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
//  positionConstraints(
//      const KinematicsCache<Scalar>& cache) const {
//    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> ret(0, 1); // default is 0-by-1, no constraints
//    return ret;
//  }
//  template <typename Scalar>
//  virtual Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
//  positionConstraintsJacobian(const KinematicsCache<Scalar>& cache,
//                              bool in_terms_of_qdot = true) const {
//    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> ret(
//      0, in_terms_of_qdot ? robot_->get_num_velocities() : robot_->get_num_positions());
//           // default is 0-by-dim(q[dot]), no constraints
//    return ret;
//  }
//  template <typename Scalar>
//  virtual Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
//  positionConstraintsJacDotTimesV(
//      const KinematicsCache<Scalar>& cache) const {
//    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> ret(0, 1); // default is 0-by-1, no constraints
//    return ret;
//  }
//  virtual size_t
//  getNumPositionConstraints() const { return 0; }

 protected:
  void set_type(int type) { type_ = type; }
  void set_robot(RigidBodyTree<T>* robot) { robot_ = robot; }

  int category_{};
  int type_{};
  RigidBodyTree<T>* robot_{};

 //private:
};

/**
 * @class CableDynamicConstraint       -- Constrain the specific frame locations
 * on the robot to maintain a fixed distance from one another.
 * @param robot
 * @param pulleys_frame         -- The pulleys' frame ids
 * @param pulley_xyz           -- The pulleys' xyz offsets, in frame coords
 * @param pulley_axes          -- The pulleys' axes
 * @param pulley_radii
 * @param pulley_num_wraps
 * Terminators are first and last pulley locations. (TODO: Maybe make this shallow;
 * lists of bodies, axes, xyz, rpy rather than packaging in pulleys)
 *
 * MATLAB each pulley consisted of:
 *     p.frame = frame;   ~(int)
 *     p.xyz = xyz;       ~(Vector3d)
 *     p.axis = axis;     ~(Vector3d)
 *     p.radius = radius; ~(double, maybe T)
 *     p.number_of_wraps = number_of_wraps; ~(int)
 *
 * Function:
 *  @function positionConstraints     --evaluate the constraint
 *    @param cache     --the kinematic posture for which to evaluate the constraint
 */

template <typename T>
class CableDynamicConstraint {
 public:
  CableDynamicConstraint(RigidBodyTree<T>* robot,
    T cable_length,
    const std::vector<int>& pulley_frames,
    const std::vector<Eigen::Vector3d>& pulley_xyz_offsets,
    const std::vector<Eigen::Vector3d>& pulley_axes,
    const std::vector<T>& pulley_radii,
    const std::vector<int>& pulley_num_wraps);

//      const std::set<int>& model_instance_id_set =
//          CableDynamicConstraint::defaultRobotNumSet);
  virtual ~CableDynamicConstraint(void);
  //  bool isTimeValid(const double* t) const;
  //  int getNumConstraint(const double* t) const;
  //  void eval(const double* t, KinematicsCache<double>& cache,
  //            const double* weights, Eigen::VectorXd& c,
  //            Eigen::MatrixXd& dc) const;
  //  void bounds(const double* t, Eigen::VectorXd& lb, Eigen::VectorXd& ub) const;
  //  void name(const double* t, std::vector<std::string>& name_str) const;
  //  bool isActive() const { return active_; }
  //  int getNumWeights() const { return num_pts_; }
  //  void addContact(int num_new_bodies, const int* body,
  //                  const Eigen::Matrix3Xd* body_pts);
  //
  //  void addContact(std::vector<int> body, const Eigen::Matrix3Xd& body_pts) {
  //    addContact(body.size(), body.data(), &body_pts);
  //}
  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> positionConstraints(
      const KinematicsCache<Scalar>& cache) const;
  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
  positionConstraintsJacobian(const KinematicsCache<Scalar>& cache,
                              bool in_terms_of_qdot = true) const;
  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
  positionConstraintsJacDotTimesV(
      const KinematicsCache<Scalar>& cache) const;

  size_t getNumPositionConstraints() const;
  
  void updateRobot(RigidBodyTree<T>* robot);
  void updatePulleyRadii(std::vector<T>& pulley_radii);

 private:
  RigidBodyTree<T>* robot_{};
  T cable_length_;
  std::vector<int> pulley_frames_;
  std::vector<Eigen::Vector3d> pulley_xyz_offsets_;
  std::vector<Eigen::Vector3d> pulley_axes_;
  std::vector<T> pulley_radii_;
  std::vector<int> pulley_num_wraps_;
};
