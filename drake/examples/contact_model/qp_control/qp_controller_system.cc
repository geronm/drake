#include "drake/examples/contact_model/qp_control/qp_controller_system.h"

#include <memory>
#include <utility>
#include <vector>

#include "drake/lcmt_inverse_dynamics_debug_info.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {

namespace {

template <typename ValueType>
ValueType& get_mutable_value(systems::State<double>* state, int index) {
  DRAKE_DEMAND(state);
  return state->get_mutable_abstract_state()
      ->get_mutable_value(index)
      .GetMutableValue<ValueType>();
}

}  // namespace


  // int input_port_index_q_manip_desired_{0};
  // int output_port_index_x_desired_u_{0};
QpControllerSystem::QpControllerSystem(const RigidBodyTree<double>& robot,
                                       double dt, double alpha, const std::vector<int> control_input_indices)
    : robot_(robot), control_dt_(dt), control_alpha_(alpha),
      qp_controller_(1.0, 1.0),
      finger_forwardkin_xyz_(3, 4),
      control_input_indices_(control_input_indices) {
  input_port_index_q_manip_desired_ = DeclareInputPort(systems::kVectorValued, 2).get_index();
  output_port_index_x_desired_u_ = DeclareOutputPort(systems::kVectorValued, control_input_indices_.size()).get_index();
  // DeclareContinuousState(kXDesiredULength + kQManipLength);

  // Store useful data for retrieving the proper finger points
  finger_forwardkin_names_.push_back("finger1_paddle");
  finger_forwardkin_names_.push_back("finger1_paddle");
  finger_forwardkin_names_.push_back("finger2_paddle");
  finger_forwardkin_names_.push_back("finger2_paddle");
  for (size_t r=0; r<3; r++) {
    for (size_t c=0; c<4; c++) {
      finger_forwardkin_xyz_(r,c) = 0.0;
      if (r==0) {
        if (c % 2 == 0) {
          finger_forwardkin_xyz_(r,c) = kPaddleHalfWidth;
        } else {
          finger_forwardkin_xyz_(r,c) = -kPaddleHalfWidth;
        }
      }
      if (r==1) {
        if (c % 2 == 0) {
          finger_forwardkin_xyz_(r,c) = kPaddleHalfDepth;
        } else {
          finger_forwardkin_xyz_(r,c) = kPaddleHalfDepth;
        }
      }
    }
  }
  std::cout << "Hello from QP Controller System!:" << std::endl;
  std::cout << finger_forwardkin_xyz_ << std::endl;

  set_name("QpControllerSystem");
  DeclarePeriodicUnrestrictedUpdate(control_dt_, 0);
}

void QpControllerSystem::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {


  if (context.get_time() >= 0.18) {
    // std::cout << "DoCalcOutput" << std::endl;
  }
  
  // Our only output is x_desired_u, which is stored in the state itself

  // drake::systems::BasicVector<double>* qp_output = output->GetMutableVectorData(output_port_index_x_desired_u_);
  // for (size_t i=0; i < (size_t)qp_output->size(); i++) {
  //   qp_output->SetAtIndex(i,0.0);
  // }
  // for (size_t u=0; u<control_input_indices_.size()/2; u++) {
  //   qp_output->SetAtIndex(control_input_indices_[u], context.get_discrete_state(0)->GetAtIndex(u));
  // }

  // Obtain the structure we need to write into.
  systems::BasicVector<double>* const output_vector =
      output->GetMutableVectorData(output_port_index_x_desired_u_);
  DRAKE_ASSERT(output_vector != nullptr);

  // Output port value is just the continuous state.
  // for (size_t i=0; i<12; i++) {
  //   (output_vector->get_mutable_value())(i) = 0.0;
  // }

  if (context.get_time() >= 0.18) {
    // std::cout << context.get_time() << std::endl;
    // std::cout << context.get_discrete_state(0)->CopyToVector().transpose() << std::endl;
    // std::cout << output_vector->get_mutable_value().transpose() << std::endl;
  }

  for (size_t u=0; u<(size_t)control_input_indices_.size(); u++) {
    (output_vector->get_mutable_value())(u) =
          (context.get_discrete_state(0)->CopyToVector())(u);
  }
}


void QpControllerSystem::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  std::cout << "Doing DoCalcUnrestrictedUpdate..." << std::endl;

  // Inputs:
  // const Eigen::VectorXd* q_manip_desired = EvalInputValue<Eigen::VectorXd>(
  //     context, input_port_index_q_manip_desired_);

  // // const Eigen::VectorXd*
  // const BasicVector<double>* q_manip_desired_bv = EvalVectorInput(context, input_port_index_q_manip_desired_);
  // // const Eigen::VectorXd q_manip_desired(q_manip_desired_bv->size());
  // const Eigen::VectorXd q_manip_desired = q_manip_desired_bv->CopyToVector();


  const Eigen::VectorXd q_manip_desired = EvalVectorInput(context, input_port_index_q_manip_desired_)->CopyToVector();
  

  // Gets the x state
  // Eigen::VectorXd& x_desired_u =
  //     get_mutable_value<Eigen::VectorXd>(state, kAbstractStateIndexXDesiredU);

  // // // // systems::ContinuousState<double>* mutable_state = state->get_mutable_continuous_state();
  // systems::DiscreteState<double>* mutable_state = state->get_mutable_discrete_state(0);
  //drake::systems::VectorBase<double>* state_vector = mutable_state->get_mutable_discrete_state();
  // Eigen::VectorXd state_vector_eigen(kXDesiredULength + kQManipLength);
  // for (size_t i=0; i<kXDesiredULength + kQManipLength; i++) {
  //   state_vector_eigen(i) = mutable_state-> // state_vector->GetAtIndex(i);
  // }

  std::cout << "Print ABC" << std::endl;
  auto x = context.get_discrete_state(0)->get_value();
  Eigen::VectorXd state_vector_eigen(kXDesiredULength + kQManipLength);
  for (size_t i=0; i<kXDesiredULength + kQManipLength; i++) {
    state_vector_eigen(i) = x[i]; // state_vector->GetAtIndex(i);
  }

  std::cout << "Print D" << std::endl;
  // Finds the four finger points from x_desired_u
  Eigen::VectorXd p_ij(8);
  p_ij.setZero();

  Eigen::VectorXd q_fake(this->robot_.get_num_positions());
  q_fake.setZero();
  for (size_t u=0; u<control_input_indices_.size()/2; u++) {
    q_fake(control_input_indices_[u]) = state_vector_eigen(u); // x_desired_u(u);
  }
  KinematicsCache<double> cache = this->robot_.doKinematics(q_fake);

  std::cout << "q_fake: " << q_fake.transpose() << std::endl;

  std::cout << "Print E" << std::endl;
  // Get the four points
  for (size_t f=0; f<finger_forwardkin_names_.size(); f++) {
    Eigen::Vector3d xyz_vec;
    xyz_vec(0) = finger_forwardkin_xyz_(0,f);
    xyz_vec(1) = finger_forwardkin_xyz_(1,f);
    xyz_vec(2) = finger_forwardkin_xyz_(2,f);
    Eigen::Vector3d p_finger = this->robot_.template transformPoints<double>(cache, xyz_vec,
                                        this->robot_.FindBodyIndex(finger_forwardkin_names_[f]),
                                        0);
    p_ij(2*f + 0) = p_finger(0);
    p_ij(2*f + 1) = p_finger(1);
    // Don't care about z-value of p_finger
  }

  // Reduce to point-case by collapsing corners
  //   ^ 0  1
  // x | 3  2
  //   +----->
  //      y
  p_ij(2*0 + 0) -= 0;
  p_ij(2*0 + 1) -= kBoxWidth;
  p_ij(2*1 + 0) -= kBoxWidth;
  p_ij(2*1 + 1) -= kBoxWidth;
  p_ij(2*2 + 0) -= kBoxWidth;
  p_ij(2*2 + 1) -= 0;
  p_ij(2*3 + 0) -= 0;
  p_ij(2*3 + 1) -= 0;
  

  std::cout << "Print F" << std::endl;
  Eigen::VectorXd q_manip_ss(2);
  Eigen::MatrixXd d_pmanip_d_pij(2,8);
  qp_controller_.getQManipAndJac(p_ij, q_manip_ss, d_pmanip_d_pij);

  // Transform q_manip_ss back by un-collapsing corners
  q_manip_ss(0) += (kBoxWidth/2);
  q_manip_ss(1) += (kBoxWidth/2);

  // Go from pointwise-Jacobian back to statewise-Jacobian
  Eigen::MatrixXd d_pij_d_q(8,q_fake.size());
  for (size_t f=0; f<finger_forwardkin_names_.size(); f++) {
    Eigen::Vector3d xyz_vec;
    xyz_vec(0) = finger_forwardkin_xyz_(0,f);
    xyz_vec(1) = finger_forwardkin_xyz_(1,f);
    xyz_vec(2) = finger_forwardkin_xyz_(2,f);
    Eigen::MatrixXd d_p_d_q = this->robot_.template transformPointsJacobian<double>(cache, xyz_vec,
                                        this->robot_.FindBodyIndex(finger_forwardkin_names_[f]),
                                        0,
                                        true);
    for (size_t i=0; i<(size_t)q_fake.size(); i++) {
      d_pij_d_q(2*f+0, i) = d_p_d_q(0, i);
      d_pij_d_q(2*f+1, i) = d_p_d_q(1, i);
    }
  }
  Eigen::MatrixXd d_pmanip_d_q = d_pmanip_d_pij * d_pij_d_q;

  std::cout << "p_ij:" << std::endl;
  std::cout << p_ij.transpose() << std::endl;
  std::cout << "d_pij_d_q: " << std::endl;
  std::cout << d_pij_d_q << std::endl;
  std::cout << "d_pmanip_d_pij: " << std::endl;
  std::cout << d_pmanip_d_pij << std::endl;
  std::cout << "d_pmanip_d_q: " << std::endl;
  std::cout << d_pmanip_d_q << std::endl;

  std::cout << "Print G" << std::endl;
  // Update x_desired_u via the Jacobian control update rule
  Eigen::MatrixXd q_manip_error = q_manip_desired - q_manip_ss;
  Eigen::VectorXd delta_x_desired = control_alpha_*control_dt_*(d_pmanip_d_q.transpose())*q_manip_error;

  std::cout << "control_alpha_: " << control_alpha_ << std::endl;
  std::cout << "control_dt_: " << control_dt_ << std::endl;
  std::cout << "d_pmanip_d_q.transpose(): " << d_pmanip_d_q.transpose() << std::endl;
  std::cout << "q_manip_error: " << q_manip_error << std::endl;

  std::cout << "delta_x_desired: " << std::endl;
  std::cout << delta_x_desired.transpose() << std::endl;

  for (size_t u=0; u<control_input_indices_.size(); u++) {
    if (u < control_input_indices_.size()/2) {
      // x_desired_u(u) += delta_x_desired[(control_input_indices_[u])]; 
      state_vector_eigen(u) += delta_x_desired[(control_input_indices_[u])]; 
    } else {
      // x_desired_u(u) = 0; // qdot desired is always 0
      state_vector_eigen(u) = 0; // qdot desired is always 0
    }
  }

  // Final guard: project x_desired_u back into safety 
  // TODO Make proper guard; for now, it just says don't go above or below a certain amount
  double safety_limit = 3.0;
  for (size_t i=0; i < (size_t)state_vector_eigen.size(); i++) {
    if (state_vector_eigen(i) > safety_limit) {
      state_vector_eigen(i) = safety_limit;
    } else if (state_vector_eigen(i) < -safety_limit) {
      state_vector_eigen(i) = -safety_limit;
    }
  }
  if (state_vector_eigen(0) - state_vector_eigen(3) > -.1) {
    std::cout << "Over a!!" << std::endl;
    double avg = .5 * (state_vector_eigen(0) + state_vector_eigen(3));
    state_vector_eigen(0) = avg - .05;
    state_vector_eigen(3) = avg + .05;
  } else if (state_vector_eigen(0) - state_vector_eigen(3) < -.2) {
    std::cout << "Over b!!" << std::endl;
    double avg = .5 * (state_vector_eigen(0) + state_vector_eigen(3));
    state_vector_eigen(0) = avg - .1;
    state_vector_eigen(3) = avg + .1;
  }
  if (state_vector_eigen(1) - state_vector_eigen(4) > 1.0) {
    std::cout << "Over2 a!!" << std::endl;
    double avg = .5 * (state_vector_eigen(1) + state_vector_eigen(4));
    state_vector_eigen(1) = avg + .5;
    state_vector_eigen(4) = avg - .5;
  } else if (state_vector_eigen(1) - state_vector_eigen(4) < -1.0) {
    std::cout << "Over2 b!!" << std::endl;
    double avg = .5 * (state_vector_eigen(1) + state_vector_eigen(4));
    state_vector_eigen(1) = avg - .5;
    state_vector_eigen(4) = avg + .5;
  }

  if (state_vector_eigen(0) + state_vector_eigen(2) > kPaddleMaxAngle) {
    std::cout << "Over3 a!!" << std::endl;
    state_vector_eigen(2) = kPaddleMaxAngle - state_vector_eigen(0);
  } else if (state_vector_eigen(0) + state_vector_eigen(2) < -kPaddleMaxAngle) {
    std::cout << "Over3 b!!" << std::endl;
    state_vector_eigen(2) = -kPaddleMaxAngle - state_vector_eigen(0);
  }
  if (state_vector_eigen(3) + state_vector_eigen(5) > kPaddleMaxAngle) {
    std::cout << "Over3 c!!" << std::endl;
    state_vector_eigen(5) = kPaddleMaxAngle - state_vector_eigen(3);
  } else if (state_vector_eigen(3) + state_vector_eigen(5) < -kPaddleMaxAngle) {
    std::cout << "Over3 d!!" << std::endl;
    state_vector_eigen(5) = -kPaddleMaxAngle - state_vector_eigen(3);
  }

  std::cout << "Print H" << std::endl;
  // Put state_vector_eigen back into state vector
  state->get_mutable_discrete_state()->get_mutable_discrete_state(0)->SetFromVector(state_vector_eigen);

  // Some prints
  std::cout << "current q_manip_desired: " << q_manip_desired.transpose() << std::endl;
  std::cout << "current q_manip_ss: " << q_manip_ss.transpose() << std::endl;
  std::cout << "current state_vector_eigen: " << state_vector_eigen.transpose() << std::endl;
}

// std::unique_ptr<systems::AbstractValues>
// QpControllerSystem::AllocateAbstractState() const {
//   std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals(2);
//   abstract_vals[kAbstractStateIndexXDesiredU] =
//       std::unique_ptr<systems::AbstractValue>(
//           new systems::Value<Eigen::VectorXd>(Eigen::VectorXd(kXDesiredULength)));
//   abstract_vals[kAbstractStateIndexCachedP] =
//       std::unique_ptr<systems::AbstractValue>(
//           new systems::Value<Eigen::VectorXd>(Eigen::VectorXd(kQManipLength)));
//   return std::make_unique<systems::AbstractValues>(std::move(abstract_vals));
// }

// // // // std::unique_ptr<ContinuousState<double>> QpControllerSystem::AllocateContinuousState()
// // // //     const {
// // // //   return make_unique<ContinuousState<double>>(
// // // //       make_unique<BasicVector<double>>(get_num_states()),
// // // //       get_num_positions() /* num_q */, get_num_velocities() /* num_v */,
// // // //       0 /* num_z */);
// // // // }

std::unique_ptr<systems::DiscreteState<double>> QpControllerSystem::AllocateDiscreteState()
    const {
  std::cout << "Doing AllocateDiscreteState..." << std::endl;

  return std::make_unique<systems::DiscreteState<double>>(
      std::make_unique<systems::BasicVector<double>>(kXDesiredULength + kQManipLength));
}

// std::unique_ptr<systems::AbstractValue>
// QpControllerSystem::AllocateOutputAbstract(
//     const systems::OutputPortDescriptor<double>& descriptor) const {
//   if (descriptor.get_index() == output_port_index_x_desired_u_) {
//     return systems::AbstractValue::Make<Eigen::VectorXd>(
//         Eigen::VectorXd(kXDesiredULength));
//   } else {
//     DRAKE_DEMAND(false);
//     return nullptr;
//   }
// }

}  // namespace examples
}  // namespace drake
