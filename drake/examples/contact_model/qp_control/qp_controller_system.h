#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/examples/contact_model/qp_control/qp_controller.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {

/**
 * A discrete time system block for an inverse dynamics controller.
 */
class QpControllerSystem : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QpControllerSystem)

  /**
   * Constructor for the inverse dynamics controller.
   * @param robot Reference to a RigidBodyTree. Its lifespan must be longer
   * than this object.
   * @param dt Control cycle period.
   * @param alpha Parameter for Jacobian controller. On each timestep,
   *    change x by -alpha*dt*Jinv*error
   */
  QpControllerSystem(const RigidBodyTree<double>& robot, double dt, double alpha, const std::vector<int> control_input_indices);

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  // std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
  //     const systems::OutputPortDescriptor<double>& descriptor) const override;

  // std::unique_ptr<systems::AbstractValues> AllocateAbstractState()
  //     const override;

  // // // // std::unique_ptr<ContinuousState<double>> AllocateContinuousState() const override;

  std::unique_ptr<systems::DiscreteState<double>> AllocateDiscreteState() const override;

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const override;

  /**
   * Returns the input port for HumanoidStatus.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_q_manip_desired() const {
    return get_input_port(input_port_index_q_manip_desired_);
  }

  // /**
  //  * Returns the input port for QpInput.
  //  */
  // inline const systems::InputPortDescriptor<double>& get_input_port_qp_input()
  //     const {
  //   return get_input_port(input_port_index_qp_input_);
  // }

  /**
   * Returns the output port for QpOutput.
   */
  inline const systems::OutputPortDescriptor<double>&
  get_output_port_x_desired_u() const {
    return get_output_port(output_port_index_x_desired_u_);
  }

  // /**
  //  * Returns the output port for lcmt_inverse_dynamics_debug_info.
  //  */
  // inline const systems::OutputPortDescriptor<double>&
  // get_output_port_debug_info() const {
  //   return get_output_port(output_port_index_debug_info_);
  // }

  inline double get_control_dt() const { return control_dt_; }

 private:
  const RigidBodyTree<double>& robot_;
  const double control_dt_{};
  const double control_alpha_{};
  // Indices into AbstractState. These are updated by DoCalcUnrestrictedUpdate,
  // and DoCalcOutput copies them to the output ports.
  const int kAbstractStateIndexXDesiredU{0};
  const int kAbstractStateIndexCachedP{1};

  const double kPaddleHalfWidth{4.0};
  const double kPaddleHalfDepth{1.0};
  const double kBoxWidth{2.0};

  const double kPaddleMaxAngle{0.01};

  const size_t kXDesiredULength{12};
  const size_t kQManipLength{2};

  // TODO(siyuan.feng): This is a bad temporary hack to the const constraint for
  // CalcOutput. It is because qp controller needs to allocate mutable workspace
  // (MathematicalProgram, temporary matrices for doing math, etc),
  // and I want to avoid allocating these repeatedly.
  // This should be taken care of with the new system2 cache.
  mutable QPController qp_controller_;

  Eigen::Matrix<double,3,4> finger_forwardkin_xyz_;
  std::vector<std::string> finger_forwardkin_names_;
  const std::vector<int> control_input_indices_;

  // int input_port_index_humanoid_status_{0};
  // int input_port_index_qp_input_{0};
  int input_port_index_q_manip_desired_{0};
  // int output_port_index_qp_output_{0};
  int output_port_index_x_desired_u_{0};
};

}  // namespace examples
}  // namespace drake
