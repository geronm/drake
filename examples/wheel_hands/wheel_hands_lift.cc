/// @file
///
/// This file implements a test that the Schunk WSG50 gripper can grip
/// a box which is sitting on the world, raise it into the air, and
/// not drop the box.
///
/// This test is, in some sense, a warning alarm.  Given the implemented contact
/// model and the encoded parameters, this test implicitly defines an acceptable
/// *behavior*.  It doesn't test the contact model's functionality (the contact
/// model has its own unit tests.)  It also doesn't guarantee physical
/// correctness. Instead, it evaluates the contact model in a larger integrated
/// context, quantifies "valid" behavior, in some sense, and serves as an early
/// warning that can indicate if something has changed in the system such that
/// the final system no longer reproduces the expected baseline behavior.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/kinematics_results.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/primitives/trajectory_source.h"

namespace drake {
namespace examples {
namespace wheel_hands {
// namespace {

DEFINE_double(tp, 10.0, "Proportional constant for theta PID controller");
DEFINE_double(ti, 0., "Integral constant for theta PID controller");
DEFINE_double(td, 2.5, "Derivative constant for theta PID controller");
DEFINE_double(fp, 3000., "Proportional constant for finger PID controller");
DEFINE_double(fi, 0., "Integral constant for finger PID controller");
DEFINE_double(fd, 50., "Derivative constant for finger PID controller");

using drake::systems::RungeKutta3Integrator;
using drake::systems::ContactResultsToLcmSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::KinematicsResults;
using drake::systems::MatrixGain;
using drake::systems::Integrator;
using drake::systems::SignalLogger;
using drake::systems::lcm::LcmtDrakeSignalTranslator;
using Eigen::Vector3d;

// Initial height of the box's origin.
const double kBoxInitZ = 0.085;
const double kBoxInitRoll = 1.57079632;
const double kBoxInitPitch = 0.0;

std::unique_ptr<RigidBodyTreed> BuildLiftTestTree(
    int* lifter_instance_id, int* gripper_instance_id) {
  std::unique_ptr<RigidBodyTreed> tree = std::make_unique<RigidBodyTreed>();
  multibody::AddFlatTerrainToWorld(tree.get());

  drake::log()->info("TEST Print 090901.");

  // Add a joint to the world which can lift the gripper.
  const auto lifter_id_table =
      parsers::sdf::AddModelInstancesFromSdfFile(
      FindResourceOrThrow("drake/examples/wheel_hands/models/test_lifter.sdf"),
      multibody::joints::kFixed, nullptr, tree.get());
  // EXPECT_EQ(lifter_id_table.size(), 1);
  *lifter_instance_id = lifter_id_table.begin()->second;

  drake::log()->info("TEST Print 3252345.");

  /// Add the gripper.  Offset it slightly back and up so that we can
  // locate the target at the origin.
  auto gripper_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "lifted_link_frame",
      tree->FindBody("lifted_link"), Eigen::Vector3d(0, -0.05, 0.05),
      Eigen::Vector3d::Zero());
  const auto gripper_id_table = parsers::sdf::AddModelInstancesFromSdfFile(
      FindResourceOrThrow(
          "drake/examples/wheel_hands/models/schunk_wsg_50_wheel_fingers.sdf"),
      multibody::joints::kFixed, gripper_frame, tree.get());
  // EXPECT_EQ(gripper_id_table.size(), 1);
  *gripper_instance_id = gripper_id_table.begin()->second;

  drake::log()->info("TEST Print 72353.");

  // Add a box to grip.
  auto box_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world",
      nullptr,
      Eigen::Vector3d(0, 0.005, kBoxInitZ), Eigen::Vector3d(kBoxInitRoll, kBoxInitPitch, 0));
  parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow("drake/multibody/models/cylinder_flat.urdf"),
      multibody::joints::kQuaternion, box_frame, tree.get());

  drake::log()->info("TEST Print 612341.");

  tree->compile();

  drake::log()->info("TEST Print 823523.");

  return tree;
}

// GTEST_TEST(SchunkWsgLiftTest, BoxLiftTest) {
int main() {
  systems::DiagramBuilder<double> builder;

  int lifter_instance_id{};
  int gripper_instance_id{};
  systems::RigidBodyPlant<double>* plant =
      builder.AddSystem<systems::RigidBodyPlant<double>>(
          BuildLiftTestTree(&lifter_instance_id, &gripper_instance_id));
  plant->set_name("plant");

  DRAKE_DEMAND(plant->get_num_actuators() == 4);
  DRAKE_DEMAND(plant->get_num_model_instances() == 3);

  // Arbitrary contact parameters.
  // const double kStiffness = 10000;
  // const double kDissipation = 2.0;
  // const double kStaticFriction = 0.9;
  // const double kDynamicFriction = 0.5;
  // const double kVStictionTolerance = 0.01;
  // plant->set_normal_contact_parameters(kStiffness, kDissipation);
  // plant->set_friction_contact_parameters(kStaticFriction, kDynamicFriction,
  //                                        kVStictionTolerance);

  // Arbitrary contact parameters.
  const double kYoungsModulus = 1e8;  // Pa
  const double kDissipation = 2.0;  // s/m
  const double kStaticFriction = 0.9;
  const double kDynamicFriction = 0.5;
  systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(kYoungsModulus)
      .set_dissipation(kDissipation)
      .set_friction(kStaticFriction, kDynamicFriction);
  plant->set_default_compliant_material(default_material);

  const double kVStictionTolerance = 0.01;  // m/s
  const double kContactRadius = 2e-4;  // m
  systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_radius = kContactRadius;
  model_parameters.v_stiction_tolerance = kVStictionTolerance;
  plant->set_contact_model_parameters(model_parameters);

  // Build a trajectory and PID controller for the lifting joint.
  const auto& lifting_input_port =
      plant->model_instance_actuator_command_input_port(lifter_instance_id);
  const auto& lifting_output_port =
      plant->model_instance_state_output_port(lifter_instance_id);

  // Constants chosen arbitrarily.
  const Vector1d lift_kp(300.);
  const Vector1d lift_ki(0.);
  const Vector1d lift_kd(5.);

  auto lifting_pid_ports =
      systems::controllers::PidControlledSystem<double>::ConnectController(
          lifting_input_port, lifting_output_port,
          lift_kp, lift_ki, lift_kd, &builder);

  // // Immediately rename the generic "pid_controller" and "input_adder"
  // for (unsigned int jjj=0; jjj < builder.GetMutableSystems().size(); jjj++) {
  //   if (builder.GetMutableSystems()[jjj]->get_name() == "pid_controller") {
  //     drake::log()->info(builder.GetMutableSystems()[jjj]->get_name());
  //     builder.GetMutableSystems()[jjj]->set_name("lifter_pid_controller");
  //     drake::log()->info(builder.GetMutableSystems()[jjj]->get_name());
  //   }
  //   if (builder.GetMutableSystems()[jjj]->get_name() == "input_adder") {
  //     drake::log()->info(builder.GetMutableSystems()[jjj]->get_name());
  //     builder.GetMutableSystems()[jjj]->set_name("lifter_input_adder");
  //     drake::log()->info(builder.GetMutableSystems()[jjj]->get_name());
  //   }
  // }

  drake::log()->info("TEST Print 143163.");

  auto zero_source =
      builder.AddSystem<systems::ConstantVectorSource<double>>(
          Eigen::VectorXd::Zero(1));
  zero_source->set_name("zero");
  builder.Connect(zero_source->get_output_port(),
                  lifting_pid_ports.control_input_port);

  drake::log()->info("TEST Print 275434.");

  // Create a trajectory with 3 seconds of main lift time from second
  // 1 to second 4, coming to a stop by second 5.  The initial delay
  // is to allow the gripper to settle on the box before we start
  // moving.
  const double kLiftHeight = 1.0;
  const double kLiftStart = 1.0;
  std::vector<double> lift_breaks{0., 0.9, kLiftStart, 4., 5};
  std::vector<Eigen::MatrixXd> lift_knots;
  lift_knots.push_back(Eigen::Vector2d(0., 0.));
  lift_knots.push_back(Eigen::Vector2d(0., 0.));
  lift_knots.push_back(Eigen::Vector2d(0., 0.09 * kLiftHeight));
  lift_knots.push_back(Eigen::Vector2d(0.9 * kLiftHeight, 0.09 * kLiftHeight));
  // Stop gradually.
  lift_knots.push_back(Eigen::Vector2d(kLiftHeight, 0.));
  PiecewisePolynomialTrajectory lift_trajectory(
      PiecewisePolynomial<double>::Cubic(
          lift_breaks, lift_knots, Eigen::Vector2d(0., 0.),
          Eigen::Vector2d(0., 0.)));
  auto lift_source =
      builder.AddSystem<systems::TrajectorySource>(lift_trajectory);
  lift_source->set_name("lift_source");
  builder.Connect(lift_source->get_output_port(),
                  lifting_pid_ports.state_input_port);

  drake::log()->info("TEST Print Q11234.");

  drake::log()->info("TEST Print D23842. Number input ports: {}", plant->get_input_size());

  for (int iii = 0; iii < 13; iii++) {
    drake::log()->info("TEST Print ZX2342. State descriptor: {} {}", iii, plant->get_rigid_body_tree().getStateName(iii));
  }
  // for (int iii = 0; iii < plant->get_input_size(); iii++) {
  //   drake::log()->info("TEST Print Oh843u. Input descriptor: {} {}", iii, plant->get_input_port(iii));
  // }
  drake::log()->info("Lifter num input ports: {}", plant->model_instance_actuator_command_input_port(lifter_instance_id).size());
  drake::log()->info("Gripper num input ports: {}", plant->model_instance_actuator_command_input_port(gripper_instance_id).size());

  const RigidBodyTreed& tree = plant->get_rigid_body_tree();
  auto positions = tree.computePositionNameToIndexMap();

  for (int iii = 0; iii < tree.get_num_positions() + tree.get_num_velocities(); iii++) {
    drake::log()->info("State {} has name {}", iii, tree.getStateName(iii));
  }

  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //
    //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //
  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //
    //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //
  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //
    //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //
  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //




  // The schunk gripper will exert a constant squeezing force
  // on the manipuland. Meanwhile, the roller fingers will
  // be PID controlled to maintain a fixed location on the
  // surface of the manipuland.
  //
  // We splice together these input ports on the schunk
  // model, using a Multiplexer system


  // Create multiplexer. Our system now has two identical input ports,
  // which combine additively to provide input to the gripper
  // actuators.
  std::vector<int> multiplexer_input_sizes{1, 2};
  systems::Multiplexer<double>& multiplexer =
      *builder.template AddSystem<systems::Multiplexer<double>>(multiplexer_input_sizes);
  builder.Connect(multiplexer.get_output_port(0),
                  plant->model_instance_actuator_command_input_port(
                      gripper_instance_id));

  drake::log()->info("TEST Print A11234.");

  // Create a trajectory for grip force.
  // Settle the grip by the time the lift starts.
  std::vector<double> grip_breaks{0., kLiftStart - 0.1, kLiftStart};
  std::vector<Eigen::MatrixXd> grip_knots;
  grip_knots.push_back(Vector1d(0. ));
  grip_knots.push_back(Vector1d(0. ));
  grip_knots.push_back(Vector1d(40.));
  PiecewisePolynomialTrajectory grip_trajectory(
      PiecewisePolynomial<double>::FirstOrderHold(grip_breaks, grip_knots));
  auto grip_source =
      builder.AddSystem<systems::TrajectorySource>(grip_trajectory);
  grip_source->set_name("grip_source");
  builder.Connect(grip_source->get_output_port(),
                  multiplexer.get_input_port(0));

  drake::log()->info("TEST Print S11234.");

  // Create a trajectory for finger position/velocity
  // command.
  std::vector<double> finger_command_breaks{0., kLiftStart};
  std::vector<Eigen::MatrixXd> finger_command_knots;
  finger_command_knots.push_back(Eigen::VectorXd::Zero(4));
  finger_command_knots.push_back(Eigen::VectorXd::Zero(4));
  PiecewisePolynomialTrajectory finger_command_trajectory(
      PiecewisePolynomial<double>::FirstOrderHold(finger_command_breaks, finger_command_knots));
  auto finger_command_source =
      builder.AddSystem<systems::TrajectorySource>(finger_command_trajectory);
  finger_command_source->set_name("finger_command_source");

  drake::log()->info("TEST Print D11234.");

  // Build a PID controller for fingers
  //
  // State will be [theta1, theta2, theta1_dot, theta2_dot]
  // Input will be [torque1, torque2]

  // Constants chosen arbitrarily.
  const Eigen::Vector2d finger_kp(FLAGS_fp, FLAGS_fp);
  const Eigen::Vector2d finger_ki(FLAGS_fi, FLAGS_fi);
  const Eigen::Vector2d finger_kd(FLAGS_fd, FLAGS_fd);

  // Choose the ports which will be PID controlled. In this
  // case, one of the Multiplexer inputs and the Plant's output.
  const auto& finger_pid_input_port =
      multiplexer.get_input_port(1);
  // const auto& gripper_full_state_output_port =
  //     plant->model_instance_state_output_port(gripper_instance_id);

  // Get a matrix to take plant state and select only
  // finger state readings.

  // Want 1 & 5 (left_finger_roller_jointdot & right_finger_roller_jointdot)
  Eigen::MatrixXd gripper_pid_state_selector =
      Eigen::MatrixXd::Zero(2*2, 7*2);
  gripper_pid_state_selector(2*0+0, 7*0+1) = 1.0;
  gripper_pid_state_selector(2*0+1, 7*0+5) = 1.0;
  gripper_pid_state_selector(2*1+0, 7*1+1) = 1.0;
  gripper_pid_state_selector(2*1+1, 7*1+5) = 1.0;
  // MatrixGain<double>& gripper_pid_state_selector_gain =
  //     *builder.template AddSystem<MatrixGain<double>>(gripper_pid_state_selector);
  // builder.Connect(gripper_full_state_output_port,
  //                 gripper_pid_state_selector_gain.get_input_port());
  // const auto& finger_pid_output_port =
  //     gripper_pid_state_selector_gain.get_output_port();
  const auto& finger_pid_output_port =
      plant->model_instance_state_output_port(gripper_instance_id);

  drake::log()->info("Number 1: {}.", finger_pid_input_port.size());
  drake::log()->info("Number 2: {}.", finger_pid_output_port.size());
  drake::log()->info("Number 3: {}.", gripper_pid_state_selector.rows());
  drake::log()->info("Number 4: {}.", gripper_pid_state_selector.cols());
  drake::log()->info("Number 5: {}.", 2);
  drake::log()->info("Number 6: {}.", 4);

  auto gripper_pid_ports =
      systems::controllers::PidControlledSystem<double>::ConnectController(
          finger_pid_input_port,
          finger_pid_output_port,
          gripper_pid_state_selector,
          finger_kp,
          finger_ki,
          finger_kd,
          &builder);

  // Immediately rename the generic "pid_controller" and "input_adder"
  for (unsigned int jjj=0; jjj < builder.GetMutableSystems().size(); jjj++) {
    if (builder.GetMutableSystems()[jjj]->get_name() == "pid_controller") {
      drake::log()->info(builder.GetMutableSystems()[jjj]->get_name());
      builder.GetMutableSystems()[jjj]->set_name("gripper_pid_controller");
      drake::log()->info(builder.GetMutableSystems()[jjj]->get_name());
    }
    if (builder.GetMutableSystems()[jjj]->get_name() == "input_adder") {
      drake::log()->info(builder.GetMutableSystems()[jjj]->get_name());
      builder.GetMutableSystems()[jjj]->set_name("gripper_input_adder");
      drake::log()->info(builder.GetMutableSystems()[jjj]->get_name());
    }
  }

  // gripper_pid_ports.control_input_port.get_system()->set_name("finger_input_adder");
  // gripper_pid_ports.state_input_port.get_system()->set_name("finger_pid_controller");
  drake::log()->info("Ctrl input port system name: {}", gripper_pid_ports.control_input_port.get_system()->get_name());
  drake::log()->info("State input port system name: {}", gripper_pid_ports.state_input_port.get_system()->get_name());
  drake::log()->info("TEST Print F11234.");


  drake::log()->info(plant->state_output_port().size());
  drake::log()->info(plant->get_num_positions());
  drake::log()->info(plant->get_num_velocities());

  // State  0 has name lift_joint
  // State 10 has name base_z
  // State 15 has name lift_jointdot
  // State 28 has name base_vz


  // |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
  //
  // Produce the Outer PID Controller
  //
  //  y_desired --+--> thetadot_c --> [integrator] --> [splitter] --> thetas_c --> [Inner PID System] --> [matrix] -+--> y
  //              |_________________________________________________________________________________________________|
  //
  Eigen::MatrixXd state_to_control_state_transform_matrix =
      Eigen::MatrixXd::Zero(2, 29);
  state_to_control_state_transform_matrix(0,  0) =  1.0;  // "y"
  state_to_control_state_transform_matrix(0,  9) = -1.0; // DO NOT ask me why this is 9 and not 10!
  state_to_control_state_transform_matrix(1, 15) = 1.0;  // "ydot"
  state_to_control_state_transform_matrix(1, 28) = -1.0; //
  auto& state_to_control_state_transform =
      *builder.template AddSystem<systems::MatrixGain<double>>(state_to_control_state_transform_matrix);

  // integrator
  auto& set_point_integrator =
      *builder.template AddSystem<systems::Integrator<double>>(1);

  // splitter
  Eigen::MatrixXd set_point_splitter_matrix =
      Eigen::MatrixXd::Zero(4, 1);
  set_point_splitter_matrix(0,  0) = -1.0;
  set_point_splitter_matrix(1,  0) =  1.0;
  set_point_splitter_matrix(2,  0) =  0.0;
  set_point_splitter_matrix(3,  0) =  0.0;
  auto& set_point_splitter =
      *builder.template AddSystem<systems::MatrixGain<double>>(set_point_splitter_matrix);


  // Constants chosen arbitrarily.
  const Vector1d theta_kp(FLAGS_tp);
  const Vector1d theta_ki(FLAGS_ti);
  const Vector1d theta_kd(FLAGS_td);
  // const Eigen::Vector2d theta_kp(300.);
  // const Eigen::Vector2d theta_ki(0.  );
  // const Eigen::Vector2d theta_kd(5.  );

  // alias ports for clarity
  const auto& theta_pid_plant_input_port =
      set_point_integrator.get_input_port();
  const auto& theta_pid_plant_output_port =
      state_to_control_state_transform.get_output_port();

  // wire up PID
  auto theta_pid_ports =
      systems::controllers::PidControlledSystem<double>::ConnectController(
          theta_pid_plant_input_port,
          theta_pid_plant_output_port,
          theta_kp,
          theta_ki,
          theta_kd,
          &builder);

  // wiring
  builder.Connect(set_point_splitter.get_output_port(),
                  gripper_pid_ports.state_input_port);
  builder.Connect(plant->state_output_port(),
                  state_to_control_state_transform.get_input_port());
  builder.Connect(set_point_integrator.get_output_port(),
                  set_point_splitter.get_input_port());

  auto zero_source_1a =
      builder.AddSystem<systems::ConstantVectorSource<double>>(
          Eigen::VectorXd::Zero(1));
  auto zero_source_2a =
      builder.AddSystem<systems::ConstantVectorSource<double>>(
          Eigen::VectorXd::Zero(2));
  builder.Connect(zero_source_1a->get_output_port(),
                  theta_pid_ports.control_input_port);
  builder.Connect(zero_source_2a->get_output_port(),
                  theta_pid_ports.state_input_port);

  //
  //
  // |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

  auto zero_source_2 =
      builder.AddSystem<systems::ConstantVectorSource<double>>(
          Eigen::VectorXd::Zero(2));
  zero_source_2->set_name("zero2");
  // auto zero_source_4 =
  //     builder.AddSystem<systems::ConstantVectorSource<double>>(
  //         Eigen::VectorXd::Zero(4));
  // zero_source_4->set_name("zero4");
  builder.Connect(zero_source_2->get_output_port(),
                  gripper_pid_ports.control_input_port);
  // builder.Connect(zero_source_4->get_output_port(),
  //                 gripper_pid_ports.state_input_port);

  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //
    //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //
  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //
    //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //
  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //
    //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //
  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //

  drake::log()->info("TEST Print R09987.");

  // Creates and adds LCM publisher for visualization.  The test doesn't
  // require `drake_visualizer` but it is convenient to have when debugging.
  drake::lcm::DrakeLcm lcm;
  auto viz_publisher = builder.template AddSystem<systems::DrakeVisualizer>(
      plant->get_rigid_body_tree(), &lcm, true);
  viz_publisher->set_name("visualization_publisher");
  builder.Connect(plant->state_output_port(),
                  viz_publisher->get_input_port(0));

  drake::log()->info("TEST Print A13451.");

  // contact force visualization
  ContactResultsToLcmSystem<double>& contact_viz =
      *builder.template AddSystem<ContactResultsToLcmSystem<double>>(
          plant->get_rigid_body_tree());
  contact_viz.set_name("contact_visualization");
  auto& contact_results_publisher = *builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm));
  contact_results_publisher.set_name("contact_results_publisher");
  // Contact results to lcm msg.
  builder.Connect(plant->contact_results_output_port(),
                  contact_viz.get_input_port(0));
  builder.Connect(contact_viz.get_output_port(0),
                  contact_results_publisher.get_input_port(0));

  // // control signal logging
  // auto& control_signal_logger = *builder.AddSystem<SignalLogger<double>>(theta_pid_plant_output_port.size());
  // control_signal_logger.set_name("control_signal_logger");
  // builder.Connect(theta_pid_plant_output_port,
  //                 control_signal_logger.get_input_port());

  // control signal publishing
  // LcmtDrakeSignalTranslator control_signal_translator(2);
  LcmtDrakeSignalTranslator control_signal_translator(plant->state_output_port().size());
  auto& control_signal_publisher = *builder.AddSystem<LcmPublisherSystem>(
          "CONTROL_SIGNAL", control_signal_translator, &lcm);
  // control_signal_publisher.set_name("control_signal_publisher");
  builder.Connect(plant->state_output_port(), // theta_pid_plant_output_port,
                  control_signal_publisher.get_input_port(0));

  drake::log()->info("TEST Print Y28341238.");

  const int plant_output_port = builder.ExportOutput(plant->get_output_port(0));
  // Expose the RBPlant kinematics results as a diagram output for body state
  // validation.
  const int kinematrics_results_index =
      builder.ExportOutput(plant->get_output_port(
          plant->kinematics_results_output_port().get_index()));

  drake::log()->info("System transform matrix:\n{}", gripper_pid_state_selector);
  drake::log()->info("Info: {}", 
              plant->get_output_port(plant->kinematics_results_output_port().get_index()).size()
          );


  // Set up the model and simulator and set their starting state.
  const std::unique_ptr<systems::Diagram<double>> model = builder.Build();
  systems::Simulator<double> simulator(*model);


  // Open the gripper.  Due to the number of links involved, this is
  // surprisingly complicated.
  systems::Context<double>& plant_context =
      model->GetMutableSubsystemContext(
          *plant, &simulator.get_mutable_context());
  Eigen::VectorXd plant_initial_state =
      Eigen::VectorXd::Zero(plant->get_num_states());
  plant_initial_state.head(plant->get_num_positions())
      = tree.getZeroConfiguration();








  // ASSERT_EQ(positions["left_finger_sliding_joint"], 1);

  // The values below were extracted from the positions corresponding
  // to an open gripper.  Dumping them here is significantly more
  // magic than I (sam.creasey) would like.  If you find yourself
  // tempted to cut and paste this, please consider creating a utility
  // function which can set a segment of a state vector to an open
  // gripper.
  //
  // Adapted from schunk gripper which had
  //
  // 0 lift_joint
  // 1 left_finger_sliding_joint
  // 2 left_finger_push
  // 3 nonphysical_rotor_mount
  // 4 right_finger_sliding_joint
  // 5 right_finger_push
  // 6 base_x
  // 7 base_y
  // 8 base_z
  // 9 base_qw
  // 10 base_qx
  //
  // New gripper has:
  //
  // 0 lift_joint
  // 1 left_finger_sliding_joint
  // 2 left_finger_roller_joint
  // 3 left_finger_push
  // 4 nonphysical_rotor_mount
  // 5 right_finger_sliding_joint
  // 6 right_finger_roller_joint
  // 7 right_finger_push
  // 8 base_x
  // 9 base_y
  // 10 base_z
  // 11 base_qw
  // 12 base_qx
  //
  // State  0 has name lift_joint
  // State  1 has name left_finger_sliding_joint
  // State  2 has name left_finger_roller_joint
  // State  3 has name left_finger_push
  // State  4 has name nonphysical_rotor_mount
  // State  5 has name right_finger_sliding_joint
  // State  6 has name right_finger_roller_joint
  // State  7 has name right_finger_push
  // State  8 has name base_x
  // State  9 has name base_y
  // State 10 has name base_z
  // State 11 has name base_qw
  // State 12 has name base_qx
  // State 13 has name base_qy
  // State 14 has name base_qz
  // State 15 has name lift_jointdot
  // State 16 has name left_finger_sliding_jointdot
  // State 17 has name left_finger_roller_jointdot
  // State 18 has name left_finger_pushdot
  // State 19 has name nonphysical_rotor_mountdot
  // State 20 has name right_finger_sliding_jointdot
  // State 21 has name right_finger_roller_jointdot
  // State 22 has name right_finger_pushdot
  // State 23 has name base_wx
  // State 24 has name base_wy
  // State 25 has name base_wz
  // State 26 has name base_vx
  // State 27 has name base_vy
  // State 28 has name base_vz

  plant_initial_state(positions["left_finger_sliding_joint"]) = -0.0550667;
  plant_initial_state(positions["left_finger_roller_joint"]) = 0.0;
  plant_initial_state(positions["left_finger_push"]) = 0.009759;
  plant_initial_state(positions["nonphysical_rotor_mount"]) = 1.27982;
  plant_initial_state(positions["right_finger_sliding_joint"]) = 0.0550667;
  plant_initial_state(positions["right_finger_roller_joint"]) = 0.0;
  plant_initial_state(positions["right_finger_push"]) = 0.009759;

  // plant_initial_state(1) = -0.0550667;
  // plant_initial_state(2) = 0.009759;
  // plant_initial_state(3) = 1.27982;
  // plant_initial_state(4) = 0.0550667;
  // plant_initial_state(5) = 0.009759;
  plant->set_state_vector(&plant_context, plant_initial_state);

  systems::Context<double>& context = simulator.get_mutable_context();

  simulator.reset_integrator<RungeKutta3Integrator<double>>(*model, &context);
  simulator.get_mutable_integrator()->request_initial_step_size_target(1e-4);
  simulator.get_mutable_integrator()->set_target_accuracy(1e-3);

  simulator.Initialize();

  // Simulate to one second beyond the trajectory motion.
  const double kSimDuration = lift_breaks[lift_breaks.size() - 1] + 1.0;

  // Simulation in two pieces -- see notes below on the test for details.
  simulator.StepTo(kLiftStart);

  // Capture the "initial" positions of the box and the gripper finger as
  // discussed in the test notes below.
  auto state_output = model->AllocateOutput(simulator.get_context());
  model->CalcOutput(simulator.get_context(), state_output.get());
  auto& interim_kinematics_results =
      state_output->get_data(kinematrics_results_index)
          ->GetValue<KinematicsResults<double>>();
  const int box_index = tree.FindBodyIndex("cylinder");
  Vector3d init_box_pos =
      interim_kinematics_results.get_body_position(box_index);
  const int finger_index = tree.FindBodyIndex("left_finger");
  Vector3d init_finger_pos =
      interim_kinematics_results.get_body_position(finger_index);

  // Now run to the end of the simulation.
  simulator.StepTo(kSimDuration);

  // Extract and log the state of the robot.
  model->CalcOutput(simulator.get_context(), state_output.get());
  const auto final_output_data =
      state_output->get_vector_data(plant_output_port)->get_value();

  drake::log()->info("Final state:");
  const int num_movable_links = plant->get_num_positions();
  for (int link_index = 0; link_index < num_movable_links; link_index++) {
    drake::log()->info("  {} {}: {} (v={})",
                        link_index, tree.get_position_name(link_index),
                        final_output_data[link_index],
                        final_output_data[link_index + num_movable_links]);
  }


  // drake::log()->info("Logged data size:\n {}", control_signal_logger.data().size());
  // drake::log()->info("Logged data shape:\n {} x {}", control_signal_logger.data().rows(), control_signal_logger.data().cols());
  // drake::log()->info("Logged data:\n");
  // for (int i = 0; i < control_signal_logger.data().cols(); i+=100) {
  //   drake::log()->info("{}", control_signal_logger.data()(1, i));
  // }

  // Some computations regarding the final gripper / manipuland pose.

  // Compute expected final position and compare with observed final position.
  auto& final_kinematics_results =
      state_output->get_data(kinematrics_results_index)
          ->GetValue<KinematicsResults<double>>();
  Vector3d final_finger_pos =
      final_kinematics_results.get_body_position(finger_index);
  Vector3d ideal_final_pos(init_box_pos);
  ideal_final_pos += final_finger_pos - init_finger_pos;
  Vector3d final_pos = final_kinematics_results.get_body_position(box_index);
  Vector3d displacement = final_pos - ideal_final_pos;
  double distance = displacement.norm();

  // // Lift duration is a sub-interval of the full simulation.
  const double kLiftDuration = kSimDuration - kLiftStart;
  const double kMeanSlipSpeed = distance / kLiftDuration;
  // EXPECT_LT(kMeanSlipSpeed, kVStictionTolerance);

  // std::cout << "EXPECTING LT: " << kMeanSlipSpeed << ", " << kVStictionTolerance << std::endl;

  while (1) viz_publisher->ReplayCachedSimulation();

  return 0;
}

// }  // namespace
}  // namespace schunk_wsg
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::wheel_hands::main();
}