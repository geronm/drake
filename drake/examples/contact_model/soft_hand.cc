/*! @file
A simple example of a rigid gripper attempting to hold a block. The gripper
has rigid geometry: two fingers at a fixed distance from each other. They
are positioned in a configuration *slightly* narrower than the box placed
between them.

This is a test to evaluate/debug the contact model.  This configuration
simplifies the test by defining a known penetration and eliminating all
controller-dependent variation.

This is an even simpler example of what is shown in schung_wsg_lift_test.
This eliminates the PID controller and ground contact.  At the end of the
simulation, the box should have slipped an amount less than the duration
length times v_stiction_tolerance (i.e., the highest allowable slip speed
during stiction).
*/

#include <memory>
#include <time.h>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/lcmt_viewer2_comms.hpp"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/constraint/rigid_body_dynamic_constraint.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/gain.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/demultiplexer.h"

DEFINE_double(accuracy, 5e-5, "Sets the simulation accuracy");
DEFINE_double(us, 0.9, "The coefficient of static friction");
DEFINE_double(ud, 0.5, "The coefficient of dynamic friction");
DEFINE_double(stiffness, 10000, "The contact material stiffness");
DEFINE_double(dissipation, 2.0, "The contact material dissipation");
DEFINE_double(v_stiction_tolerance, 0.01,
              "The maximum slipping speed allowed during stiction");
DEFINE_double(sim_duration, 5, "Amount of time to simulate");
DEFINE_bool(playback, true,
            "If true, simulation begins looping playback when complete");
DEFINE_double(fing1_force, -10.0,
            "The force exerted at proximal torsion joint 1");
DEFINE_double(fing2_force, 10.0,
            "The force exerted at proximal torsion joint 2");
DEFINE_double(fing_spring, 10.0,
            "Spring constant k");

namespace drake {
namespace examples {

using drake::systems::RungeKutta3Integrator;
using drake::systems::ContactResultsToLcmSystem;
using drake::systems::lcm::LcmPublisherSystem;

static double getUnixTime(void)
{
    struct timespec tv;

    if(clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

    return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}

std::unique_ptr<RigidBodyTreed> BuildTestTree() {
  std::unique_ptr<RigidBodyTreed> tree = std::make_unique<RigidBodyTreed>();

  // Add the gripper.  Offset it slightly back and up so that we can
  // locate the box at the origin.
  auto gripper_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "gripper_pose_frame",
      &tree->world(), Eigen::Vector3d(0, -0.065, 0.05),
      Eigen::Vector3d::Zero());
  parsers::urdf::AddModelInstanceFromUrdfFile(
      GetDrakePath() + "/examples/contact_model/soft_hand_box_twofingers.urdf",
      multibody::joints::kFixed, gripper_frame, tree.get());

//  // Add a box to grip.  Position it such that if there *were* a plane at z = 0,
//  // the box would be sitting on it (this maintains parity with the
//  // schunk_wsg_lift_test scenario).
//  auto box_frame = std::allocate_shared<RigidBodyFrame<double>>(
//      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "box_offset",
//      &tree->world(), Eigen::Vector3d(0, 0, 0.075), Eigen::Vector3d::Zero());
//  parsers::urdf::AddModelInstanceFromUrdfFile(
//      GetDrakePath() + "/multibody/models/box_small.urdf",
//      multibody::joints::kQuaternion, box_frame, tree.get());

  return tree;
}

void publishLine(const std::string json_str, std::vector<std::string> path, drake::lcm::DrakeLcm& lcm) {
  long long int now = getUnixTime() * 1000 * 1000;
  auto msg = lcmt_viewer2_comms();
  msg.utime = now;
  msg.format = "treeviewer_json";
  msg.format_version_major = 1;
  msg.format_version_minor = 0;
  msg.data.clear();
  for (auto& c : json_str) {
    msg.data.push_back(c);
    // std::cout << c;
  }
  // std::cout << std::endl << std::endl << std::endl;
  msg.num_bytes = json_str.size();
  // Use channel 0 for remote viewer communications.
  lcm.get_lcm_instance()->publish("DIRECTOR_TREE_VIEWER_REQUEST_<0>", &msg);
}

int main() {
  systems::DiagramBuilder<double> builder;

//  systems::RigidBodyPlant<double>* plant =
//      builder.AddSystem<systems::RigidBodyPlant<double>>(BuildTestTree());

  // Make the RBT
  auto tree = BuildTestTree();

  // Find the actuators we want to control
  int finger1_spring_body_id = -1; //(tree.get())->FindBodyIndex("finger1_tensioner"); // (tree.get())->GetActuator("finger1_tensioner_actuator").body_->get_body_index();
  int finger2_spring_body_id = -1; //(tree.get())->FindBodyIndex("finger2_tensioner");//(tree.get())->GetActuator("finger2_tensioner_actuator").body_->get_body_index();
  
  if ((tree.get())->constraint_cables.size() > 0) { // if constraints

    // Find frame id for some named links
    std::cout << "finger1_paddle -> " << (tree.get())->FindBodyIndex("finger1_paddle") << std::endl;
    std::cout << "finger2_paddle -> " << (tree.get())->FindBodyIndex("finger2_paddle") << std::endl;
    std::cout << "q_size -> " << (tree.get())->get_num_positions() << std::endl;


    for (int i=0; i< (tree.get())->get_num_positions() + (tree.get())->get_num_velocities(); ++i) {
      std::cout << "tree state " << i << ": " << (tree.get())->getStateName(i);
      if ((tree.get())->getStateName(i) == "finger1_tensioner") {
        finger1_spring_body_id = i;
        std::cout << " <-- matched f1t!";
      }
      if ((tree.get())->getStateName(i) == "finger2_tensioner") {
        finger2_spring_body_id = i;
        std::cout << " <-- matched f2t!";
      }
      std::cout << std::endl;
    }

    // Get body indices for the two tensioners
    // int f1_index = (tree.get())->FindBodyIndex("finger1_paddle");
    // int f2_index = (tree.get())->FindBodyIndex("finger2_paddle");

    // Get a KinematicsCache ready
  //  KinematicsCache<double> cache = (tree.get())->CreateKinematicsCache();
  //  (tree.get())->doKinematics(cache, false);
    Eigen::VectorXd q((tree.get())->get_num_positions());
    q.setZero();
    q(0) = 0.0;
    q(1) = 0.0;
    q(2) = 0.0;
    q(3) = 0.0;
    q(4) = 0.0;
    q(5) = 0.0;
    q(6) = 0.0;
    q(7) = 0.0;
    q(8) = 0.0;
    q(9) = 0.0;
    q(10) = 0.0;

    std::cout << "q.transpose(): " << std::endl << q.transpose() << std::endl;

    Eigen::VectorXd v((tree.get())->get_num_velocities());
    v.setZero();
    for (int i=0; i < (tree.get())->get_num_velocities(); i++) {
      v(i) = (1 + i * 53) % 7;
    }

    std::cout << "v.transpose(): " << std::endl << v.transpose() << std::endl;

    // Test some dynamic constraints stuff out on the tree.
    // for (int i=0; i<=11; i++) {
    //   std::cout << std::endl << i << std::endl;
    //   std::cout << (tree.get())->transformPoints(cache, Eigen::Vector3d::Zero(),
    //                                       i,
    //                                       0) << std::endl;
    // }

    KinematicsCache<double> cache = (tree.get())->doKinematics(q, v, true);

    // This is a cable stretching from one tensioner to the other, then in an L-shape to the left thingy
    double cable_length = 6.0;
    std::vector<int> pulley_frames;
    std::vector<std::string> pulley_link_names;
    std::vector<Eigen::Vector3d> pulley_xyz_offsets;
    std::vector<Eigen::Vector3d> pulley_axes;
    std::vector<double> pulley_radii;
    std::vector<int> pulley_num_wraps;
    std::vector<int> pulley_num_faces;

    std::cout << "Sin of pi is: " << std::sin(3.14159) << std::endl;

    pulley_link_names.push_back("finger1_paddle");
    pulley_link_names.push_back("finger2_paddle");
    pulley_link_names.push_back("finger2_paddle");

    pulley_frames.push_back((tree.get())->FindBodyIndex("finger1_paddle"));
    pulley_frames.push_back((tree.get())->FindBodyIndex("finger2_paddle"));
    pulley_frames.push_back((tree.get())->FindBodyIndex("finger2_paddle"));

    pulley_axes.push_back(Eigen::Vector3d::Zero());
    pulley_axes.push_back(Eigen::Vector3d::Zero());
    pulley_axes.push_back(Eigen::Vector3d::Zero());

    Eigen::Vector3d xyz_off_1;
    xyz_off_1.setZero();
    xyz_off_1(0) = -2.0;
    xyz_off_1(1) = 0.0;
    xyz_off_1(2) = 0.0;
    pulley_xyz_offsets.push_back(Eigen::Vector3d::Zero()); // finger1 paddle
    pulley_xyz_offsets.push_back(Eigen::Vector3d::Zero());  // finger2 paddle
    pulley_xyz_offsets.push_back(xyz_off_1);  // finger2 paddle + [2,0,0]'

  //  Teehee: 
  //  5.94333
  //  11.1912
  //     0.05
  //  Teehee: 
  //  5.94333
  //  7.19118
  //     0.05
  //  Teehee: 
  //  7.19177
  //  4.46329
  //     0.05


    pulley_radii.push_back(0.0);
    pulley_radii.push_back(0.0);
    pulley_radii.push_back(0.0);

    pulley_num_wraps.push_back(0);
    pulley_num_wraps.push_back(0);
    pulley_num_wraps.push_back(0);

    pulley_num_faces.push_back(0);
    pulley_num_faces.push_back(0);
    pulley_num_faces.push_back(0);

    CableDynamicConstraint<double> cableConstraint(tree.get(), cable_length, pulley_link_names, pulley_xyz_offsets, pulley_axes, pulley_radii, pulley_num_wraps, pulley_num_faces);

    std::cout << "cable stretching from one tensioner to the other. Should have length 4+2 in nominal pose: " << std::endl;
    std::cout << "  cableConstraint.getNumPositionConstraints() -> " << cableConstraint.getNumPositionConstraints() << std::endl;
    std::cout << "  cableConstraint.positionConstraints(cache) -> " << cableConstraint.positionConstraints(cache) << std::endl;
    std::cout << "  cableConstraint.positionConstraintsJacobian(cache,false) -> " << cableConstraint.positionConstraintsJacobian(cache, false) << std::endl;
    std::cout << "  cableConstraint.positionConstraintsJacobian(cache,true)  -> " << cableConstraint.positionConstraintsJacobian(cache, true) << std::endl;
    std::cout << "  cableConstraint.positionConstraintsJacDotTimesV(cache)   -> " << cableConstraint.positionConstraintsJacDotTimesV(cache) << std::endl;

    // Empirically verify gradients

    // Verify Jacobian
    auto phi_q = cableConstraint.positionConstraints(cache);
    
    std::cout << "Empirical gradients: " << std::endl;
    double h=.00001;
    for (int i=0; i<11; i++) {
      q(i) += h;
      cache = (tree.get())->doKinematics(q);
      auto phi_q_di = cableConstraint.positionConstraints(cache);

      std::cout << (phi_q_di - phi_q) / h << std::endl;

      q(i) -= h;
    }
    
    // Verify phi-double-dot
    cache = (tree.get())->doKinematics(q);
    auto phi_q_0 = cableConstraint.positionConstraints(cache);
    q += v*h;
    cache = (tree.get())->doKinematics(q);
    auto phi_q_h = cableConstraint.positionConstraints(cache);
    q += v*h;
    cache = (tree.get())->doKinematics(q);
    auto phi_q_2h = cableConstraint.positionConstraints(cache);

    std::cout << "Empirical phi-double-dot: " << ((phi_q_2h - 2*phi_q_h + phi_q_0)/h/h) << std::endl;    

    q -= 2*v*h;

    cache = (tree.get())->doKinematics(q, v, true);
    std::cout << "q.transpose(): " << std::endl << q.transpose() << std::endl;
    std::cout << "v.transpose(): " << std::endl << v.transpose() << std::endl;
    std::cout << "cable stretching from one tensioner to the other. Should have length 4+2 in nominal pose: " << std::endl;
    std::cout << "  (tree.get())->constraint_cables[0].getNumPositionConstraints() -> " << (tree.get())->constraint_cables[0].getNumPositionConstraints() << std::endl;
    std::cout << "  (tree.get())->constraint_cables[0].positionConstraints(cache) -> " << (tree.get())->constraint_cables[0].positionConstraints(cache) << std::endl;
    std::cout << "  (tree.get())->constraint_cables[0].positionConstraintsJacobian(cache,false) -> " << (tree.get())->constraint_cables[0].positionConstraintsJacobian(cache, false) << std::endl;
    std::cout << "  (tree.get())->constraint_cables[0].positionConstraintsJacobian(cache,true)  -> " << (tree.get())->constraint_cables[0].positionConstraintsJacobian(cache, true) << std::endl;
    std::cout << "  (tree.get())->constraint_cables[0].positionConstraintsJacDotTimesV(cache)   -> " << (tree.get())->constraint_cables[0].positionConstraintsJacDotTimesV(cache) << std::endl;

    std::cout << "cable stretching from one tensioner to the other. Should have length 4+2 in nominal pose: " << std::endl;
    std::cout << "  (tree.get())->getNumPositionConstraints() -> " << (tree.get())->getNumPositionConstraints() << std::endl;
    std::cout << "  (tree.get())->positionConstraints(cache) -> " << (tree.get())->positionConstraints(cache) << std::endl;
    std::cout << "  (tree.get())->positionConstraintsJacobian(cache,false) -> " << (tree.get())->positionConstraintsJacobian(cache, false) << std::endl;
    std::cout << "  (tree.get())->positionConstraintsJacobian(cache,true)  -> " << (tree.get())->positionConstraintsJacobian(cache, true) << std::endl;
    std::cout << "  (tree.get())->positionConstraintsJacDotTimesV(cache)   -> " << (tree.get())->positionConstraintsJacDotTimesV(cache) << std::endl;

  } // endif cables

  std::cout << "Finger 1 at " << finger1_spring_body_id << std::endl;
  std::cout << "Finger 2 at " << finger2_spring_body_id << std::endl;

  DRAKE_DEMAND(finger1_spring_body_id != -1);
  DRAKE_DEMAND(finger2_spring_body_id != -1);

  // Alright, let's get to simulating
  systems::RigidBodyPlant<double>* plant =
      builder.AddSystem<systems::RigidBodyPlant<double>>( std::move(tree) );

  // Command-line specified contact parameters.
  std::cout << "Contact properties:\n";
  std::cout << "\tStiffness:                " << FLAGS_stiffness << "\n";
  std::cout << "\tstatic friction:          " << FLAGS_us << "\n";
  std::cout << "\tdynamic friction:         " << FLAGS_ud << "\n";
  std::cout << "\tAllowed stiction speed:   " << FLAGS_v_stiction_tolerance
            << "\n";
  std::cout << "\tDissipation:              " << FLAGS_dissipation << "\n";
  plant->set_normal_contact_parameters(FLAGS_stiffness, FLAGS_dissipation);
  plant->set_friction_contact_parameters(FLAGS_us, FLAGS_ud,
                                         FLAGS_v_stiction_tolerance);

  // Creates and adds LCM publisher for visualization.  The test doesn't
  // require `drake_visualizer` but it is convenient to have when debugging.
  drake::lcm::DrakeLcm lcm;
  const auto viz_publisher =
      builder.template AddSystem<systems::DrakeVisualizer>(
          plant->get_rigid_body_tree(), &lcm, true);
  builder.Connect(plant->state_output_port(),
                  viz_publisher->get_input_port(0));


  std::cout << "Plant Port Descriptors:" << std::endl;
  // std::cout << plant->get_input_port(0) << std::endl;
  std::cout << plant->get_num_input_ports() << std::endl;
  std::cout << plant->get_num_output_ports() << std::endl;

  // (geronm) Create an input vector to set command torques
  int num_control_actuators = 2;
  VectorX<double> source_value(num_control_actuators);
  source_value(0) = FLAGS_fing1_force;
  source_value(1) = FLAGS_fing2_force;
  const auto force_input =
      builder.template AddSystem<systems::ConstantVectorSource<double>>(source_value);
  // builder.Connect(force_input->get_output_port(), plant->actuator_command_input_port());

  // (geronm) Create a controller to apply spring law to system springs
  double finger_spring_constant = FLAGS_fing_spring;
  int num_spring_actuators = 2;
  const auto spring_forces1 =
      builder.template AddSystem<systems::Gain<double>>(-finger_spring_constant, 1);
  const auto spring_forces2 =
      builder.template AddSystem<systems::Gain<double>>(-finger_spring_constant, 1);
  const auto state_outputs_split =
      builder.template AddSystem<systems::Demultiplexer<double>>(plant->state_output_port().size());
  std::vector<int> spring_input_joiner;
  for (int i=0; i<num_spring_actuators; ++i) {
    spring_input_joiner.push_back(1);
  }
  const auto spring_command_vector =
      builder.template AddSystem<systems::Multiplexer<double>>(spring_input_joiner);
  // VectorX<double> spring_dummy_forces(num_spring_actuators);
  // spring_dummy_forces(0) = 5.0;
  // spring_dummy_forces(1) = 5.0;
  // const auto spring_forces =  // TODO(geronm) replace this temporary constant system with the proper gain on spring state locations
  //     builder.template AddSystem<systems::ConstantVectorSource<double>>(spring_dummy_forces);

  // (geronm) Create a multiplexer to concatenate the control inputs and sprint inputs
  std::vector<int> input_sizes;
  input_sizes.push_back(num_control_actuators);
  input_sizes.push_back(num_spring_actuators);
  // systems:Multiplexer<double>> my_multi(std::vector<int>{2,2});
  // std::cout << my_multi.get_num_output_ports() << std::endl;
  const auto input_multiplexer =
      builder.template AddSystem<systems::Multiplexer<double>>(input_sizes);

  builder.Connect(force_input->get_output_port(), input_multiplexer->get_input_port(0));

  // std::cout << "builder.Connect(force_input->get_output_port(), input_multiplexer->get_input_port(0));" << std::endl;

  builder.Connect(plant->state_output_port(), state_outputs_split->get_input_port(0));

  // std::cout << "builder.Connect(plant->state_output_port(), state_outputs_split->get_input_port(0));" << std::endl;

  builder.Connect(state_outputs_split->get_output_port(finger1_spring_body_id), spring_forces1->get_input_port());
  builder.Connect(spring_forces1->get_output_port(), spring_command_vector->get_input_port(0));

  // std::cout << "builder.Connect(state_outputs_split->get_output_port(finger1_spring_body_id), spring_command_vector->get_input_port(0));" << std::endl;

  std::cout << "Stuff:" << std::endl;
  std::cout << state_outputs_split->get_num_output_ports() << std::endl;
  std::cout << plant->state_output_port().size() << std::endl;
  // std::cout << state_outputs_split.get_num_output_ports() << std::endl;
  // std::cout << state_outputs_split.get_num_output_ports() << std::endl;

  builder.Connect(state_outputs_split->get_output_port(finger2_spring_body_id), spring_forces2->get_input_port());
  builder.Connect(spring_forces2->get_output_port(), spring_command_vector->get_input_port(1));

  // std::cout << "builder.Connect(state_outputs_split->get_output_port(finger2_spring_body_id), spring_command_vector->get_input_port(1));" << std::endl;

//  // builder.Connect(spring_command_vector->get_output_port(0), spring_forces->get_input_port());

  // std::cout << "builder.Connect(spring_command_vector->get_output_port(0), spring_forces->get_input_port());" << std::endl;

//  // builder.Connect(spring_forces->get_output_port(), input_multiplexer->get_input_port(1));

  builder.Connect(spring_command_vector->get_output_port(0), input_multiplexer->get_input_port(1));

  // std::cout << "builder.Connect(spring_forces->get_output_port(), input_multiplexer->get_input_port(1));" << std::endl;

  // std::cout << " " << input_multiplexer->get_output_port(0).size() << " " << plant->actuator_command_input_port().size() << std::endl;
  builder.Connect(input_multiplexer->get_output_port(0), plant->actuator_command_input_port());

  // std::cout << "builder.Connect(input_multiplexer->get_output_port(0), plant->actuator_command_input_port());" << std::endl;



  // std::cout << "Spring num inputs: " << spring_forces->get_num_input_ports() << "   num_outputs: " << spring_forces->get_num_output_ports() << std::endl;
  std::cout << "Control num inputs: " << force_input->get_num_input_ports() << "   num_outputs: " << force_input->get_num_output_ports() << std::endl;
  std::cout << "Multiplexer num inputs: " << input_multiplexer->get_num_input_ports() << "   num_outputs: " << input_multiplexer->get_num_output_ports() << std::endl;

  // Enable contact force visualization.
//  const ContactResultsToLcmSystem<double>& contact_viz =
//  *builder.template AddSystem<ContactResultsToLcmSystem<double>>(
//      plant->get_rigid_body_tree());
//  auto& contact_results_publisher = *builder.AddSystem(
//      LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
//          "CONTACT_RESULTS", &lcm));
//  // Contact results to lcm msg.
//  builder.Connect(plant->contact_results_output_port(),
//                  contact_viz.get_input_port(0));
//  builder.Connect(contact_viz.get_output_port(0),
//                  contact_results_publisher.get_input_port(0));

  // Set up the model and simulator and set their starting state.
  const std::unique_ptr<systems::Diagram<double>> model = builder.Build();
  systems::Simulator<double> simulator(*model);

  auto context = simulator.get_mutable_context();

  simulator.reset_integrator<RungeKutta3Integrator<double>>(*model, context);
  simulator.get_mutable_integrator()->request_initial_step_size_target(1e-4);
  simulator.get_mutable_integrator()->set_target_accuracy(FLAGS_accuracy);
  std::cout << "Variable-step integrator accuracy: " << FLAGS_accuracy << "\n";

  simulator.Initialize();

  // Print a time stamp update every tenth of a second.  This helps communicate
  // progress in the event that the integrator crawls to a very small timestep.
  const double kPrintPeriod = 1.0;
  int step_count =
      static_cast<int>(std::ceil(FLAGS_sim_duration / kPrintPeriod));
  for (int i = 1; i <= step_count; ++i) {
    double t = context->get_time();
    std::cout << "time: " << t << "\n";
    simulator.StepTo(i * kPrintPeriod);
    
    // std::cout << "state: " << std::endl << std::endl;
    // std::cout << (simulator.get_context()).get_continuous_state()->CopyToVector();
    // std::cout << std::endl << std::endl;
  }

  std::string json_str = "{\"delete\":{},\"setgeometry\":[{\"geometry\":{\"points\":[[1.86657557780192,-0.464247709762942,0.747660221127523],[-1.53370505391172,1.3916316561968,0.504459205256841],[-1.73012210768299,1.39219064159683,-0.427621228005646],[2.73162407272025,-0.31940360668027,0.381326330271418]],\"type\":\"line\"},\"path\":[\"test_line\"]}],\"settransform\":{},\"timestamp\":1492025357657361}";
  std::vector<std::string> path_vec;
  path_vec.push_back("remote_tree_seer");
  path_vec.push_back("cable");
//  publishLine(json_str, path_vec, lcm);

  std::cout << "Simulating...." << std::endl;

  while (FLAGS_playback) viz_publisher->ReplayCachedSimulation();
  return 0;
}

}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::main();
}
