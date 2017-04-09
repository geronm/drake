
// Simple Continuous Time System Example
//
// This is meant to be a sort of "hello world" example for the
// drake::system classes.  It defines a very simple continuous time system,
// simulates it from a given initial condition, and plots the result.

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/context.h"
#include "drake/common/eigen_autodiff_types.h"

// Simple Continuous Time System. No input.
//
//  q = [px py]
//  x = [px py pxdot pydot]
//
//  xdot = [pxdot pydot 0 -1 (gravity)]
//
//   (y = py)
class SimpleContinuousTimeSystem : public drake::systems::LeafSystem<double> {
 public:
  SimpleContinuousTimeSystem() {
    const int kSize = 4;  // The dimension of state (x).
    const int kOutSize = 1;  // The dimension of output (y)

    this->DeclareOutputPort(drake::systems::kVectorValued, kOutSize);
    this->DeclareContinuousState(kSize);
  }

  // xdot = [pxdot pydot 0 -1 (gravity)]
  void DoCalcTimeDerivatives(
      const drake::systems::Context<double>& context,
      drake::systems::ContinuousState<double>* derivatives) const override {
    //double px = context.get_continuous_state_vector().GetAtIndex(0);
    //double py = context.get_continuous_state_vector().GetAtIndex(1);
    double pxdot = context.get_continuous_state_vector().GetAtIndex(2);
    double pydot = context.get_continuous_state_vector().GetAtIndex(3);
    
    derivatives->get_mutable_vector()->SetAtIndex(0, pxdot);
    derivatives->get_mutable_vector()->SetAtIndex(1, pydot);
    derivatives->get_mutable_vector()->SetAtIndex(2, 0);
    derivatives->get_mutable_vector()->SetAtIndex(3, -1);
  }

  // y = x
  void DoCalcOutput(
      const drake::systems::Context<double>& context,
      drake::systems::SystemOutput<double>* output) const override {
    double py = context.get_continuous_state_vector().GetAtIndex(1);
    output->GetMutableVectorData(0)->SetAtIndex(0, py);
  }

  
  // constraints. In this case, px = py
  int do_get_num_constraint_equations(const drake::systems::Context<double>& context) const {
    return 1;
  }

  Eigen::VectorXd DoEvalConstraintEquations(
      const drake::systems::Context<double>& context) const {
    //DRAKE_DEMAND(get_num_constraint_equations(context) == 0);
    Eigen::VectorXd q(1);
    double px = context.get_continuous_state_vector().GetAtIndex(0);
    double py = context.get_continuous_state_vector().GetAtIndex(1);
    q(0) = py - px;

    return q;
  }
  Eigen::VectorXd DoEvalConstraintEquationsDot(
        const drake::systems::Context<double>& context) const {
    Eigen::VectorXd q(1);
    double pxdot = context.get_continuous_state_vector().GetAtIndex(2);
    double pydot = context.get_continuous_state_vector().GetAtIndex(3);
    q(0) = pydot - pxdot;

    return q;
  }
//  Eigen::VectorXd
//    DoCalcVelocityChangeFromConstraintImpulses(const Context<T>& context,
//                                               const Eigen::MatrixXd& J,
//                                               const Eigen::VectorXd& lambda)
//                                                   const {
//    DRAKE_DEMAND(get_num_constraint_equations(context) == 0);
//    const auto& gv = context.get_continuous_state()->get_generalized_velocity();
//    return Eigen::VectorXd::Zero(gv.size());
//  }

  /// Computes the norm of the constraint error. This default implementation
  /// computes a Euclidean norm of the error. Derived classes can override this
  /// function, which is called by CalcConstraintErrorNorm(). This norm need be
  /// neither continuous nor differentiable.
  /// @sa CalcConstraintErrorNorm() for parameter documentation.
  //  double DoCalcConstraintErrorNorm(const Context<T>& context,
  //                                          const Eigen::VectorXd& error) const {
  //   return error.norm();
  // }
};


int main() {
  // Create the simple system.
  SimpleContinuousTimeSystem system;

  // Create the simulator.
  drake::systems::Simulator<double> simulator(system);

  // Set the initial conditions x(0).
  drake::systems::ContinuousState<double>& xc =
      *simulator.get_mutable_context()->get_mutable_continuous_state();
  xc[0] = 0;
  xc[0] = 0;
  xc[0] = 0;
  xc[0] = 0;

  std::cout << xc[0] << std::endl;
  std::cout << xc[1] << std::endl << std::endl;

  std::cout << system.EvalConstraintEquations(simulator.get_context()) << std::endl << std::endl;

  // Simulate for 10 seconds.
  simulator.StepTo(10);

  std::cout << xc[0] << std::endl;
  std::cout << xc[1] << std::endl << std::endl;
  
  std::cout << system.EvalConstraintEquations(simulator.get_context()) << std::endl << std::endl;

  std::cout << system.get_num_constraint_equations(simulator.get_context()) << std::endl;

  // std::cout << xc[1] << std::endl;

  // std::cout << xc[1] << std::endl;

  // Make sure the simulation converges to the stable fixed point at x=0.
  // DRAKE_DEMAND(xc[0] < 1.0e-4);

  

  // TODO(russt): make a plot of the resulting trajectory.

  return 0;
}

