#include "drake/examples/QPSolve/qp_solve.h"

//namespace drake {
//namespace examples {
//namespace qp_solve {

int main(int argc, char* argv[]) {
  using namespace drake;

  std::unique_ptr<drake::solvers::MathematicalProgram> prog;
  drake::solvers::GurobiSolver solver;
  drake::solvers::VectorXDecisionVariable basis;
  drake::solvers::VectorXDecisionVariable x_decision;
  
  Eigen::VectorXd solution;

  std::cout << "`Hello, World!`, in Drake!" << std::endl;
  

  prog.reset(new solvers::MathematicalProgram());
  x_decision = prog->NewContinuousVariables(2, "x");

  // Some constraints
  Eigen::MatrixXd a_mat(2,2);
  Eigen::VectorXd b_mat(2,1);

  a_mat(0,0) = 1;
  a_mat(0,1) = 1;
  a_mat(1,0) = -1;
  a_mat(1,1) = -1;

  b_mat(0,0) = 1;
  b_mat(1,0) = -1;

  prog->AddLinearEqualityConstraint(a_mat,b_mat,x_decision);

  // Quadratic constraint
  // drake::solvers::QuadraticConstraint* cost;

//  
//  eq_dynamics_ =
//    prog_
//        ->AddLinearEqualityConstraint(
//            MatrixX<double>::Zero(1, 2), // num_eqns, num_vars
//            VectorX<double>::Zero(1), {p_})
//        .get();
//
//  eq_dynamics_->set_description("dynamics eq");
  
  // Quadratic cost
  Eigen::MatrixXd q_mat(2,2);
  Eigen::VectorXd b_vec(2);

  // b = [0, 0]'
  b_vec.setZero();

  // Q = [1,0; 0,1]
  q_mat.setZero();
  q_mat(0,0) = 1;
  q_mat(1,1) = 1;

  prog->AddQuadraticCost(q_mat, b_vec, x_decision).get();
  
  // Solve!    
  solvers::SolutionResult result = solver.Solve(*(prog.get()));
  if (result != solvers::SolutionResult::kSolutionFound) {
    std::cerr << "solution not found\n";
    return -1;
  }
  solution = prog->GetSolutionVectorValues();
  
  // Inspect dat stuff
  std::cout << "Solution:" << std::endl << solution << std::endl;

  return 0;
  
}

//}  // namespace qp_solve
//}  // namespace examples
//}  // namespace drake
