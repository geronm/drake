#include "drake/examples/QPSolve/qp_solve.h"

//namespace drake {
//namespace examples {
//namespace qp_solve {

int main(int argc, char* argv[]) {
  using namespace drake;

  // These two vectors are constants which will shape
  // the solution
  Eigen::VectorXd pa1_vec(2);
  Eigen::VectorXd pa2_vec(2);
  Eigen::VectorXd pb1_vec(2);
  Eigen::VectorXd pb2_vec(2);

  // choose some values
  pa1_vec(0) = -1.0;
  pa1_vec(1) =  2.0;
  pa2_vec(0) =  1.0;
  pa2_vec(1) =  4.0;

  pb1_vec(0) = -1.0;
  pb1_vec(1) =  -2.0;
  pb2_vec(0) =  1.0;
  pb2_vec(1) =  -4.0;


  std::cout << "`Hello, World!`, in Drake!" << std::endl;
  

  // Solver boilerplate
  std::unique_ptr<drake::solvers::MathematicalProgram> prog;
  drake::solvers::GurobiSolver solver;
  drake::solvers::VectorXDecisionVariable basis;
  drake::solvers::VectorXDecisionVariable x_decision;
  // std::shared_ptr<drake::solvers::QuadraticConstraint> qca1;
  Eigen::VectorXd solution;

  prog.reset(new solvers::MathematicalProgram());
  
  // Decision variable x: [px py la1 la2 lb1 lb2]
  x_decision = prog->NewContinuousVariables(6, "x");
  
  // Quadratic cost: (la1 + la2)^2 + (lb1 + lb2)^2
  Eigen::MatrixXd q_mat(6,6);
  Eigen::VectorXd b_vec(6);

  // b = 0s
  b_vec.setZero();

  // Q = 
  //   [[0,0,0,0,0,0]
  //    [0,0,0,0,0,0]
  //    [0,0,1,1,0,0]
  //    [0,0,1,1,0,0]
  //    [0,0,0,0,1,1]
  //    [0,0,0,0,1,1]]
  q_mat.setZero();
  q_mat(2,2) = 1;
  q_mat(2,3) = 1;
  q_mat(3,2) = 1;
  q_mat(3,3) = 1;
  q_mat(4,4) = 1;
  q_mat(4,5) = 1;
  q_mat(5,4) = 1;
  q_mat(5,5) = 1;

  prog->AddQuadraticCost(q_mat, b_vec, x_decision);

  // Some lorentz cone constraints:
  
  //  la1^2 >= (p - pa1)^2
  Eigen::MatrixXd aca1_mat(3,6);
  Eigen::VectorXd bca1_vec(3);

  aca1_mat.setZero();
  aca1_mat(0,2) = 1;
  aca1_mat(1,0) = 1;
  aca1_mat(2,1) = 1;

  bca1_vec.setZero();
  bca1_vec(0) = 0.0;
  bca1_vec(1) = -pa1_vec(0);
  bca1_vec(2) = -pa1_vec(1);

  prog->AddLorentzConeConstraint(aca1_mat, bca1_vec, x_decision);

  //  la2^2 >= (p - pa2)^2
  Eigen::MatrixXd aca2_mat(3,6);
  Eigen::VectorXd bca2_vec(3);

  aca2_mat.setZero();
  aca2_mat(0,3) = 1;
  aca2_mat(1,0) = 1;
  aca2_mat(2,1) = 1;

  bca2_vec.setZero();
  bca2_vec(0) = 0.0;
  bca2_vec(1) = -pa2_vec(0);
  bca2_vec(2) = -pa2_vec(1);

  prog->AddLorentzConeConstraint(aca2_mat, bca2_vec, x_decision);

  //  lb1^2 >= (p - pb1)^2
  Eigen::MatrixXd acb1_mat(3,6);
  Eigen::VectorXd bcb1_vec(3);

  acb1_mat.setZero();
  acb1_mat(0,4) = 1;
  acb1_mat(1,0) = 1;
  acb1_mat(2,1) = 1;

  bcb1_vec.setZero();
  bcb1_vec(0) = 0.0;
  bcb1_vec(1) = -pb1_vec(0);
  bcb1_vec(2) = -pb1_vec(1);

  prog->AddLorentzConeConstraint(acb1_mat, bcb1_vec, x_decision);

  //  lb2^2 >= (p - pb2)^2
  Eigen::MatrixXd acb2_mat(3,6);
  Eigen::VectorXd bcb2_vec(3);

  acb2_mat.setZero();
  acb2_mat(0,5) = 1;
  acb2_mat(1,0) = 1;
  acb2_mat(2,1) = 1;

  bcb2_vec.setZero();
  bcb2_vec(0) = 0.0;
  bcb2_vec(1) = -pb2_vec(0);
  bcb2_vec(2) = -pb2_vec(1);

  prog->AddLorentzConeConstraint(acb2_mat, bcb2_vec, x_decision);

  
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
 