#include "drake/examples/contact_model/qp_control/qp_controller.h"

namespace drake {
namespace examples {

QPController::QPController(const double k_1, const double k_2) : k_1(k_1), k_2(k_2) {
  // pass
}

void QPController::getQManipAndJac(Eigen::VectorXd p_ij, Eigen::VectorXd & q_manip_out, Eigen::MatrixXd & jac_out) {

  // Dummy implementation: instead of solving a QP, we set q_manip equal to the average
  // of the four finger inputs

  getQManip(p_ij, q_manip_out);

  Eigen::VectorXd q_manip_test(2);


  // Clear jacobian
  jac_out.setZero();

  {
    // Take forward numerical Jacobian dqmanip / dpij
    double h = 0.5;
    // VectorXd h(p_ij.size());
    // h.setZero();
    // h(0) = 1.0;
    // h(1) = 1.0;
    // h(2) = 1.0;
    // h(3) = 1.0;
    // h(4) = 1.0;
    // h(5) = 1.0;
    // h(6) = 1.0;
    // h(7) = 1.0;
    const double kBound = 100.0;
    for (size_t i=0; i<(size_t)p_ij.size(); i++) {
      p_ij(i) += h;

      getQManip(p_ij, q_manip_test);

      jac_out(0, i) += 0.5 * std::max(std::min((q_manip_test(0) - q_manip_out(0)) / h, kBound), -kBound);
      jac_out(1, i) += 0.5 * std::max(std::min((q_manip_test(1) - q_manip_out(1)) / h, kBound), -kBound);

      p_ij(i) -= h;
    }
  }

  {
    // Take reverse numerical Jacobian dqmanip / dpij
    double h = -0.5;
    // VectorXd h(p_ij.size());
    // h.setZero();
    // h(0) = 1.0;
    // h(1) = 1.0;
    // h(2) = 1.0;
    // h(3) = 1.0;
    // h(4) = 1.0;
    // h(5) = 1.0;
    // h(6) = 1.0;
    // h(7) = 1.0;
    const double kBound = 100.0;
    for (size_t i=0; i<(size_t)p_ij.size(); i++) {
      p_ij(i) += h;

      getQManip(p_ij, q_manip_test);

      jac_out(0, i) += 0.5 * std::max(std::min((q_manip_test(0) - q_manip_out(0)) / h, kBound), -kBound);
      jac_out(1, i) += 0.5 * std::max(std::min((q_manip_test(1) - q_manip_out(1)) / h, kBound), -kBound);

      p_ij(i) -= h;
    }
  }

  // jac_out(0, 2*0 + 0) = 0.25;
  // jac_out(0, 2*0 + 1) = 0.00;
  // jac_out(1, 2*0 + 0) = 0.00;
  // jac_out(1, 2*0 + 1) = 0.25;
  // jac_out(0, 2*1 + 0) = 0.25;
  // jac_out(0, 2*1 + 1) = 0.00;
  // jac_out(1, 2*1 + 0) = 0.00;
  // jac_out(1, 2*1 + 1) = 0.25;
  // jac_out(0, 2*2 + 0) = 0.25;
  // jac_out(0, 2*2 + 1) = 0.00;
  // jac_out(1, 2*2 + 0) = 0.00;
  // jac_out(1, 2*2 + 1) = 0.25;
  // jac_out(0, 2*3 + 0) = 0.25;
  // jac_out(0, 2*3 + 1) = 0.00;
  // jac_out(1, 2*3 + 0) = 0.00;
  // jac_out(1, 2*3 + 1) = 0.25;

  return;




  // Dummy Jacobian
  // jac_out(0, 2*0 + 0) = 0.25;
  // jac_out(0, 2*0 + 1) = 0.00;
  // jac_out(1, 2*0 + 0) = 0.00;
  // jac_out(1, 2*0 + 1) = 0.25;
  // jac_out(0, 2*1 + 0) = 0.25;
  // jac_out(0, 2*1 + 1) = 0.00;
  // jac_out(1, 2*1 + 0) = 0.00;
  // jac_out(1, 2*1 + 1) = 0.25;
  // jac_out(0, 2*2 + 0) = 0.25;
  // jac_out(0, 2*2 + 1) = 0.00;
  // jac_out(1, 2*2 + 0) = 0.00;
  // jac_out(1, 2*2 + 1) = 0.25;
  // jac_out(0, 2*3 + 0) = 0.25;
  // jac_out(0, 2*3 + 1) = 0.00;
  // jac_out(1, 2*3 + 0) = 0.00;
  // jac_out(1, 2*3 + 1) = 0.25;  

 // done
}

void QPController::getQManip(Eigen::VectorXd p_ij, Eigen::VectorXd & q_manip_out) {
  // q_manip_out(0) = .25 * (p_ij(2*0 + 0) + p_ij(2*1 + 0) + p_ij(2*2 + 0) + p_ij(2*3 + 0));
  // q_manip_out(1) = .25 * (p_ij(2*0 + 1) + p_ij(2*1 + 1) + p_ij(2*2 + 1) + p_ij(2*3 + 1));

    // These two vectors are constants which will shape
  // the solution
  Eigen::VectorXd pa1_vec(2);
  Eigen::VectorXd pa2_vec(2);
  Eigen::VectorXd pb1_vec(2);
  Eigen::VectorXd pb2_vec(2);

  // choose some values
  pa1_vec(0) = p_ij(0);
  pa1_vec(1) = p_ij(1);
  pa2_vec(0) = p_ij(2);
  pa2_vec(1) = p_ij(3);

  pb1_vec(0) = p_ij(4);
  pb1_vec(1) = p_ij(5);
  pb2_vec(0) = p_ij(6);
  pb2_vec(1) = p_ij(7);

  double la = (pa1_vec - pa2_vec).norm();
  double lb = (pb1_vec - pb2_vec).norm();


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
  
  // Quadratic cost: (la1 + la2 - La)^2 + (lb1 + lb2 - Lb)^2
  // Quadratic cost: la1^2 + 2la1la2 + la2^2 - La*la1 - La*la2 + La
  //                     + lb1^2 + 2lb1lb2 + lb2^2 - Lb*lb1 - Lb*lb2 + Lb
  Eigen::MatrixXd q_mat(6,6);
  Eigen::VectorXd b_vec(6);

  // b = 0s
  b_vec.setZero();
  b_vec(2) = -la;
  b_vec(3) = -la;
  b_vec(4) = -lb;
  b_vec(5) = -lb;

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
    return;
  }
  solution = prog->GetSolutionVectorValues();
  
  // Inspect dat stuff
  std::cout << "Solution:" << std::endl << solution << std::endl;

  q_manip_out(0) = solution(0);
  q_manip_out(1) = solution(1);
}


}  // namespace examples
}  // namespace drake
