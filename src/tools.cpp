#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd mse = VectorXd(4);
  mse << 0, 0, 0, 0;
  
  for (int i = 0; i < estimations.size(); i++) {
    VectorXd error = (estimations[i] - ground_truth[i]).array().square();
    mse = mse + error;
  }
  return (mse / estimations.size()).array().sqrt();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}
