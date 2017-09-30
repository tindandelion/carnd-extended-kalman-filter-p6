#ifndef __MOTION_MODEL_H
#define __MOTION_MODEL_H

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;


class MotionModel {
private:
  MatrixXd acc_noise_cov;

  void CalculateProcessNoise(double time_delta) {
    MatrixXd G = MatrixXd(4, 2);
    G <<
      time_delta * time_delta / 2, 0,
      0, time_delta * time_delta / 2,
      time_delta, 0,
      0, time_delta;
    
    Q = G * acc_noise_cov * G.transpose();
  }

  void CalculateProcessChange(double time_delta) {
    F <<
      1, 0, time_delta, 0,
      0, 1, 0, time_delta,
      0, 0, 1, 0,
      0, 0, 0, 1;
  }
  
public:
  VectorXd x;
  MatrixXd P;
  MatrixXd F;
  MatrixXd Q;
  MotionModel(double acc_noise_variance):
    x(VectorXd::Zero(4)),
    P(MatrixXd::Zero(4, 4)),
    F(MatrixXd::Zero(4, 4)),
    Q(MatrixXd::Zero(4, 4)),
    acc_noise_cov(MatrixXd(2 ,2)) {
    acc_noise_cov <<
      acc_noise_variance, 0,
      0, acc_noise_variance;
  }

  void Init(const VectorXd& x0) {
    x = x0;
    P <<
      1000, 0, 0, 0,
      0, 1000, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;
  }

  void Predict(double time_delta) {
    CalculateProcessNoise(time_delta);
    CalculateProcessChange(time_delta);
  }
  
};


#endif
