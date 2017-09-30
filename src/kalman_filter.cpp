#include "kalman_filter.h"
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

static MatrixXd I = MatrixXd::Identity(4, 4);

VectorXd h(const VectorXd& x) {
  double px = x[0], py = x[1], vx = x[2], vy = x[3];
  VectorXd res(3);
  
  double rho = sqrt(px*px + py*py);
  double phi = atan2(py, px);
  double rhodot = (px*vx + py*vy) / rho;
  
  res << rho, phi, rhodot;
  return res;
}

double NormAngle(double angle) {
  if (angle > M_PI) {
    angle = angle - 2 * M_PI;
  } else if (angle < -M_PI) {
    angle = angle + 2 * M_PI;
  };
  return angle;
}



KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  H_ = H_in;
  R_ = R_in;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd h_calc = h(x_);
  VectorXd y = z - h_calc;
  y[1] = NormAngle(y[1]);
    
  cout << "Z" << ";" << z[0] << ";" << z[1] << ";" << z[2] << endl;
  cout << "H" << ";" << h_calc[0] << ";" << h_calc[1] << ";" << h_calc[2] << endl;
  cout << "Y" << ";" << y[0] << ";" << y[1] << ";" << y[2] << endl;
  
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}
