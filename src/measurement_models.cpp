#include "measurement_models.hpp"

static const MatrixXd I = MatrixXd::Identity(4, 4);

VectorXd CartesianToPolar(const VectorXd& x) {
  double px = x[0], py = x[1], vx = x[2], vy = x[3];
  VectorXd res(3);
  
  double rho = sqrt(px*px + py*py);
  double phi = atan2(py, px);
  double rhodot = (px*vx + py*vy) / rho;
  
  res << rho, phi, rhodot;
  return res;
}

double NormalizeAngle(double angle) {
  if (angle > M_PI) {
    angle = angle - 2 * M_PI;
  } else if (angle < -M_PI) {
    angle = angle + 2 * M_PI;
  };
  return angle;
}

MatrixXd CalculateJacobian(const VectorXd& x) {
  double px = x[0], py = x[1], vx = x[2], vy = x[3];

  double c1 = px * px + py * py;
  double c2 = sqrt(c1);
  double c3 = (c1 * c2);

  if(fabs(c1) < 0.0001) {
    return MatrixXd();
  } else {
    MatrixXd J(3, 4);
    J <<
      (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

    return J;
  }
}

LaserMeasurementModel::LaserMeasurementModel(): R(2, 2), H(2, 4) {
  R <<
    0.0225, 0,
    0, 0.0225;
  H <<
    1, 0, 0, 0,
    0, 1, 0, 0;
}

void LaserMeasurementModel::InitState(const VectorXd& measurement, VectorXd& state) const {
  state << measurement[0], measurement[1], 0, 0;
}

void LaserMeasurementModel::UpdateState(const VectorXd& measurement, VectorXd& state, MatrixXd& cov) const {
  VectorXd y = measurement - H * state;
  MatrixXd S = H * cov * H.transpose() + R;
  MatrixXd K = cov * H.transpose() * S.inverse();
    
  state = state + K * y;
  cov = (I - K * H) * cov;
}

RadarMeasurementModel::RadarMeasurementModel(): R(3, 3) {
  R <<
    0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09;
}

void RadarMeasurementModel::InitState(const VectorXd& measurement, VectorXd& state) const {
  double rho = measurement[0], phi = measurement[1], rhodot = measurement[2];
  state <<
    rho * cos(phi),
    rho * sin(phi),
    rhodot * cos(phi),
    rhodot * sin(phi);
}

void RadarMeasurementModel::UpdateState(const VectorXd& measurement, VectorXd& state, MatrixXd& cov) const {
  MatrixXd H = CalculateJacobian(state);

  if (H.size() > 0) {
    VectorXd y = measurement - CartesianToPolar(state);
    y[1] = NormalizeAngle(y[1]);
    
    MatrixXd S = H * cov * H.transpose() + R;
    MatrixXd K = cov * H.transpose() * S.inverse();
      
    state = state + K * y;
    cov = (I - K * H) * cov;
  }
}


  

