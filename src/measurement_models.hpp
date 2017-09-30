#ifndef __MEASUREMENT_MODELS
#define __MEASUREMENT_MODELS

#include "motion_model.hpp"

static const MatrixXd I = MatrixXd::Identity(4, 4);

class LaserMeasurementModel: public MeasurementModel {
private:
  MatrixXd R;
  MatrixXd H;

public:
  LaserMeasurementModel(): R(MatrixXd(2, 2)), H(MatrixXd(2, 4)) {
    R <<
      0.0225, 0,
      0, 0.0225;
    H <<
      1, 0, 0, 0,
      0, 1, 0, 0;
  }
  
  void InitState(const VectorXd& measurement, VectorXd& state) const {
    state << measurement[0], measurement[1], 0, 0;
  }

  void UpdateState(const VectorXd& measurement, VectorXd& state, MatrixXd& cov) const {
      VectorXd y = measurement - H * state;
      MatrixXd S = H * cov * H.transpose() + R;
      MatrixXd K = cov * H.transpose() * S.inverse();
      
      state = state + K * y;
      cov = (I - K * H) * cov;
  }
};

class RadarMeasurementModel: public MeasurementModel {
public:
  MatrixXd R;

  RadarMeasurementModel(): R(MatrixXd(3, 3)) {
    R <<
      0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;

    
  }
  
  void InitState(const VectorXd& measurement, VectorXd& state) const {
    double rho = measurement[0], phi = measurement[1], rhodot = measurement[2];
    state <<
      rho * cos(phi),
      rho * sin(phi),
      rhodot * cos(phi),
      rhodot * sin(phi);
  }
};


#endif
