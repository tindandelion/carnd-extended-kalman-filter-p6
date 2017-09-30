#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "motion_model.hpp"
#include "tools.h"

class FusionEKF {
public:

  FusionEKF();

  void ProcessMeasurement(const MeasurementPackage &measurement_pack);
  VectorXd GetEstimate() const { return ekf_.x_; }
private:
  MotionModel model;
  KalmanFilter ekf_;
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;
};

#endif /* FusionEKF_H_ */
