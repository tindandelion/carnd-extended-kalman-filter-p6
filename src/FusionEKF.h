#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "Eigen/Dense"

#include "measurement_package.h"
#include "motion_model.hpp"
#include "measurement_models.hpp"

class FusionEKF {
public:
  FusionEKF();
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);
  VectorXd GetEstimate() const { return model.x; }
  
private:
  MotionModel model;
  LaserMeasurementModel laser;
  RadarMeasurementModel radar;
  bool is_initialized_;
  long long previous_timestamp_;
};

#endif /* FusionEKF_H_ */
