#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "Eigen/Dense"

#include "measurement_package.h"
#include "motion_model.hpp"
#include "measurement_models.hpp"

class FusionEKF {
public:
  const VectorXd& ProcessMeasurement(const MeasurementPackage &measurement_pack);
  
private:
  static const int noise_axy = 9;
  
  MotionModel motion_model = MotionModel(noise_axy, noise_axy);
  LaserMeasurementModel laser;
  RadarMeasurementModel radar;
  
  bool is_initialized_ = false;
  long long previous_timestamp_ = 0;

  MeasurementModel& SelectModel(MeasurementPackage::SensorType sensor);
};

#endif /* FusionEKF_H_ */
