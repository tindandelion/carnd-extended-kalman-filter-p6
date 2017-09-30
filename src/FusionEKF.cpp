#include <iostream>
#include "FusionEKF.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

MeasurementModel& FusionEKF::SelectModel(MeasurementPackage::SensorType sensor) {
  if (sensor == MeasurementPackage::RADAR) {
    return radar;
  } else {
    return laser;
  }
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage& measurement_pack) {
  const VectorXd& measurement = measurement_pack.raw_measurements_;
  MeasurementModel& measurement_model = SelectModel(measurement_pack.sensor_type_);
  double time_delta_seconds = (measurement_pack.timestamp_ - previous_timestamp_) / 1e6;

  previous_timestamp_ = measurement_pack.timestamp_;
  if (!is_initialized_) {
    model.Init(measurement, measurement_model);
    is_initialized_ = true;
  } else {
    model.Predict(time_delta_seconds);
    model.Update(measurement, measurement_model);
  }
}
