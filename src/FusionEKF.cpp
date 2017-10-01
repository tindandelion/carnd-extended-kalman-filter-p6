#include "FusionEKF.h"

using Eigen::VectorXd;

MeasurementModel& FusionEKF::SelectModel(MeasurementPackage::SensorType sensor) {
  if (sensor == MeasurementPackage::RADAR) {
    return radar;
  } else {
    return laser;
  }
}

const VectorXd& FusionEKF::ProcessMeasurement(const MeasurementPackage& measurement_pack) {
  const VectorXd& measurement = measurement_pack.raw_measurements_;
  MeasurementModel& measurement_model = SelectModel(measurement_pack.sensor_type_);
  double time_delta_seconds = (measurement_pack.timestamp_ - previous_timestamp_) / 1e6;

  previous_timestamp_ = measurement_pack.timestamp_;
  if (!is_initialized_) {
    motion_model.Init(measurement, measurement_model);
    is_initialized_ = true;
  } else {
    motion_model.Predict(time_delta_seconds);
    motion_model.Update(measurement, measurement_model);
  }

  return motion_model.GetState();
}
