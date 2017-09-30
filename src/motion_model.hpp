#ifndef __MOTION_MODEL_H
#define __MOTION_MODEL_H

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class MeasurementModel {
public:
  virtual void InitState(const VectorXd& measurement, VectorXd& state) const = 0;
  virtual void UpdateState(const VectorXd& measurement, VectorXd& state, MatrixXd& cov) const = 0;
};

class MotionModel {
private:
  VectorXd x;
  MatrixXd P;

  MatrixXd F;
  MatrixXd Q;
  MatrixXd Qa;

  void CalculateProcessNoise(double time_delta);
  void CalculateProcessChange(double time_delta);
  void PredictState();
  
public:
  MotionModel(double noise_ax, double noise_ay);
  
  const VectorXd& GetState() const { return x; }

  void Init(const VectorXd& measurement, const MeasurementModel& measurement_model);
  void Predict(double time_delta);
  void Update(const VectorXd& measurement, const MeasurementModel& measurement_model);  
};


#endif
