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

  void CalculateProcessNoise(double time_delta) {
    MatrixXd G = MatrixXd(4, 2);
    G <<
      time_delta * time_delta / 2, 0,
      0, time_delta * time_delta / 2,
      time_delta, 0,
      0, time_delta;
    
    Q = G * Qa * G.transpose();
  }

  void CalculateProcessChange(double time_delta) {
    F <<
      1, 0, time_delta, 0,
      0, 1, 0, time_delta,
      0, 0, 1, 0,
      0, 0, 0, 1;
  }

  void PredictState() {
    x = F * x;
    P = F * P * F.transpose() + Q;
  }
  
public:
  MotionModel(double noise_ax, double noise_ay):
    x(VectorXd::Zero(4)),
    P(MatrixXd::Zero(4, 4)),
    F(MatrixXd::Zero(4, 4)),
    Q(MatrixXd::Zero(4, 4)),
    Qa(2, 2) {
    Qa <<
      noise_ax, 0,
      0, noise_ay;
  }

  const VectorXd& GetState() const {
    return x;
  }

  void Init(const VectorXd& measurement, const MeasurementModel& measurement_model) {
    measurement_model.InitState(measurement, x);
    P <<
      1000, 0, 0, 0,
      0, 1000, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;
  }

  void Predict(double time_delta) {
    CalculateProcessNoise(time_delta);
    CalculateProcessChange(time_delta);
    PredictState();
  }

  void Update(const VectorXd& measurement, const MeasurementModel& measurement_model) {
    measurement_model.UpdateState(measurement, x, P);
  }
  
};


#endif
