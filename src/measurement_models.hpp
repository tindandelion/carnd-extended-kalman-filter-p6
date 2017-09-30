#ifndef __MEASUREMENT_MODELS
#define __MEASUREMENT_MODELS

#include "motion_model.hpp"

class LaserMeasurementModel: public MeasurementModel {
private:
  MatrixXd R;
  MatrixXd H;

public:
  LaserMeasurementModel();
  
  void InitState(const VectorXd& measurement, VectorXd& state) const;
  void UpdateState(const VectorXd& measurement, VectorXd& state, MatrixXd& cov) const;
};

class RadarMeasurementModel: public MeasurementModel {
private:
  MatrixXd R;

public:
  RadarMeasurementModel();  

  void InitState(const VectorXd& measurement, VectorXd& state) const;
  void UpdateState(const VectorXd& measurement, VectorXd& state, MatrixXd& cov) const;
};


#endif
