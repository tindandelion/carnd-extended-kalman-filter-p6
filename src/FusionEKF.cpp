#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

const static int acc_noise_var = 9;

class LaserMeasurementModel: public MeasurementModel {
public:
  void InitState(const VectorXd& measurement, VectorXd& state) const {
    state << measurement[0], measurement[1], 0, 0;
  }
};

class RadarMeasurementModel: public MeasurementModel {
public:
  void InitState(const VectorXd& measurement, VectorXd& state) const {
    double rho = measurement[0], phi = measurement[1], rhodot = measurement[2];
    state <<
      rho * cos(phi),
      rho * sin(phi),
      rhodot * cos(phi),
      rhodot * sin(phi);
  }
};
/*
 * Constructor.
 */
FusionEKF::FusionEKF(): model(acc_noise_var) {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  H_laser_ <<
    1, 0, 0, 0,
    0, 1, 0, 0;

}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;

    previous_timestamp_ = measurement_pack.timestamp_;

    const VectorXd& z = measurement_pack.raw_measurements_;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      const MeasurementModel& measurementModel = RadarMeasurementModel();
      model.Init(z, measurementModel);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      const MeasurementModel& measurementModel = LaserMeasurementModel();
      model.Init(z, measurementModel);
    }


    ekf_.x_ = model.x;
    ekf_.P_ = model.P;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  double time_delta = (measurement_pack.timestamp_ - previous_timestamp_) / 1e6;
  previous_timestamp_ = measurement_pack.timestamp_;
  cout << "T" << ";" << time_delta << endl;

  model.Predict(time_delta);
  
  ekf_.x_ = model.x;
  ekf_.P_ = model.P;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
  model.x = ekf_.x_;
  model.P = ekf_.P_;
}
