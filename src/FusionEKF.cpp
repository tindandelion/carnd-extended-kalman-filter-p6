#include "FusionEKF.h"
#include "tools.h"
#include "measurement_models.hpp"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

const static int noise_axy = 9;

/*
 * Constructor.
 */
FusionEKF::FusionEKF(): model(noise_axy, noise_axy) {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  Hj_ = MatrixXd(3, 4);

  R_radar_ = radar.R;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  const VectorXd& z = measurement_pack.raw_measurements_;
  
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


    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      model.Init(z, radar);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      model.Init(z, laser);
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
    ekf_.R_ = radar.R;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    
    model.x = ekf_.x_;
    model.P = ekf_.P_;
  } else {
    laser.UpdateState(z, model.x, model.P);

    ekf_.x_ = model.x;
    ekf_.P_ = model.P;
  }
}
