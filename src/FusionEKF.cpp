#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
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

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */

  // State covariance matrix 
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ <<  1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0, 
              0, 0, 0, 1000;

  // Measurement matrix for the Laser
  H_laser_ << 1, 0, 0, 0, 
              0, 1, 0, 0;

  // H_radar_

  ekf_.F_ = MatrixXd(4, 4);
  ekf_.Q_ = MatrixXd(4, 4);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement. [x]
     * TODO: Create the covariance matrix. [x] Done in the constructor
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // First measurement
    cout << "[EKF]: Initialization..." << endl;
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      std::cout << "[EKF]: Radar setting initial state." <<std::endl;

      // State initialization.
      double rho = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      double rho_d = measurement_pack.raw_measurements_[2];

      double x = rho * cos(phi);
      double y = rho * sin(phi);

      // Double check both values as it is a guess
      // x = x < 0.001f ? 0.001f : x;
      // y = y < 0.001f ? 0.001f : y;

      double v_x = rho_d * cos(phi);
      double v_y = rho_d * sin(phi);

      ekf_.x_ << x, y, v_x, v_y;
    }
    
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      std::cout << "[EKF]: Lidar setting initial state." <<std::endl;

      // State initialization.
      ekf_.x_ << 
      measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 1, 1;
    }

    // Timestamp updated
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;

    std::cout << "[EKF]: Initialization done." <<std::endl;

    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // dt is now in seconds 
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ <<  1, 0, dt, 0,
              0, 1, 0, dt,
              0, 0, 1, 0, 
              0, 0, 0, 1;

  // I can set them as member attributes 
  float noise_ax = 9.0f;
  float noise_ay = 9.0f;

  float dt_4 = pow(dt, 4) / 4.0f;
  float dt_3 = pow(dt, 3) / 2.0f;
  float dt_2 = pow(dt, 2);

  // Process noise covariance matrix (Prediction)
  ekf_.Q_ <<  dt_4 * noise_ax, 0, dt_3 * noise_ax, 0,
              0, dt_4 * noise_ay, 0, dt_3 * noise_ay,
              dt_3 * noise_ax, 0, dt_2 * noise_ax, 0,
              0, dt_3 * noise_ay, 0, dt_2 * noise_ay;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
  {
    std::cout << "[EKF]: Starting Radar update." <<std::endl;

    /*
      Radar updates
        R = Measurement covariance Matrix
        H = Jacobian - Linearized measurement matrix
            Done for state linearization
    */
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } 
  
  else 
  {
    /*
      Laser updates
        R = Measurement covariance Matrix
        H = Measurement matrix
    */

    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // Print the output
  // cout << "x_ = " << ekf_.x_ << endl;
  // cout << "P_ = " << ekf_.P_ << endl;
}
