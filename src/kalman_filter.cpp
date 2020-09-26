#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * Motion prediction 
   */


  // State transition matrix (Prediction)
  x_ = (F_ * x_);

  MatrixXd F_trp = F_.transpose();
  
  // Process measurement matrix
  P_ = (F_ * P_ * F_trp) + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * Measurement Update
   */
  // z: is the measurement from the sensor [px, py]

  // Here is not required as the measuerement arrives with the same state
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd H_t = H_.transpose();
  MatrixXd S = (H_ * P_ * H_t) + R_;
  MatrixXd K = P_ * H_t * S.inverse();

  // New estimation
  x_ = x_ + (K * y);

  // Identity matrix with the state size
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * DESC: Updates the state by using EKF
   */

  // Remember we compare the measurement with the predicted state
  // z: Is the measurement from the sensor [rho, phi, rhodot]

  // Extracting the predicted state
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  // Converting the prediction into polar coordinates
  float rho = sqrt(pow(px, 2) + pow(py, 2));  // Distance 
  float phi = atan2(py, px);                  // Bearing angle
  float rho_d = ((px * vx) + (py * vy))/ rho; // Rate of change 
  
  // z_pred represents the h(x) function
  VectorXd z_pred = H_ * x_;
  z_pred << rho, phi, rho_d;

  // Updating matrices
  VectorXd y = z - z_pred;

  y[1] = atan2(sin(y[1]), cos(y[1]));

  MatrixXd H_t = H_.transpose();
  MatrixXd S = (H_ * P_ * H_t) + R_;
  MatrixXd K = P_ * H_t * S.inverse();

  // New estimation
  x_ = x_ + (K * y);

  // Identity matrix with the state size
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  P_ = (I - K * H_) * P_;
}
