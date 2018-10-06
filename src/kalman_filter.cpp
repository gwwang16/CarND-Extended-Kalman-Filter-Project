#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

#include <iostream>
using namespace std;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

  cout << "Predict state: " << endl;
  cout << "F_: " << F_ << endl;
  cout << "x_: " << x_ << endl;
  cout << "P_ " << P_ << endl;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // estimate
  x_ = x_ + K*y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  cout << "KF Update state: " << endl;
  cout << "x_: " << x_ << endl;
  cout << "P_ " << P_ << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // map from cartesian to polar coordinates
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  // h(x) 
  float rho = sqrt(px*px + py*py);
  float phi = atan2(py, px);
  float rho_dot;
  if(fabs(rho) < 0.001){
    px = 0.001;
    py = 0.001;
  } else{
    rho_dot = (px*vx+py*vy)/rho;
  }

  // calculate y = z - h(x')
  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;

  // normalize the angle of y vector into +-pi
  while(y(1) > M_PI){y(1) -= 2*M_PI;}
  while(y(1) < -M_PI){y(1) += 2*M_PI;}

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  cout << "EKF Update state: " << endl;
  cout << "before update x_: " << x_ << endl;
  cout << "P_ " << P_ << endl;
  cout << "Hj_ " << H_ << endl;
  cout << "Si_ " << Si << endl;
  cout << "K " << K << endl;

  // estimate
  x_ = x_ + K*y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  cout << "EKF Update state: " << endl;
  cout << "x_: " << x_ << endl;
  cout << "P_ " << P_ << endl;
}
