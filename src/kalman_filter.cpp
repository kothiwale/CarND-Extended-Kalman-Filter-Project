#include "kalman_filter.h"
using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
  

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  VectorXd z_pred(3);
  
  float v1 = sqrt(x_(0)*x_(0)+x_(1)*x_(1));
  
  float v2;
  if (x_(0) != 0){
    v2 = atan2(x_(1), x_(0));
  }else{
    v2=0;
  }
  
  float v3;
  if (fabs(v1) < 0.0001){
    v3 = 0;
  }else{
    v3 = (x_(0)*x_(2) + x_(1)*x_(3))/v1;
  }
  
  
  z_pred << v1, v2, v3;
  
  /////y(1) = atan2(sin(y(1)), cos(y(1)));
  
  VectorXd y = z - z_pred;
  
  if (y(1) > 3.14){
    y(1) = y(1) - 2*3.14;
  }
  else if (y(1) < -3.14){
    y(1) = y(1) + 2*3.14; 
  }
  
  MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}
