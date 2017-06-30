#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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
/*	cout << "X: " << x_ << endl;
	cout << "F: " << F_ << endl;
	cout << "Q: " << Q_ << endl;
	cout << "P: " << P_ << endl;*/
	x_ = F_ * x_;
//	cout << "X': " << x_ << endl;
	MatrixXd Ft = F_.transpose();
//	cout << "Ft: " << Ft << endl;
	P_ = F_ * P_ * Ft + Q_;
//		cout << "P': " << P_ << endl;
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
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	const double Max_pi = M_PI;
	const double Min_pi = -M_PI;

	float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
	float phi = atan2(x_(1), x_(0));
	float rho_dot;

	if (rho == 0) {
		rho_dot = 0;
	} else {
		rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;

	}

	VectorXd z_pred(3);

	z_pred << rho, phi, rho_dot;

	VectorXd y = z - z_pred;

	//normalize angle
	if(y(1) < Min_pi) {
		y(1) = Max_pi + fmod(y(1) - Min_pi, Max_pi - Min_pi);
	}
	if(y(1) > Max_pi) {
		y(1) = fmod(y(1) - Min_pi, Max_pi - Min_pi) + Min_pi;
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
