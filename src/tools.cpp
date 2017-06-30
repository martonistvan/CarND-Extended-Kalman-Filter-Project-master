#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	VectorXd red;
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() == 0) {
		cout << "The estimation vector size should not be zero" << endl;
	}
	else if (estimations.size() != ground_truth.size()) {
		cout << "The estimation vector size should equal ground truth vector size" << endl;
	}
	else {
		//accumulate squared residuals
		for(int i=0; i < estimations.size(); ++i){
			// ... your code here
			red = estimations[i]-ground_truth[i];
			red = red.array()*red.array();
			rmse += red;
		}

		//calculate the mean
		rmse = rmse / estimations.size();
		//calculate the squared root
		rmse = rmse.array().sqrt();
	}
	//return the result
	return rmse;
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
/*
  TODO:
    * Calculate a Jacobian here.
*/
	MatrixXd Hj(3,4);
	Hj << 0,0,0,0,
			0,0,0,0,
			0,0,0,0;
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
	//cout << "px, py, vx, vy: " << px << ";" << py << ";" << vx << ";" << vy << endl;

	float denom = px*px+py*py;
	//cout << "denom: " << denom << endl;
	//check division by zero
	if (px == 0 && py == 0) {
	//if(fabs(denom) < 0.00001){
		// if condition is true then print the following
		cout << "Error: division by 0!" << endl;
	}else {
		//compute the Jacobian matrix
		//cout << "sqrt(denom): " << sqrt(denom) << endl;
		float H00 = px / sqrt(denom);
		float H01 = py / sqrt(denom);
		float H10 = -1*py / (denom);
		float H11 = px / (denom);
		float H20 = (py*(vx*py-vy*px)) / pow((denom),1.5);
		float H21 = (px*(vy*px-vx*py)) / pow((denom),1.5);
		float H22 = px / sqrt(denom);
		float H23 = py / sqrt(denom);
		Hj << H00, H01, 0, 0,
				H10, H11, 0, 0,
				H20, H21, H22, H23;

	}

	//cout << "Hj: " << Hj << endl;
	return Hj;

}


