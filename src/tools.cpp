#include <iostream>
#include "tools.h"


using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  * Calculate the RMSE here.
  */

  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()	|| estimations.size() == 0) {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  // accumulate squared residuals
  for (int i = 0; i < estimations.size(); ++i) {
    VectorXd diff = estimations[i] - ground_truth[i];
    diff = diff.array()*diff.array();
    rmse += diff;
  }

  //calculate the mean
  rmse = rmse / estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	/**
	* Calculate a Jacobian here.
	*/

	MatrixXd Hj(3, 4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float px2py2 = px*px + py*py;

	//check division by zero
	if (px2py2 <= 0.0001) {
		std::cout << "CalculateJacobian() - Error - Division by Zero" << std::endl;
    Hj << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;
		return Hj;
	}

	//compute the Jacobian matrix
  // 1st row
	Hj(0, 0) = px / std::pow(px2py2, 0.5);
	Hj(0, 1) = py / std::pow(px2py2, 0.5);
  Hj(0, 2) = 0;
  Hj(0, 3) = 0;
  // 2nd row
	Hj(1, 0) = -py / px2py2;
	Hj(1, 1) = px / px2py2;
  Hj(1, 2) = 0;
  Hj(1, 3) = 0;
  // 3rd row
	Hj(2, 0) = py*(vx*py - vy*px) / std::pow(px2py2, 1.5);
	Hj(2, 1) = px*(vy*px - vx*py) / std::pow(px2py2, 1.5);
	Hj(2, 2) = px / std::pow(px2py2, 0.5);
	Hj(2, 3) = py / std::pow(px2py2, 0.5);

	return Hj;
}


VectorXd Tools::CalculatePolar(const Eigen::VectorXd& x_state) {
  /**
  * Calculate h(x), which provides polar positions ro, phi and ro_dot from cartesian position and velocity values
  */

  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  float px2py2 = px*px + py*py;

  // define and initialize h_x
  VectorXd h_x(3);

  // set first element
  h_x(0) = std::pow(px2py2, 0.5);

  // set second element
  h_x(1) = atan2(py,px);

  // set third element
  if (h_x(0) <= 0.0001) {
    h_x(2) = (px*vx + py*vy) / 0.0001;
  }  else {
    h_x(2) = (px*vx + py*vy) / h_x(0);
  }

  return h_x;

}