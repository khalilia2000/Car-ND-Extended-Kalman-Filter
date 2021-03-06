#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
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

  //measurement matrix - laser
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //define amd initialize state covariance matrix P with placeholders
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ <<  1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

  //initialize the process covariance matrix Q with placeholder elements
  //all 1s are placeholders and should be updated.
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  1, 0, 1, 0,
              0, 1, 0, 1,
              1, 0, 1, 0,
              0, 1, 0, 1;
  
  //the initial transition matrix F with placeholder elements
  //only elements in positions 0,2 and 1,3 should be updated 
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ <<  1, 0, 1, 0,
	            0, 1, 0, 1,
	            0, 0, 1, 0,
	            0, 0, 0, 1;

  //set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    double velocity_uncertainty_;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Initialize state.
      */
      // Convert to cartesian coordinates and assign the position
      Eigen::VectorXd radar_x_ = tools.CalculateCartesianFromPolar(measurement_pack.raw_measurements_);
      ekf_.x_ << radar_x_;
      //
      previous_timestamp_ = measurement_pack.timestamp_;
      //
      velocity_uncertainty_ = 2;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      //
      previous_timestamp_ = measurement_pack.timestamp_;
      //
      velocity_uncertainty_ = 50;
    }

    //re-set state covariance matrix P
    ekf_.P_(0, 0) = 1;
    ekf_.P_(1, 1) = 1;
    ekf_.P_(2, 2) = velocity_uncertainty_;
    ekf_.P_(3, 3) = velocity_uncertainty_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /****************************************************************************************************
   *  Prediction
   *  Update the state transition matrix F according to the new elapsed time - convert time to seconds.
   *  Update the process noise covariance matrix.
   ****************************************************************************************************/


  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //update the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //update the process noise covariance matrix Q
  //note noise_ax and noise_ay are already set in the constructor
  ekf_.Q_(0, 0) = dt_4 / 4 * noise_ax;
  ekf_.Q_(0, 2) = dt_3 / 2 * noise_ax;
  ekf_.Q_(1, 1) = dt_4 / 4 * noise_ay;
  ekf_.Q_(1, 3) = dt_3 / 2 * noise_ay;
  ekf_.Q_(2, 0) = dt_3 / 2 * noise_ax;
  ekf_.Q_(2, 2) = dt_2 * noise_ax;
  ekf_.Q_(3, 1) = dt_3 / 2 * noise_ay;
  ekf_.Q_(3, 3) = dt_2 * noise_ay;
  
  //call predict function - same for both laser and radar
  ekf_.Predict();

  /********************************************************************************************************
   *  Update - Use the sensor type to perform the update step - Update the state and covariance matrices.
   ********************************************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    
    // Convert to cartesian coordinate and calculate Jacobian matrix Hj
    Eigen::VectorXd radar_x_ = tools.CalculateCartesianFromPolar(measurement_pack.raw_measurements_);
    Hj_ = tools.CalculateJacobian(radar_x_);
    // Assign matrices R and Hj
    ekf_.R_ = R_radar_;
    ekf_.H_ = Hj_;
    // use Extended Kalman Filter update
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    // Set measurement matrix H and measurmeent covariance matrix R
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    // use regular Kalman Filter update
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
