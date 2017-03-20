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

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  DONE.
  */


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
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    DONE.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro;
      float phi;
      float ro_dot;
      float px;
      float py;
      float px_dot;
      float py_dot;
      ro = measurement_pack.raw_measurements_[0];
      phi = measurement_pack.raw_measurements_[1];
      ro_dot = measurement_pack.raw_measurements_[2];
      // calculating position and velocity
      px = ro * cos(phi);
      py = ro * sin(phi);
      if ((py == 0) || ((px*px / py + py) == 0)) {
        py_dot = 0;
      }
      else {
        py_dot = ro * ro_dot / (px*px / py + py);
      }
      if (py == 0) {
        px_dot = 0;
      }
      else {
        px_dot = px*py_dot / py;
      }
      // 
      ekf_.x_ << px, py, px_dot, py_dot;
      //
      previous_timestamp_ = measurement_pack.timestamp_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

      previous_timestamp_ = measurement_pack.timestamp_;
    }

    //re-set state covariance matrix P
    ekf_.P_(0, 0) = 1;
    ekf_.P_(1, 1) = 1;
    ekf_.P_(2, 2) = 1000;
    ekf_.P_(3, 3) = 1000;

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

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   Done.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // Set measurement matrix Hj and measurmeent covariance matrix R
    
    /**
    Convert radar from polar to cartesian coordinates and initialize state.
    */
    float ro;
    float phi;
    float ro_dot;
    float px;
    float py;
    float px_dot;
    float py_dot;
    ro = measurement_pack.raw_measurements_[0];
    phi = measurement_pack.raw_measurements_[1];
    ro_dot = measurement_pack.raw_measurements_[2];
    // calculating position and velocity
    px = ro * cos(phi);
    py = ro * sin(phi);
    if ((py == 0) || ((px*px / py + py)==0)) {
      py_dot = 0;
    } else {
      py_dot = ro * ro_dot / (px*px / py + py);
    }
    if (py == 0) {
      px_dot = 0;
    } else {
      px_dot = px*py_dot / py;
    }
    //
    Eigen::VectorXd radar_x_(4);
    radar_x_ << px, py, px_dot, py_dot;
    //
    Hj_ = tools.CalculateJacobian(radar_x_);

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
