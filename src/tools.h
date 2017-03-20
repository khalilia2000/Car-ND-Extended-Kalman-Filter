#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

  /**
  * A helper method to calculate h(x), which returns the polar coordinates (i.e. ro, phi and ro_dot) from cartesian coordinates (i.e. px, py, px_dot and py_dot)
  */
  Eigen::VectorXd CalculatePolarFromCartesian(const Eigen::VectorXd& x_state);

  /**
  * A helper method to calculate c(x), which returns the cartesian coordinates (i.e. px, py, px_dot and py_dot) from polar coordinates (i.e. ro, phi and ro_dot)
  */
  Eigen::VectorXd CalculateCartesianFromPolar(const Eigen::VectorXd& x_state);

};

#endif /* TOOLS_H_ */
