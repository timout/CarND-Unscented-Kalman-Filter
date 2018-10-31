#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF 
{
private:
  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  const double std_a_ = {1};

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  const double std_yawdd_ = {0.5};

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  ///* Laser measurement noise standard deviation position1 in m
  const double std_laspx_ = {0.15};

  ///* Laser measurement noise standard deviation position2 in m
  const double std_laspy_ = {0.15};

  ///* Radar measurement noise standard deviation radius in m
  const double std_radr_ = {0.3};

  ///* Radar measurement noise standard deviation angle in rad
  const double std_radphi_ = {0.03};

  ///* Radar measurement noise standard deviation radius change in m/s
  const double std_radrd_  = {0.3};
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  ///* State dimension
  const int n_x_ = {5};

  ///* Augmented state dimension
  const int n_aug_ = {7};

  ///* Sigma point spreading parameter
  const double lambda_ =  {static_cast<double>(3 - n_x_)};

  ///* time when the state is true, in us
  long long time_us_ = {-1};

  // Calculate delta_t, store current time for future
  double delta_t;

public:

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* the current NIS for radar
  //double NIS_radar_;

  ///* the current NIS for laser
  //double NIS_laser_ ;

  /**
   * Constructor
   */
  UKF() : UKF(true, true) {}

  /**
   * Constructor
   */
  UKF(bool use_laser, bool use_radar);

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage & meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction();

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage & meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage & meas_package);

private:

  bool isSupported(MeasurementPackage & meas_package);

  bool init(MeasurementPackage & meas_package);

  void radarInit(MeasurementPackage & meas_package);

  void lidarInit(MeasurementPackage & meas_package);

  /**
   * Update state mean and covariance matrix
   */
  void updateState(MatrixXd & Tc, MatrixXd & S, VectorXd & z_diff);

};

#endif /* UKF_H */
