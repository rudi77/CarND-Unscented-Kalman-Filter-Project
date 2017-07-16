#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  
  // will be set to true after having received first measurement
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // timestamp
  time_us_ = 0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  //std_a_ = 3;
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  //std_yawdd_ = 0.7;
  std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  // The State dimension: [px, py, v, psi, psi_dot]
  n_x_ = 5;

  // State dimension of the radar [rho, theta, rhodot]
  n_zr_ = 3;

  // State dimension of the laser [px, py]
  n_zl_ = 2; 

  // Augmented state dimension which also adds the noise vector
  // to the state vector.
  n_aug_ = 7;

  // This is the scaling factor. How far we want to spread the sigma points
  lambda_ = 3 - n_aug_;
  
  // Sigma Points initialization
  // the number of sigma points can be calculated as follows:
  // n_sigma = 2 * n_aug +_ 1 where n_aug is the AUGMENTED STATE DIMENSION 
  // which is 7 thus we will have 15 sigma points.
  Xsig_pred_ = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);

  // Sigma Point weights. There is one weight for every sigma point. We need
  // them to calculate the mean and covariance of the predicted state.
  // The weights invert the spreading of the sigma points.
  weights_ = VectorXd::Zero(2 * n_aug_ + 1);

  weights_[0] = lambda_ / (lambda_ + n_aug_);

  for (auto i = 1; i < 2 * n_aug_ + 1; i++)
  {
    weights_[i] = 0.5 / (lambda_ + n_aug_);
  }
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) 
{
  if (!is_initialized_)
  {
    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      auto px = meas_package.raw_measurements_[0];
      auto py = meas_package.raw_measurements_[1];

      x_ << px, py, 0, 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      // Convert radar from polar to cartesian coordinates and initialize state.
      auto ro     = meas_package.raw_measurements_[0];
      auto theta  = meas_package.raw_measurements_[1];
      auto rodot  = meas_package.raw_measurements_[2];

      auto px = cos(theta) * ro;
      auto py = sin(theta) * ro;
      auto vx = rodot * cos(theta);
      auto vy = rodot * sin(theta);
      auto v  = sqrt(vx*vx + vy*vy);

      x_ << px, py, v, 0, 0;
    }
    else
    {
      // throw exception
    }

    time_us_ = meas_package.timestamp_;

    is_initialized_ = true;
  }
  else
  {
    // 1.) Prediction Step:
    // For the prediction step we use the Process Model
    // The prediction step is independent from the measurement model
    // Predict state and covariance matrix
    
    // elapsed time between k and k+1 in seconds
    auto delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;

    Prediction(delta_t);

    // 2.)Update Step:
    // Here we use the RADAR or LASER measurement model
    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      UpdateLidar(meas_package);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      UpdateRadar(meas_package);
    }
    else
    {
      // throw exception
    }

    // Update time
    time_us_ = meas_package.timestamp_;
  }
}

////////////////////////////////// start prediction methods //////////////////////////////////////////

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) 
{
  // 1.) Calc augemented sigma points
  auto Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  AugmentedSigmaPoints(&Xsig_aug);

  // 2.) Predict sigma points
  SigmaPointPrediction(Xsig_aug, delta_t);

  // 3.) Predict covariance matrix
  PredictMeanAndCovariance(&x_, &P_);
}

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) const
{
  //create augmented mean vector
  auto x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  auto P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  auto Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (auto i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  //print result
  //cout << "Xsig_aug = " << endl << Xsig_aug << endl;

  //write result
  *Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(const MatrixXd& Xsig_aug, const double delta_t)
{
  //predict sigma points
  for (auto i = 0; i < 2 * n_aug_ + 1; i++)
  {
    //extract values for better readability
    auto p_x      = Xsig_aug(0, i);
    auto p_y      = Xsig_aug(1, i);
    auto v        = Xsig_aug(2, i);
    auto yaw      = Xsig_aug(3, i);
    auto yawd     = Xsig_aug(4, i);
    auto nu_a     = Xsig_aug(5, i);
    auto nu_yawdd = Xsig_aug(6, i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }

    auto v_p    = v;
    auto yaw_p  = yaw + yawd * delta_t;
    auto yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }
}

void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out)
{
  //predicted state mean

  //create vector for predicted state
  auto x = VectorXd(n_x_);
  x.fill(0.0);

  //iterate over sigma points
  for (auto i = 0; i < 2 * n_aug_ + 1; i++)
  {
    x = x + weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix

  //create covariance matrix for prediction
  auto P = MatrixXd(n_x_, n_x_);
  P.fill(0.0);

  //iterate over sigma points
  for (auto i = 0; i < 2 * n_aug_ + 1; i++) {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;

    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

    P = P + weights_(i) * x_diff * x_diff.transpose();
  }

  //write result
  *x_out = x;
  *P_out = P;
}

////////////////////////////////// end prediction methods //////////////////////////////////////////



////////////////////////////////// start update methods //////////////////////////////////////////

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) 
{
  // predicted measurement
  auto z_pred = VectorXd(n_zr_);

  // predicted covariance matrix
  auto S_pred = MatrixXd(n_zr_, n_zr_);

  auto Zsig_out = MatrixXd(n_zr_, 2 * n_aug_ + 1);

  PredictRadarMeasurement(&z_pred, &S_pred, &Zsig_out);

  UpdateRadarState(meas_package, z_pred, S_pred, Zsig_out);
}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out)
{
  //set measurement dimension, radar can measure r, phi, and r_dot
  //create matrix for sigma points in measurement space
  auto Zsig = MatrixXd(n_zr_, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (auto i = 0; i < 2 * n_aug_ + 1; i++) 
  {
    // extract values for better readibility
    auto p_x  = Xsig_pred_(0, i);
    auto p_y  = Xsig_pred_(1, i);
    auto v    = Xsig_pred_(2, i);
    auto yaw  = Xsig_pred_(3, i);

    auto v1 = cos(yaw) * v;
    auto v2 = sin(yaw) * v;

    // measurement model
    Zsig(0, i) = sqrt(p_x*p_x + p_y*p_y);                         // r
    Zsig(1, i) = atan2(p_y, p_x);                                 // phi
    Zsig(2, i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);     // r_dot
  }

  //mean predicted measurement
  auto z_pred = VectorXd(n_zr_);

  z_pred.fill(0.0);

  for (auto i = 0; i < 2 * n_aug_ + 1; i++) 
  {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  auto S = MatrixXd(n_zr_, n_zr_);

  S.fill(0.0);

  //2n+1 simga points
  for (auto i = 0; i < 2 * n_aug_ + 1; i++) 
  {
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  auto R = MatrixXd(n_zr_, n_zr_);

  R <<  std_radr_ * std_radr_, 0, 0,
        0, std_radphi_ * std_radphi_, 0,
        0, 0, std_radrd_ * std_radrd_;

  S = S + R;

  //write result
  *z_out = z_pred;
  *S_out = S;
  *Zsig_out = Zsig;
}

void UKF::UpdateRadarState(const MeasurementPackage& meas_package, const VectorXd& z_pred, const MatrixXd& S_pred, const MatrixXd& Zsig)
{  
  // Create matrix for cross correlation Tc. It contains
  // the cross-correlation values between the sigma points in the
  // state space and the measurement space.
  auto Tc = MatrixXd(n_x_, n_zr_);

  //calculate cross correlation matrix
  Tc.fill(0.0);

  //2n+1 simga points
  for (auto i = 0; i < 2 * n_aug_ + 1; i++) {

    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S_pred.inverse();

  //residual
  auto ro = meas_package.raw_measurements_[0];
  auto theta = meas_package.raw_measurements_[1];
  auto rodot = meas_package.raw_measurements_[2];

  auto z = VectorXd(n_zr_);
  z << ro, theta, rodot;

  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S_pred * K.transpose();

  //print result
  cout << "Updated state x: " << endl << x_ << endl;
  cout << "Updated state covariance P: " << endl << P_ << endl;
}

////////////////////////////////// end update methods //////////////////////////////////////////