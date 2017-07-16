#include <iostream>
#include "ukf.h"
#include "tests.h"

using namespace std;

// Test SigmaPoint Augmentation
void test_sigmapoint_augmentation()
{
  auto n_x = 5;
  auto n_aug = 7;

  //set example state
  auto x = VectorXd(n_x);
  x << 5.7441,
    1.3800,
    2.2049,
    0.5015,
    0.3528;

  //create example covariance matrix
  auto P = MatrixXd(n_x, n_x);
  P << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020,
    -0.0013, 0.0077, 0.0011, 0.0071, 0.0060,
    0.0030, 0.0011, 0.0054, 0.0007, 0.0008,
    -0.0022, 0.0071, 0.0007, 0.0098, 0.0100,
    -0.0020, 0.0060, 0.0008, 0.0100, 0.0123;

  auto Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

  UKF ukf;

  ukf.x_ = x;
  ukf.P_ = P;
  ukf.std_a_ = 0.2;
  ukf.std_yawdd_ = 0.2;

  ukf.AugmentedSigmaPoints(&Xsig_aug);

  cout << "Xsig_aug = " << endl << Xsig_aug << endl;
}

// Test SigmaPoint Prediction
void test_sigmapoint_prediction()
{
  UKF ukf;
  
  //create example sigma point matrix
  auto Xsig_aug = MatrixXd(ukf.n_aug_, 2 * ukf.n_aug_ + 1);
  Xsig_aug <<
    5.7441, 5.85768, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.63052, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441,
    1.38, 1.34566, 1.52806, 1.38, 1.38, 1.38, 1.38, 1.38, 1.41434, 1.23194, 1.38, 1.38, 1.38, 1.38, 1.38,
    2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.2049, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 2.2049, 2.2049,
    0.5015, 0.44339, 0.631886, 0.516923, 0.595227, 0.5015, 0.5015, 0.5015, 0.55961, 0.371114, 0.486077, 0.407773, 0.5015, 0.5015, 0.5015,
    0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.3528, 0.3528, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879, 0.3528, 0.3528,
    0, 0, 0, 0, 0, 0, 0.34641, 0, 0, 0, 0, 0, 0, -0.34641, 0,
    0, 0, 0, 0, 0, 0, 0, 0.34641, 0, 0, 0, 0, 0, 0, -0.34641;

  auto delta_t = 0.1; //time diff in sec

  ukf.SigmaPointPrediction(Xsig_aug, delta_t);

  cout << "Xsig_pred = " << endl << ukf.Xsig_pred_ << endl;
}

void test_mean_and_covariance_prediction()
{
  UKF ukf;

  //create example matrix with predicted sigma points
  auto Xsig_pred = MatrixXd(ukf.n_x_, 2 * ukf.n_aug_ + 1);
  Xsig_pred <<
    5.9374, 6.0640, 5.925, 5.9436, 5.9266, 5.9374, 5.9389, 5.9374, 5.8106, 5.9457, 5.9310, 5.9465, 5.9374, 5.9359, 5.93744,
    1.48, 1.4436, 1.660, 1.4934, 1.5036, 1.48, 1.4868, 1.48, 1.5271, 1.3104, 1.4787, 1.4674, 1.48, 1.4851, 1.486,
    2.204, 2.2841, 2.2455, 2.2958, 2.204, 2.204, 2.2395, 2.204, 2.1256, 2.1642, 2.1139, 2.204, 2.204, 2.1702, 2.2049,
    0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337, 0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188, 0.5367, 0.535048,
    0.352, 0.29997, 0.46212, 0.37633, 0.4841, 0.41872, 0.352, 0.38744, 0.40562, 0.24347, 0.32926, 0.2214, 0.28687, 0.352, 0.318159;

  ukf.Xsig_pred_ = Xsig_pred;

  //create vector for predicted state
  auto x = VectorXd(ukf.n_x_);

  //create covariance matrix for prediction
  auto P = MatrixXd(ukf.n_x_, ukf.n_x_);

  ukf.PredictMeanAndCovariance(&x, &P);

  cout << "Predicted state" << endl;
  cout << x << endl;

  cout << "Predicted covariance matrix" << endl;
  cout << P << endl;

}

void test_predict_radar_measurement()
{
  UKF ukf;

  ukf.std_radr_   = 0.3;
  ukf.std_radphi_ = 0.0175;
  ukf.std_radrd_  = 0.1;

  //create example matrix with predicted sigma points
  auto Xsig_pred = MatrixXd(ukf.n_x_, 2 * ukf.n_aug_ + 1);

  Xsig_pred <<
    5.9374, 6.0640, 5.925, 5.9436, 5.9266, 5.9374, 5.9389, 5.9374, 5.8106, 5.9457, 5.9310, 5.9465, 5.9374, 5.9359, 5.93744,
    1.48, 1.4436, 1.660, 1.4934, 1.5036, 1.48, 1.4868, 1.48, 1.5271, 1.3104, 1.4787, 1.4674, 1.48, 1.4851, 1.486,
    2.204, 2.2841, 2.2455, 2.2958, 2.204, 2.204, 2.2395, 2.204, 2.1256, 2.1642, 2.1139, 2.204, 2.204, 2.1702, 2.2049,
    0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337, 0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188, 0.5367, 0.535048,
    0.352, 0.29997, 0.46212, 0.37633, 0.4841, 0.41872, 0.352, 0.38744, 0.40562, 0.24347, 0.32926, 0.2214, 0.28687, 0.352, 0.318159;

  ukf.Xsig_pred_ = Xsig_pred;

  // predicted measurement
  auto z_pred = VectorXd(ukf.n_zr_);

  // predicted covariance matrix
  auto S_pred = MatrixXd(ukf.n_zr_, ukf.n_zr_);

  auto Zsig_out = MatrixXd(ukf.n_zr_, 2 * ukf.n_aug_ + 1);

  ukf.PredictRadarMeasurement(&z_pred, &S_pred, &Zsig_out);

  cout << "z_pred: " << endl << z_pred << endl;

  cout << "S: " << endl << S_pred << endl;
}

void test_update_radar_state()
{
  UKF ukf;

  //create example matrix with predicted sigma points
  auto Xsig_pred = MatrixXd(ukf.n_x_, 2 * ukf.n_aug_ + 1);
  Xsig_pred <<
    5.9374, 6.0640, 5.925, 5.9436, 5.9266, 5.9374, 5.9389, 5.9374, 5.8106, 5.9457, 5.9310, 5.9465, 5.9374, 5.9359, 5.93744,
    1.48, 1.4436, 1.660, 1.4934, 1.5036, 1.48, 1.4868, 1.48, 1.5271, 1.3104, 1.4787, 1.4674, 1.48, 1.4851, 1.486,
    2.204, 2.2841, 2.2455, 2.2958, 2.204, 2.204, 2.2395, 2.204, 2.1256, 2.1642, 2.1139, 2.204, 2.204, 2.1702, 2.2049,
    0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337, 0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188, 0.5367, 0.535048,
    0.352, 0.29997, 0.46212, 0.37633, 0.4841, 0.41872, 0.352, 0.38744, 0.40562, 0.24347, 0.32926, 0.2214, 0.28687, 0.352, 0.318159;

  ukf.Xsig_pred_ = Xsig_pred;

  //create example vector for predicted state mean
  auto x = VectorXd(ukf.n_x_);
  x <<
    5.93637,
    1.49035,
    2.20528,
    0.536853,
    0.353577;

  ukf.x_ = x;

  //create example matrix for predicted state covariance
  auto P = MatrixXd(ukf.n_x_, ukf.n_x_);
  P <<
    0.0054342, -0.002405, 0.0034157, -0.0034819, -0.00299378,
    -0.002405, 0.01084, 0.001492, 0.0098018, 0.00791091,
    0.0034157, 0.001492, 0.0058012, 0.00077863, 0.000792973,
    -0.0034819, 0.0098018, 0.00077863, 0.011923, 0.0112491,
    -0.0029937, 0.0079109, 0.00079297, 0.011249, 0.0126972;

  ukf.P_ = P;

  //create example matrix with sigma points in measurement space
  auto Zsig = MatrixXd(ukf.n_zr_, 2 * ukf.n_aug_ + 1);
  Zsig <<
    6.1190, 6.2334, 6.1531, 6.1283, 6.1143, 6.1190, 6.1221, 6.1190, 6.0079, 6.0883, 6.1125, 6.1248, 6.1190, 6.1188, 6.12057,
    0.24428, 0.2337, 0.27316, 0.24616, 0.24846, 0.24428, 0.24530, 0.24428, 0.25700, 0.21692, 0.24433, 0.24193, 0.24428, 0.24515, 0.245239,
    2.1104, 2.2188, 2.0639, 2.187, 2.0341, 2.1061, 2.1450, 2.1092, 2.0016, 2.129, 2.0346, 2.1651, 2.1145, 2.0786, 2.11295;

  //create example vector for mean predicted measurement
  auto z_pred = VectorXd(ukf.n_zr_);
  z_pred <<
    6.12155,
    0.245993,
    2.10313;

  //create example matrix for predicted measurement covariance
  auto S = MatrixXd(ukf.n_zr_, ukf.n_zr_);
  S <<
    0.0946171, -0.000139448, 0.00407016,
    -0.000139448, 0.000617548, -0.000770652,
    0.00407016, -0.000770652, 0.0180917;

  //create example vector for incoming radar measurement
  auto z = VectorXd(ukf.n_zr_);
  z <<
    5.9214,
    0.2187,
    2.0062;

  MeasurementPackage meas_package;
  meas_package.sensor_type_ = MeasurementPackage::RADAR;
  meas_package.raw_measurements_ = z;

  ukf.UpdateRadarState(meas_package, z_pred, S, Zsig);

  //print result
  cout << "Updated state x: " << endl << ukf.x_ << endl;
  cout << "Updated state covariance P: " << endl << ukf.P_ << endl;
}