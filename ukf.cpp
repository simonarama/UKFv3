#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
//David Simon code modified from lecture
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;
  
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initial state vector
  x_ = VectorXd(5);
  
  x_ << 1, 1, 0, 0, 0;

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  
  // Set initial as identity matrix as recommended in lecture
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;
  
  S_laser_ = MatrixXd(2,2);
  
  Si = MatrixXd(3,3); //S inverse for radar
  
  Si_laser = MatrixXd(2,2); // S inverse for laser

  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.15, 0, //.0225 in EKF
              0, 0.15;
  
  H_ = MatrixXd(2, 5);
  //initialize H_laser_  change to 2 x 5
  H_ << 1,0,0,0,0,//add 0
        0,1,0,0,0; //add 0

  delta_t = 0;
  
  // Process noise standard deviation longitudinal acceleration in m/s^2  change from 30 to 1?
  std_a_ = 1.35;

  // Process noise standard deviation yaw acceleration in rad/s^2  change from 30 to 1?
  std_yawdd_ = 0.5;

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
  
  n_x_ = 5;
  
  n_aug_ = 7;  //add linear (longitudinal) acceleration and angular (yaw) acceleration
  
  
  //define spreading parameter
  double lambda_ = 3 - n_aug_;
  
  
  //create sigma point matrix
  
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_) {
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /** 
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      //set the state with the initial location and zero velocity

      float ro = meas_package.raw_measurements_[0];
      float theta = meas_package.raw_measurements_[1];
      float ro_dot = meas_package.raw_measurements_[2];
      float px = ro*cos(theta);
      float py = ro*sin(theta);
      double p_x = px;
      double p_y = py;
      x_<< p_x, p_y, 0, 0, 0;
      cout << "x_ radar" << endl<< x_ << endl << endl;
      previous_timestamp_ = meas_package.timestamp_;
      

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //set the state with the initial location and zero velocity
      float px = meas_package.raw_measurements_[0];

      float py = meas_package.raw_measurements_[1];

      double p_x = px;
      double p_y = py;
      x_<< p_x, p_y, 0, 0, 0;
      cout << "x laser" << endl << x_ << endl << endl;
      previous_timestamp_ = meas_package.timestamp_;}
      
    is_initialized_ = true;
    
  return;}

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    
    
    double delta_t = (meas_package.timestamp_ - previous_timestamp_)/1000000.0;
    previous_timestamp_ = meas_package.timestamp_;
    cout << "delta_t" << endl << delta_t << endl << endl;  
    Prediction(delta_t);
    UpdateRadar(meas_package);}


  if (meas_package.sensor_type_ == MeasurementPackage::LASER){
    // Laser updates
    
    double delta_t = (meas_package.timestamp_ - previous_timestamp_)/1000000.0;
    previous_timestamp_ = meas_package.timestamp_;
    cout << "delta_t" << endl << delta_t << endl << endl;  
    Prediction(delta_t);
    UpdateLidar(meas_package);}
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */

 

 
 void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
   
  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);
   

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  
  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  
  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();
  
  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
  
  //create matrix with predicted sigma points as columns and initialize
  Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
  
  
  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }
  
  //define spreading parameter
  
  
  //create vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);
  weights << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0; // initialize
  
  // set weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights(i) = weight;
  }

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_ + weights(i) * Xsig_pred.col(i);
  }
  cout << "x_ predicted" << endl << x_ << endl << endl;
  
  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

  P_ = P_ + weights(i) * x_diff * x_diff.transpose() ;
  }
  cout << "P_" << endl << P_ << endl << endl;
  
}




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

  float px = meas_package.raw_measurements_[0];
  float py = meas_package.raw_measurements_[1];
  
  z_ = VectorXd(2);
  z_<< 0,0; //initialize
  z_ << px, py;
  cout << "z_ laser" << endl << z_ << endl << endl;
    //lidar is linear, update state using Kalman Filter equations in EKF project

    //measurement covariance matrix - laser
   
  VectorXd Hx = H_ * x_;

  VectorXd y = z_ - Hx;
  cout << "y" << endl << y << endl << endl;
    
  MatrixXd HP = H_ * P_;
  MatrixXd Ht = H_.transpose();
  //initialize 
  S_laser_ << 0,0,
              0,0;
  
  S_laser_ = HP * Ht + R_laser_;
  //MatrixXd S = (H_ * P_ * Ht) + R_;
  
  //initialize S inverse
  Si_laser << 0,0,
              0,0;
  Si_laser = S_laser_.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si_laser;
  
  x_ = x_ + K * y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

    //NIS normalized innovation squared
  VectorXd ytran = y.transpose();
  //VectorXd NIS = ytran * S.inverse() * y;
  VectorXd Si_lasery = Si_laser * y;
  cout << "Lidar NIS is 0.103(95%), 5.99(5%)" << endl << ytran.dot(Si_lasery) << endl << endl;
  
  //print result
  cout << "Updated state x lidar: " << endl << x_ << endl << endl;
  cout << "Updated state covariance P: " << endl << P_ << endl;
  

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  //if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

    

  float ro = meas_package.raw_measurements_[0];
  float theta = meas_package.raw_measurements_[1];
  float ro_dot = meas_package.raw_measurements_[2];
    
  z_ = VectorXd(3);
  z_ << 0,0,0;  //initialize
  z_ << ro, theta, ro_dot;
  cout << "z_radar" << endl << z_ << endl << endl;
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  
  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);
  weights << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0; //initialize weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights(0) = weight_0;
  
  for (int i=1; i<2*n_aug_+1; i++) {  
    double weight = 0.5/(n_aug_+lambda_);
    weights(i) = weight;

  }
  
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  //initialize
  Zsig << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;

  

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points

    // extract values for better readibility
    double p_x = Xsig_pred(0,i) + 0.0000001; //add adjustment to avoid nan
    double p_y = Xsig_pred(1,i) + 0.00000001; //add adjustment to avoid nan
    double v  = Xsig_pred(2,i);
    double yaw = Xsig_pred(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot

  }
  
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  //z_pred.fill(0.0);
  z_pred << 0,0,0;
  for (int i=0; i < 2*n_aug_+1; i++) {
    z_pred = z_pred + weights(i) * Zsig.col(i);
  }
  cout << "z_pred" << endl << z_pred << endl << endl;
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  //S.fill(0.0); //initialize S
  S << 0,0,0,
       0,0,0,
       0,0,0;
   
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  //Tc.fill(0.0);
  Tc << 0,0,0,
        0,0,0,
        0,0,0,
        0,0,0,
        0,0,0;

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    S = S + weights(i) * z_diff * z_diff.transpose();

    VectorXd x_diff = Xsig_pred.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  //initialize R for radar
  R << 0,0,0,
       0,0,0,
       0,0,0;

  R << std_radr_*std_radr_, 0, 0,
       0, std_radphi_*std_radphi_, 0,
       0, 0,std_radrd_*std_radrd_;

  S = S + R;
   
  //initialize S inverse
  Si << 0,0,0,
        0,0,0,
        0,0,0;

  //Kalman gain K;
  Si = S.inverse(); 
  MatrixXd K = Tc * Si;

  
  //residual
  VectorXd y = z_ - z_pred;
  cout << "y" << endl << y << endl << endl;
  //angle normalization
  while (y(1)> M_PI) y(1)-=2.*M_PI;
  while (y(1)<-M_PI) y(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * y;
  cout << "x_radar" << endl << x_ << endl << endl;
  
  P_ = P_ - K*S*K.transpose();
  cout << "P_" << endl << P_ << endl << endl;
  
  //NIS
  ytran = y.transpose();
  VectorXd Siy = Si * y;
  //NIS = ytran * Siy;
  cout << "Radar NIS is 0.352(95%), 7.815(5%)" << endl << ytran.dot(Siy) << endl << endl;}
  