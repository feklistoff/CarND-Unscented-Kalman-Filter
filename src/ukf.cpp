#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <fstream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * initializes Unscented Kalman filter
 */
UKF::UKF() 
{    
    // for the first measurement
    is_initialized_ = false;
    
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;
    
    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // state dimension
    n_x_ = 5;
    
    // augmented state dimension
    n_aug_ = n_x_ + 2;

    // sigma points dimension
    n_sig_ = 2 * n_aug_ + 1;

    // radar measurement dimension (r, phi, and r_dot)
    n_z_radar_ = 3;

    // lidar measurement dimension (px , py)
    n_z_lidar_ = 2;

    // sigma point spreading parameter
    a_ = 0.5;
    k_ = 5.0;
    lambda_ = a_*a_ * (n_aug_ + k_) - n_aug_;

    // time when the state is true, in us
    time_us_ = 0.0;

    // initial state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    x_ = VectorXd(n_x_);

    // initial covariance matrix
    P_ = MatrixXd(n_x_, n_x_);
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;
    
    // predicted sigma points matrix
    Xsig_pred_ = MatrixXd(n_x_, n_sig_);
    Xsig_pred_.fill(0.0);

    // process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 1.5;

    // process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.5;

    // laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;

    // weights of sigma points
    weights_ = VectorXd(n_sig_);
    weights_.fill(1 / (2 * (lambda_ + n_aug_)));
    weights_(0) = lambda_ / (lambda_ + n_aug_);

    // process noise covariance matrix
    Q_ = MatrixXd(2, 2);
    Q_ << std_a_*std_a_, 0,
          0, std_yawdd_*std_yawdd_;

    // radar measurement noise covariance matrix
    R_radar_ = MatrixXd(n_z_radar_, n_z_radar_);
    R_radar_ << std_radr_*std_radr_, 0, 0,
                0, std_radphi_*std_radphi_, 0,
                0, 0, std_radrd_*std_radrd_;

    // lidar measurement noise covariance matrix
    R_lidar_ = MatrixXd(n_z_lidar_, n_z_lidar_);
    R_lidar_ << std_laspx_*std_laspx_, 0,
                0, std_laspy_*std_laspy_;

    NIS_radar_ = 0.0;
    NIS_lidar_ = 0.0;
}

UKF::~UKF() {}

/**
 * @param meas_package the latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) 
{
    if (!is_initialized_)
    {
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
        {
            double rho     = meas_package.raw_measurements_(0);
            double phi     = meas_package.raw_measurements_(1);
            double rho_dot = meas_package.raw_measurements_(2);
            
            // [pos1 pos2 vel_abs yaw_angle yaw_rate]
            double px = rho * cos(phi);
            double py = rho * sin(phi);
            
            double vx = rho_dot * cos(phi);
            double vy = rho_dot * sin(phi);
            double v = sqrt(vx * vx + vy * vy);
            
            x_ << px, py, v, 0, 0;
        }
        
        if (meas_package.sensor_type_ == MeasurementPackage::LASER)
        {
            // [pos1 pos2 vel_abs yaw_angle yaw_rate]
            double px = meas_package.raw_measurements_(0);
            double py = meas_package.raw_measurements_(1);
            
            x_ << px, py, 0, 0, 0;
        }

        is_initialized_ = true;
        time_us_ = meas_package.timestamp_;
    }
    else
    {
        // calculate elapsed time 
        double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
        time_us_ = meas_package.timestamp_;

        Prediction(dt);
        
        if (meas_package.sensor_type_ == MeasurementPackage::LASER and use_laser_)
            UpdateLidar(meas_package);
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR and use_radar_)
            UpdateRadar(meas_package);
        
        // // create an ofstream for the csv file output
        // ofstream outFile;
        // // open file
        // outFile.open("nis.csv", ios_base::app);
        // if (outFile.is_open())
        // {
        //     outFile << NIS_lidar_ << ", " << NIS_radar_ << endl;
        //     outFile.close();
        // }
        // else
        // {
        //     cout << "Not open" << endl;
        // }
    }
}

/**
 * predicts sigma points, the state, and the state covariance matrix
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one
 */
void UKF::Prediction(double dt) 
{    
    // generate sigma points
    
    // augmented state vector
    VectorXd x_aug = VectorXd(n_aug_);
    x_aug.head(n_x_) = x_;  // fill with first elements from x
    x_aug(n_x_)   = 0;      // add zeroes for remaining elements in vector
    x_aug(n_x_+1) = 0;
    
    // augmented state covariance matrix
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(P_.rows(), P_.cols()) = P_;
    P_aug.bottomRightCorner(Q_.rows(), Q_.cols()) = Q_;
    
    // square root matrix L
    MatrixXd L = P_aug.llt().matrixL();
    // square root
    L *= sqrt(lambda_ + n_aug_);

    // create sigma points matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);
    Xsig_aug.fill(0.0);
    Xsig_aug.col(0) = x_aug;
    for (int i = 0; i < n_aug_; i++)
    {
        Xsig_aug.col(i+1)        = x_aug + L.col(i);
        Xsig_aug.col(i+n_aug_+1) = x_aug - L.col(i);
    }

    // predict sigma points
    for (int i = 0; i < n_sig_; i++)
    {
        // extract values for better readability
        double p_x      = Xsig_aug(0,i);
        double p_y      = Xsig_aug(1,i);
        double v        = Xsig_aug(2,i);
        double yaw      = Xsig_aug(3,i);
        double yawd     = Xsig_aug(4,i);
        double nu_a     = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);

        // predicted state values
        double px_p, py_p, v_p, yaw_p, yawd_p;

        // avoid division by zero
        if (fabs(yawd) > 0.001) 
        {
            px_p = p_x + (v / yawd) * (sin(yaw + yawd * dt) - sin(yaw));
            py_p = p_y + (v / yawd) * (cos(yaw) - cos(yaw + yawd * dt));
        }
        else 
        {
            px_p = p_x + v * dt * cos(yaw);
            py_p = p_y + v * dt * sin(yaw);
        }

        v_p = v;
        yaw_p = yaw + yawd * dt;
        yawd_p = yawd;

        // add noise
        double dt2 = dt * dt;
        px_p   += (1 / 2) * dt2 * cos(yaw) * nu_a;
        py_p   += (1 / 2) * dt2 * sin(yaw) * nu_a;
        v_p    += dt * nu_a;
        yaw_p  += (1 / 2) * dt2 * nu_yawdd;
        yawd_p += dt * nu_yawdd;

        // write predicted sigma point into right column
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
    }

    // predict state mean
    x_ = Xsig_pred_ * weights_;
    
    // predict state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < n_sig_; i++) 
    {
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // angle normalization
        while (x_diff(3) >  M_PI) x_diff(3) -= 2.0*M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2.0*M_PI;

        P_ += weights_(i) * x_diff * x_diff.transpose();
    }
}

/**
 * updates the state and the state covariance matrix using a laser measurement
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) 
{
    // matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z_lidar_, n_sig_);
    
    //transform sigma points into measurement space
    Zsig = Xsig_pred_.block(0, 0, n_z_lidar_, n_sig_);
        
    // mean predicted measurement
    VectorXd z_pred = VectorXd(n_z_lidar_);
    z_pred = Zsig * weights_;

    // measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z_lidar_, n_z_lidar_);
    S.fill(0.0);
    for (int i = 0; i < n_sig_; i++)
    {
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        S += weights_(i) * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    S += R_lidar_;

    // incoming radar measurement
    VectorXd z = VectorXd(n_z_lidar_);
    z << meas_package.raw_measurements_(0),  // px
         meas_package.raw_measurements_(1);  // py
    
    // cross correlation matrix Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z_lidar_);
    Tc.fill(0.0);
    for (int i = 0; i < n_sig_; i++)
    {  
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // angle normalization
        while (x_diff(3) >  M_PI) x_diff(3) -= 2.0*M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2.0*M_PI;

        Tc += weights_(i) * x_diff * z_diff.transpose();
    }

    // Kalman Gain
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z_diff = z - z_pred;

    // update state mean and covariance matrix
    x_ += K * z_diff;
    P_ -= K * S * K.transpose();

    // NIS Update
    NIS_lidar_ = z_diff.transpose() * S.inverse() * z_diff;
    // cout << " LIDAR measurement NIS: " << NIS_lidar_ << "; reference value 5.991" << endl;
}

/**
 * updates the state and the state covariance matrix using a radar measurement
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) 
{    
    // matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z_radar_, n_sig_);
    Zsig.fill(0.0);
    
    //transform sigma points into measurement space
    for (int i = 0; i < n_sig_; i++) 
    {        
        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v   = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);

        double vx = v * cos(yaw);
        double vy = v * sin(yaw);

        // measurement model
        Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        // r
        Zsig(1,i) = atan2(p_y, p_x);                                // phi
        Zsig(2,i) = (p_x*vx + p_y*vy ) / sqrt(p_x*p_x + p_y*p_y);   // r_dot
    }
    
    // mean predicted measurement
    VectorXd z_pred = VectorXd(n_z_radar_);
    z_pred = Zsig * weights_;

    // measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z_radar_, n_z_radar_);
    S.fill(0.0);
    for (int i = 0; i < n_sig_; i++)
    {
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // angle normalization
        while (z_diff(1) >  M_PI) z_diff(1) -= 2.0*M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;

        S += weights_(i) * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    S += R_radar_;

    // incoming radar measurement
    VectorXd z = VectorXd(n_z_radar_);
    z << meas_package.raw_measurements_(0),  // r
         meas_package.raw_measurements_(1),  // phi
         meas_package.raw_measurements_(2);  // r_dot
    
    // cross correlation matrix Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);
    Tc.fill(0.0);
    for (int i = 0; i < n_sig_; i++)
    {  
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // angle normalization
        while (z_diff(1) >  M_PI) z_diff(1) -= 2.0*M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // angle normalization
        while (x_diff(3) >  M_PI) x_diff(3) -= 2.0*M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2.0*M_PI;

        Tc += weights_(i) * x_diff * z_diff.transpose();
    }

    // Kalman Gain
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z_diff = z - z_pred;

    // angle normalization
    while (z_diff(1) >  M_PI) z_diff(1) -= 2.0*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;

    // update state mean and covariance matrix
    x_ += K * z_diff;
    P_ -= K * S * K.transpose();

    // NIS Update
    NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
    // cout << "RADAR measurement NIS: " << NIS_radar_ << "; reference value 7.815" << endl;
}
