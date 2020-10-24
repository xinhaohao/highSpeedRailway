#include <iostream>
#include "kalman_filter.h"

#define PI 3.14159265

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict(Eigen::MatrixXd &F_in)
{
    //Use the state using the state transition matrix
    F_ = F_in;
    x_ = F_ * x_;
    //Update the covariance matrix using the process noise and state transition matrix
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{

    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;

    VectorXd y = z - H_ * x_;
    MatrixXd S = H_ * PHt + R_;
    MatrixXd K = PHt * S.inverse();

    //Update State
    x_ = x_ + (K * y);
    //Update covariance matrix
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::predictEKF(const Eigen::VectorXd &dxyz)
{
    cout << "predictEKF:\n";
    MatrixXd F(5, 5);
    double dx, dy, dz;
    double px = x_(0);
    double py = x_(1);
    double pz = x_(2);
    double the = x_(3);
    double rho = x_(4);
    dx = dxyz(0);
    dy = dxyz(1);
    dz = dxyz(2);

    VectorXd x_pred(5);
    x_pred = x_;
    x_pred(0) = x_(0) + rho * (dx * cos(the) - dy * sin(the));
    x_pred(1) = x_(1) + rho * (dy * cos(the) + dx * sin(the));
    x_pred(2) = x_(2) + dz;

    x_ = x_pred;

    F.setIdentity();
    for (int j = 0; j < 3; j++)
        F(j, 4) = dxyz(j);
    F(0, 3) = rho * (-dx * sin(the) - dy * cos(the));
    F(1, 3) = rho * (-dy * sin(the) + dx * cos(the));
    F(0, 4) *= rho;
    F(1, 4) *= rho;

    MatrixXd Ft = F.transpose();
    P_ = F * P_ * Ft + Q_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, const VectorXd &dxyz)
{

    // double px = x_(0);
    // double py = x_(1);
    // double pz = x_(2);
    // double the = x_(3);
    // double rho = x_(4);
    // double dx, dy, dz;
    // dx = dxyz(0);
    // dy = dxyz(1);
    // dz = dxyz(2);

    // VectorXd z_pred(5);
    // x_pred = x_;
    // x_pred(0) = x_(0) + rho * dx + dx * cos(the) - dy * sin(the);
    // x_pred(1) = x_(1) + rho * dy + dy * cos(the) + dx * sin(the);
    // x_pred(2) = x_(2) + dz;

    // VectorXd z_pred = VectorXd(3);
    // z_pred << rho_p, theta_p, rho_dot_p;

    // VectorXd y = z - z_pred;

    // //Adjust the value of theta if it is outside of [-PI, PI]
    // if (y(1) > PI)
    // {
    //     y(1) = y(1) - 2 * PI;
    // }

    // else if (y(1) < -PI)
    // {
    //     y(1) = y(1) + 2 * PI;
    // }

    // MatrixXd Ht = H_.transpose();
    // MatrixXd PHt = P_ * Ht;

    // MatrixXd S = H_ * PHt + R_;
    // MatrixXd K = PHt * S.inverse();

    // //Update State
    // x_ = x_ + (K * y);
    // //Update covariance matrix
    // long x_size = x_.size();
    // MatrixXd I = MatrixXd::Identity(x_size, x_size);
    // P_ = (I - K * H_) * P_;
}
