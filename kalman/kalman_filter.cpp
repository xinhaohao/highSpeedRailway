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

void KalmanFilter::Predict(SystemModel &sys)
{
    x_ = sys.f(x_);
    F_ = sys.F_;
    MatrixXd Ft = F_.transpose();
    cout << F_ << endl;
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(MeasureModel &mea)
{
    auto z_pre = mea.f(x_);
    auto z = mea.z();
    H_ = mea.H();

    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;

    VectorXd y = z - z_pre;
    MatrixXd S = H_ * PHt + R_;
    MatrixXd K = PHt * S.inverse();

    //Update State
    x_ = x_ + (K * y);
    //Update covariance matrix
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
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
