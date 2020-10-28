#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class SystemModel
{
public:
    virtual VectorXd f(const VectorXd &x) = 0;
    MatrixXd Q_, F_;
};

class MeasureModel
{
public:
    // virtual ~MeasureModel() = 0;
    virtual VectorXd f(VectorXd &x) = 0;
    int isUpdate;

    /**
     * @brief 设置观测矩阵
     * 
     * @param H 观测矩阵
     */
    void setH(const MatrixXd &H)
    {
        H_ = H;
        isSetH = 1;
    }
    void setZ(const VectorXd &z)
    {
        isUpdate = 1;
        z_ = z;
    }
    const VectorXd &z()
    {
        assert(isUpdate == 1);
        return z_;
    }
    const MatrixXd &H()
    {
        assert(isSetH == 1);
        return H_;
    }

    MatrixXd H_, C;

protected:
    VectorXd z_;
    int isSetH = 0;
};

class KalmanFilter
{
public:
    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // state transition matrix
    Eigen::MatrixXd F_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    // measurement matrix
    Eigen::MatrixXd H_;

    // measurement covariance matrix
    Eigen::MatrixXd R_;

    /**
   * Constructor
   */
    KalmanFilter();

    /**
   * Destructor
   */
    virtual ~KalmanFilter();

    /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
    void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
              Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

    /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
    void Predict(Eigen::MatrixXd &F_in);

    /**
     * @brief 使用systemModel对象做预测
     * 
     * @param sys 需要实现系统函数及雅可比矩阵
     */
    void Predict(SystemModel &sys);

    /**
     * @brief 使用MeasureModel更新观测
     * 
     * @param mea 需要实现观测方程及雅可比矩阵
     */
    void Update(MeasureModel &mea);

    /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
    void Update(const Eigen::VectorXd &z);
};

#endif /* KALMAN_FILTER_H_ */
