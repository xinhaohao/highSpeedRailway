#include "kalman_filter.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;

class HighSpeedMeasureModel : public MeasureModel
{
public:
    int state = 5;
    int measurement = 3;
    HighSpeedMeasureModel() : state(5), measurement(3), C(MatrixXd(measurement, state)) {}
    ~HighSpeedMeasureModel();
    MatrixXd C;
    VectorXd f(VectorXd &x) override;
};

class HighSpeedSystemModel : public SystemModel
{
public:
    int state = 5;
    VectorXd dxyz;
    int isUpdate;
    HighSpeedSystemModel() : state(5), isUpdate(0) {}
    void updateDxyz(VectorXd dxyz1)
    {
        dxyz = dxyz1;
        isUpdate = 1;
    }
    VectorXd f(const VectorXd &xx) override;
};