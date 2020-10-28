#include "kalman_filter.h"
#include "highSpeedRailway.h"
#include "kalman.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;

VectorXd HighSpeedSystemModel::f(const VectorXd &xx)
{
    assert(isUpdate == 1);
    assert(xx.rows() == state);
    assert(dxyz.size() == 3);
    VectorXd x1(state);
    x1 = xx;
    double the, alp, dx, dy;
    the = xx(3);
    alp = xx(4);
    dx = dxyz(0);
    dy = dxyz(1);
    x1(0) = xx(0) + alp * (dx * cos(the) - dy * sin(the));
    x1(1) = xx(1) + alp * (dx * sin(the) + dy * cos(the));
    x1(2) = xx(2) + alp * dxyz(2);
    F_.resize(state, state);
    F_.setIdentity();
    F_(0, 3) = (-dx * sin(the) - dy * cos(the));
    F_(1, 3) = dx * cos(the) - dy * sin(the);
    F_(0, 4) = (dx * cos(the) - dy * sin(the));
    F_(1, 4) = (dx * sin(the) + dy * cos(the));
    F_(2, 4) = dxyz(2);
    isUpdate = 0;
    return x1;
}

VectorXd HighSpeedMeasureModel::f(VectorXd &x)
{
    assert(x.rows() == state);
    VectorXd z(3);
    z = x.head(3);
    return z;
}

HighSpeedMeasureModel::~HighSpeedMeasureModel()
{
}

int withInuErr()
{
    // 读取imu数据+点坐标
    imuCoord imucoord;
    imucoord.readCsv("../data/0_new_coord.csv");
    imucoord.downSampling(0.2);

    // 读取GPS数据
    vector<baseLine> baselines;
    vector<string> baseNames = {"B015", "B016", "B017", "B018", "B019", "B020"};

    // 读取基站坐标
    baseCoords basecoord;
    basecoord.readData("../data/baseCoords.txt");
    basecoord.logBaseCoords();
    for (auto &baseName : baseNames)
    {
        string filename;
        filename = "../data/0_" + baseName + "_R001.csv";
        baselines.emplace_back();
        baselines.back().readCsv(filename);
        baselines.back().downSampling(0.2);
    }

    // 初始化
    HighSpeedMeasureModel mea;
    HighSpeedSystemModel sys;

    // 参数设置
    double measure_error = 0.05;
    // 设置起点坐标
    Eigen::VectorXd x0(5);
    Eigen::MatrixXd P(5, 5), F(5, 5), H(3, 5), R(3, 3), Q(5, 5);
    for (int i = 0; i < 3; i++)
    {
        x0(i) = imucoord.data[0][4 + i] + measure_error;
    }
    x0(3) = 0;
    x0(4) = 1;

    P.setIdentity();
    P *= 0.02 * 0.02;
    P(3, 3) = 0.001 * 0.001;
    P(4, 4) = 0.00001 * 0.00001;
    F.setIdentity();
    H << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0;
    R = Eigen::Matrix3d::Identity() * pow(measure_error, 2);
    Q.setIdentity();
    Q *= pow(0.000001, 2);

    mea.setH(H);
    // 初始化滤波器
    KalmanFilter filter;
    filter.Init(x0, P, F, H, R, Q);
    std::ofstream ofile;
    ofile.open("../data/kalman_res.csv");
    ofile.precision(5);
    std::cout << "start filter:\n";

    double deltaTheta = 0.001, deltaAlpha = 1.001;

    for (int i = 0; i < imucoord.data.size() - 1; i++)
    {
        Eigen::VectorXd z(3), z_err(3);
        z << imucoord.data[i + 1][4], imucoord.data[i + 1][5], imucoord.data[i + 1][6];
        z_err = z + Eigen::Vector3d::Random() * measure_error;

        Eigen::Vector3d dxyz;
        double dx, dy;

        for (int j = 0; j < 3; j++)
            dxyz(j) = imucoord.data[i + 1][4 + j] - imucoord.data[i][4 + j];
        dx = dxyz(0);
        dy = dxyz(1);
        dxyz(0) = dx * cos(deltaTheta) - dy * sin(deltaTheta);
        dxyz(1) = dx * sin(deltaTheta) + dy * cos(deltaTheta);
        dxyz *= deltaAlpha;

        std::cout << "dxyz:\n"
                  << dxyz << endl;

        sys.updateDxyz(dxyz);
        filter.Predict(sys);
        mea.setZ(z_err);
        filter.Update(mea);

        std::cout << z - z_err << endl;
        std::cout << z - filter.x_.head(3) << endl;
        auto diff1 = z - z_err;
        auto diff2 = z - filter.x_.head(3);
        for (int j = 0; j < 3; j++)
            ofile << diff1(j) << ",";
        for (int j = 0; j < 3; j++)
            ofile << diff2(j) << ",";
        ofile << filter.x_(3) << "," << filter.x_(4) << endl;
        ofile << endl;
    }
    ofile.close();
    std::cout << "final x:\n"
              << filter.x_ << endl;
    return 0;
}

int noImuErr()
{
    // 读取imu数据+点坐标
    imuCoord imucoord;
    imucoord.readCsv("../data/0_new_coord.csv");
    imucoord.downSampling(0.2);

    // 读取GPS数据
    vector<baseLine> baselines;
    vector<string> baseNames = {"B015", "B016", "B017", "B018", "B019", "B020"};

    // 读取基站坐标
    baseCoords basecoord;
    basecoord.readData("../data/baseCoords.txt");
    basecoord.logBaseCoords();
    for (auto &baseName : baseNames)
    {
        string filename;
        filename = "../data/0_" + baseName + "_R001.csv";
        baselines.emplace_back();
        baselines.back().readCsv(filename);
        baselines.back().downSampling(0.2);
    }

    // 参数设置
    double measure_error = 0.05;
    // 设置起点坐标
    Eigen::VectorXd x0(4);
    Eigen::MatrixXd P(4, 4), F(4, 4), H(3, 4), R(3, 3), Q(4, 4);
    for (int i = 0; i < 3; i++)
    {
        x0(i) = imucoord.data[0][4 + i] + measure_error;
    }
    x0(3) = 1;

    P = Eigen::MatrixXd::Identity(4, 4);
    P *= pow(0.02, 2);
    F = Eigen::MatrixXd::Identity(4, 4);
    H << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0;
    R = Eigen::Matrix3d::Identity() * pow(measure_error, 2);
    Q = Eigen::Matrix4d::Identity() * pow(0.000001, 2);
    Q(3, 3) = 1e-20;

    // 初始化滤波器
    KalmanFilter filter;
    filter.Init(x0, P, F, H, R, Q);
    std::ofstream ofile;
    ofile.open("../data/kalman_res.csv");
    ofile.precision(5);
    for (int i = 0; i < imucoord.data.size() - 1; i++)
    {
        Eigen::VectorXd z(3), z_err(3);
        z << imucoord.data[i + 1][4], imucoord.data[i + 1][5], imucoord.data[i + 1][6];
        z_err = z + Eigen::Vector3d::Random() * measure_error;
        for (int j = 0; j < 3; j++)
        {
            F(j, 3) = imucoord.data[i + 1][4 + j] - imucoord.data[i][4 + j];
        }
        filter.Predict(F);
        filter.Update(z_err);
        cout << z - z_err << endl;
        cout << z - filter.x_.head(3) << endl;
        auto diff1 = z - z_err;
        auto diff2 = z - filter.x_.head(3);
        for (int j = 0; j < 3; j++)
            ofile << diff1(j) << ",";
        for (int j = 0; j < 3; j++)
            ofile << diff2(j) << ",";
        ofile << endl;
    }
    ofile.close();
    return 1;
}

int main()
{
    return withInuErr();
}