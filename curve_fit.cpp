// g2o - General Graph Optimization
// Copyright (C) 2012 R. Kümmerle
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <Eigen/Core>
#include <iostream>

#include "highSpeedRailway.h"

#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_with_hessian.h"
#include "g2o/core/optimization_algorithm_dogleg.h"

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/slam3d/edge_pointxyz.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"

using namespace std;

/**
 * \brief the params, a, b, and lambda for a * exp(-lambda * t) + b
 */
class VertexImuErr : public g2o::BaseVertex<3, Eigen::Vector3d>
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        VertexImuErr()
        {
        }

        virtual bool read(std::istream& /*is*/)
        {
            cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
            return false;
        }

        virtual bool write(std::ostream& /*os*/) const
        {
            cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
            return false;
        }

        virtual void setToOriginImpl()
        {
            cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        }

        virtual void oplusImpl(const double* update)
        {
            Eigen::Vector3d::ConstMapType v(update);
            _estimate += v;
        }
};

/**
 * \brief measurement for a point on the curve
 *
 * Here the measurement is the point which is lies on the curve.
 * The error function computes the difference between the curve
 * and the point.
 */
class EdgeImuPoints : public g2o::BaseMultiEdge<3, Eigen::Vector3d>
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeImuPoints()
        {
        }
        virtual bool read(std::istream& /*is*/)
        {
            cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
            return false;
        }
        virtual bool write(std::ostream& /*os*/) const
        {
            cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
            return false;
        }

        void computeError()
        {
            const VertexImuErr* params = static_cast<const VertexImuErr*>(vertex(0));
            const double& txyz = params->estimate()(0);
            const double& tz = params->estimate()(1);
            const double& scale = params->estimate()(2);
            Eigen::Vector3d diff = static_cast<const g2o::VertexPointXYZ*>(vertex(2))->estimate() - static_cast<const g2o::VertexPointXYZ*>(vertex(1))->estimate();
            diff *= scale;
            double x, y;
            x = diff(0); y = diff(1);
            diff(0) = x*cos(txyz) - y*sin(txyz);
            diff(1) = x*sin(txyz) + y*cos(txyz);
            diff(2) += diff.norm() * sin(tz);
            _error = diff - measurement();
            // double fval = a * exp(-lambda * measurement()(0)) + b;
            // _error(0) = fval - measurement()(1);
        }
};

using namespace std;
int main(int argc, char** argv)
{
    // TODO:
    int roverNo = 300;
    int useImu = 0;
    int solverType;
    string resFile = "../data/diff.csv";
    if(argc>=2){
        roverNo = atoi(argv[1]);
    }
    if(argc>=3){
        useImu = atoi(argv[2]);
    }
    if(argc>=4){
        resFile = string(argv[3]);
    }
    if(argc>=5){
        solverType = atoi(argv[4]);
    }
    cout.setf(ios::fixed, ios::floatfield);
    cout.precision(4);
    

    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(true);

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>> MyBlockSolver;
    typedef g2o::LinearSolverDense<MyBlockSolver::PoseMatrixType> MyLinearSolver;
    g2o::OptimizationAlgorithm* solver;
    switch (solverType)
    {
    case 0:
        solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));
        break;
    case 1:
        solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));
        break;
    case 2:
        solver = new g2o::OptimizationAlgorithmDogleg(g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));
        break;
    default:
        solver = new g2o::OptimizationAlgorithmDogleg(g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));
        break;
    }
    
    optimizer.setAlgorithm(solver);

    VertexImuErr* vtxImu = new VertexImuErr();
    

    // 读取imu数据+点坐标
    imuCoord imucoord;
    imucoord.readCsv("../data/0_new_coord.csv");
    
    // 读取GPS数据
    vector<baseLine> baselines;
    vector<string> baseNames = {"B015", "B016", "B017", "B018", "B019", "B020"};

    // 读取基站坐标
    baseCoords basecoord;
    basecoord.readData("../data/baseCoords.txt");
    basecoord.logBaseCoords();
    for(auto &baseName: baseNames){
        string filename;
        filename = "../data/0_" + baseName + "_R001.csv";
        baselines.emplace_back();
        baselines.back().readCsv(filename);
    }
    
    vector<g2o::VertexPointXYZ*> roverVtxs;
    // 流动站点
    auto &time = imucoord.time;
    int id = 0;
    for(int i=0;i<time.size()&&i<roverNo;i++){
        auto& t = time[i];
        auto &imudata = imucoord.data[i];
        g2o::VertexPointXYZ* vtx = new g2o::VertexPointXYZ();
        double std = 0.02;
        vtx->setEstimate(Eigen::Vector3d(imudata[4]+g2o::Sampler::gaussRand(0, std), 
                                        imudata[5]+g2o::Sampler::gaussRand(0, std), 
                                        imudata[6]+g2o::Sampler::gaussRand(0, std)));
        vtx->setId(id++);
        roverVtxs.push_back(vtx);
        optimizer.addVertex(vtx);
        cout << "add rover coord " << imudata[4] << "\t" << imudata[5] << "\t" << imudata[6] << endl;
    }

    // 基准站
    int baseStartId = id;
    vector<g2o::VertexPointXYZ*> baseVtxs;
    for(auto basename:baseNames){
        auto coord = basecoord.coordsMap[basename];
        auto vtx = new g2o::VertexPointXYZ();
        vtx->setEstimate(Eigen::Vector3d(coord[0], coord[1], coord[2]));
        vtx->setFixed(true);
        vtx->setId(id++);
        optimizer.addVertex(vtx);
        baseVtxs.push_back(vtx);
        cout << "add base coord " << coord[0] << "\t" << coord[1] << "\t" << coord[2] << endl;

    }

    int edgeId = 0;
    // imu观测
#if 1
    if(useImu){
        for(int i=0;i<imucoord.data.size()-1&&i<roverNo-1;i++){
            double diff[3];
            cout << "add od error:";
            for(int j=0;j<3;j++){
                diff[j] = imucoord.data[i+1][4+j] - imucoord.data[i][j+4];
                // 加里程计误差
                cout << diff[j] * 0.001 << "\t";
                diff[j] *= 0.999;
            }
            cout << endl;
            
            // 加航向角误差
            double x, y;
            x = diff[0]; y = diff[1];
            diff[0] = cos(0.001)*x-y*sin(0.001);
            diff[1] = sin(0.001)*x+y*cos(0.001);
            cout << "hxj error:" << diff[0] - x << diff[1] - y << endl;

            auto edge = new g2o::EdgePointXYZ();
            edge->setMeasurement(Eigen::Vector3d(diff[0], diff[1], diff[2]));
            edge->setInformation(Eigen::Matrix3d::Identity() / 0.000000006);
            edge->setVertex(0, roverVtxs[i]);
            edge->setVertex(1, roverVtxs[i+1]);
            edge->setId(edgeId++);
            optimizer.addEdge(edge);
            cout << "add edge between rovers:" << i << "\t" << i+1 << "\t" << diff[0] << "\t" << diff[1];
            cout << "\t" << diff[2] << endl;
        }
    }
#endif
    int baseEdgeIdStart = edgeId;
    // GPS基线
    #if 1
    for(int i=0;i<baseNames.size();i++){
        auto &basename = baseNames[i];
        auto &coord = basecoord.coordsMap[basename];
        auto &bsl = baselines[i];
        for(int j=0;j<bsl.data.size()&&j<roverNo;j++){
            auto &dxyz = bsl.data[j];
            auto e = new g2o::EdgePointXYZ;
            double std = 0.02;
            e->setId(edgeId++);
            e->setMeasurement(Eigen::Vector3d(dxyz[0]+g2o::Sampler::gaussRand(0, std), 
                                            dxyz[1]+g2o::Sampler::gaussRand(0, std), 
                                            dxyz[2]+g2o::Sampler::gaussRand(0, std)));
            e->setInformation(Eigen::Matrix3d::Identity()/0.02);
            e->setVertex(0, baseVtxs[i]);
            e->setVertex(1, roverVtxs[j]);
            optimizer.addEdge(e);
            cout << "add edge between base rover:" << basename << "\t" << j;
            cout << "\t" << dxyz[0] << "\t" << dxyz[1] << "\t" << dxyz[2] << endl;
        }
    }
    #endif
    cout << roverVtxs[0]->estimate()(0) << "\t";
    cout << roverVtxs[0]->estimate()(1) << "\t";
    cout << roverVtxs[0]->estimate()(2) << endl;

    optimizer.initializeOptimization();
    optimizer.optimize(100);
    cout << 100 << endl;
    ofstream ofile;
    ofile.open(resFile);
    ofile.setf(ios::floatfield, ios::fixed);
    ofile.precision(5);
    for(int i=0;i<roverVtxs.size();i++){
        double s = 0, diff[3];
        for(int j=0;j<3;j++){
            diff[j] = roverVtxs[i]->estimate()(j) - imucoord.data[i][j+4];
            s += pow(roverVtxs[i]->estimate()(j) - imucoord.data[i][j+4], 2);
            ofile << roverVtxs[i]->estimate()(j) << "," << imucoord.data[i][j+4] << ",";
            ofile << diff[j] << ",";
        }
        s = pow(s, 0.5);
        ofile << s << endl;
        
    }
    ofile.close();
    cout << roverVtxs[0]->estimate()(0) << "\t";
    cout << roverVtxs[0]->estimate()(1) << "\t";
    cout << roverVtxs[0]->estimate()(2) << endl;

    cout << imucoord.data[0][4] << "\t" << imucoord.data[0][5] << "\t" << imucoord.data[0][6] << endl;
    




    // int numPoints;
    // int maxIterations;
    // bool verbose;
    // std::vector<int> gaugeList;
    // string dumpFilename;
    // g2o::CommandArgs arg;
    // arg.param("dump", dumpFilename, "", "dump the points into a file");
    // arg.param("numPoints", numPoints, 50, "number of points sampled from the curve");
    // arg.param("i", maxIterations, 10, "perform n iterations");
    // arg.param("v", verbose, false, "verbose output of the optimization process");

    // arg.parseArgs(argc, argv);

    // // generate random data
    // double a = 2.;
    // double b = 0.4;
    // double lambda = 0.2;
    // Eigen::Vector2d* points = new Eigen::Vector2d[numPoints];
    // for (int i = 0; i < numPoints; ++i) {
    //     double x = g2o::Sampler::uniformRand(0, 10);
    //     double y = a * exp(-lambda * x) + b;
    //     // add Gaussian noise
    //     y += g2o::Sampler::gaussRand(0, 0.02);
    //     points[i].x() = x;
    //     points[i].y() = y;
    // }

    // if (dumpFilename.size() > 0) {
    //     ofstream fout(dumpFilename.c_str());
    //     for (int i = 0; i < numPoints; ++i)
    //         fout << points[i].transpose() << endl;
    // }

    // // some handy typedefs
    // typedef g2o::BlockSolver< g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic> >    MyBlockSolver;
    // typedef g2o::LinearSolverDense<MyBlockSolver::PoseMatrixType> MyLinearSolver;

    // // setup the solver
    // g2o::SparseOptimizer optimizer;
    // optimizer.setVerbose(false);

    // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
    //     g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));

    // optimizer.setAlgorithm(solver);

    // // build the optimization problem given the points
    // // 1. add the parameter vertex
    // VertexParams* params = new VertexParams();
    // params->setId(0);
    // params->setEstimate(Eigen::Vector3d(1,1,1)); // some initial value for the params
    // optimizer.addVertex(params);
    // // 2. add the points we measured to be on the curve
    // for (int i = 0; i < numPoints; ++i) {
    //     EdgePointOnCurve* e = new EdgePointOnCurve;
    //     e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
    //     e->setVertex(0, params);
    //     e->setMeasurement(points[i]);
    //     optimizer.addEdge(e);
    // }

    // // perform the optimization
    // optimizer.initializeOptimization();
    // optimizer.setVerbose(verbose);
    // optimizer.optimize(maxIterations);

    // if (verbose)
    //     cout << endl;

    // // print out the result
    // cout << "Target curve" << endl;
    // cout << "a * exp(-lambda * x) + b" << endl;
    // cout << "Iterative least squares solution" << endl;
    // cout << "a            = " << params->estimate()(0) << endl;
    // cout << "b            = " << params->estimate()(1) << endl;
    // cout << "lambda = " << params->estimate()(2) << endl;
    // cout << endl;

    // // clean up
    // delete[] points;

    return 0;
}
