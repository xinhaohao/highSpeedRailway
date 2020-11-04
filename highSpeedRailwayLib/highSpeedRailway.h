//
// Created by xhh14 on 2020/9/23.
//

#ifndef HIGHSPEEDRAILWAYDATA_HIGHSPEEDRAILWAY_H
#define HIGHSPEEDRAILWAYDATA_HIGHSPEEDRAILWAY_H
#include <vector>
#include <string>
#include <iostream>
#include "fstream"
#include "map"
#include "algorithm"

#include "dateTime.hpp"
#include "timeSeriesBase.hpp"
#include "baseCoords.hpp"
#include "baseLine.hpp"
#include "commonFunction.hpp"
#include "imuCoord.hpp"
#include "imuData.hpp"
#include "lcjData.hpp"
#include "staticCoord.hpp"
#include "imuLcjData.hpp"
#include "gnssCoord.hpp"
using namespace std;
using std::stod;

/**
 * @brief 读取gps基线数据
 * 时间 dx dy dz
 */

void calCoord(imuLcjData &imulcj, vector<imuCoord> &res);

void transCoord(vector<vector<double>> &coords, vector<vector<double>> &staticCoord, int idx);

void simulate(baseLine &res, imuCoord &coords, double error);
#endif //HIGHSPEEDRAILWAYDATA_HIGHSPEEDRAILWAY_H
