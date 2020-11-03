//
// Created by xhh14 on 2020/9/23.
//
#include "highSpeedRailway.h"
using std::stod;

class imuCoord : public TimeSeriesBase
{
public:
    vector<vector<int>> interval;
    imuCoord()
    {
        items = {"time", "lcj", "hxj", "fyj", "hgj", "x", "y", "z"};
    }
    int ic;
    void readData(const string &filename) override {}
};
