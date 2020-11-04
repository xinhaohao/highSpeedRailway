//
// Created by xhh14 on 2020/9/22.
//

#include "highSpeedRailway.h"
using std::stod;

class imuLcjData : public TimeSeriesBase
{
public:
    imuLcjData()
    {
        items = {"time", "lcj", "hxj", "fyj", "hgj"};
    }
    // 启停位置
    vector<vector<int>> interval;
    void readData(const string &filename) override
    {
    }
};
