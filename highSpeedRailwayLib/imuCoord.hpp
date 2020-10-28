//
// Created by xhh14 on 2020/9/23.
//
#include "highSpeedRailway.h"
using std::stof;

class imuCoord : public TimeSeriesBase
{
public:
    imuCoord()
    {
        items = {"time", "lcj", "hxj", "fyj", "hgj", "x", "y", "z"};
    }
    int ic;
    void readData(const string &filename) override {}
};
