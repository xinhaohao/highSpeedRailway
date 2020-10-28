#include "commonFunction.hpp"
#include "highSpeedRailway.h"
#include <iostream>
using namespace std;
int main()
{
    // 读取流动站坐标
    string roverFile = "../rover_coord/2018140_02_1s/2018140_12.XYZ.r001.LC";
    gnssCoord roverGnssCoord;
    roverGnssCoord.readData(roverFile, "r001");
    return 0;

    // 读取基准站坐标
    string baseCoordFiles = "../";
}
