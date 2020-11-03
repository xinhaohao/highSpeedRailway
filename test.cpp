#include "commonFunction.hpp"
#include "highSpeedRailway.h"
#include <iostream>
#include "geolib.h"
using namespace std;

static void calCoordDiff(const vector<vector<double>> &roverData, const vector<double> &bseCoord, vector<vector<double>> &res)
{
    assert(bseCoord.size() == 3);
    assert(roverData.size() > 0);
    assert(roverData[0].size() == 3);
    res.clear();
    res = roverData;
    for (auto &item : res)
    {
        for (int i = 0; i < 3; i++)
        {
            item[i] -= bseCoord[i];
        }
    }
}

static map<string, string> readConfig(string configFile)
{
    ifstream iFile;
    iFile.open(configFile);
    string buff;
    map<string, string> config;
    while (getline(iFile, buff))
    {
        auto items = split_str(buff, "\t ");
        if (items.size() == 2)
        {
            config[items[0]] = items[1];
        }
    }
    return config;
}

int main()
{
    // 读取流动站坐标
    auto config = readConfig("/home/eric/Documents/highSpeedRailway/config_file.txt");
    string roverFile = "../data/rover_coord/2018140_02_1s/2018140_12.XYZ.r001.LC";
    gnssCoord roverGnssCoord;
    if (!config.count("rover_coord"))
    {
        throw("no rover coord config");
    }
    roverGnssCoord.readData(config["rover_coord"], "r001");
    roverGnssCoord.writeCsv("../data/debug/r001.csv");

    // 读取基准站坐标
    string baseCoordFile = "../data/base_data/baseCoords_xyz.csv";
    baseCoords bseCord;
    bseCord.coordSys = "ecef";
    vector<int> baseIdx = {1, 2, 3};
    if (!config.count("bse_coord_xyz"))
    {
        throw("no base coord xyz config");
    }
    bseCord.readData(config["bse_coord_xyz"], baseIdx, 0);

    double lat0 = 118.15 / 180.0 * PI;
    auto ell = geoElliposoid("cgcs2000");
    auto proj = getProj(lat0);

    vector<int> baseCoordIdx = {0, 1, 2};
    coordTrans(xyz2xyh, bseCord.data, baseCoordIdx, proj, ell);
    bseCord.resetCoordMap();
    bseCord.coordSys = "xyh-118.15";
    coordTrans(xyz2xyh, roverGnssCoord.data, baseCoordIdx, proj, ell);
    roverGnssCoord.writeCsv("../data/debug/debug_xyh.csv");

    // 构建基线
    vector<string> baseStas = {"B016", "B017", "B018", "B015"};
    vector<baseLine> bsls(baseStas.size());
    for (int i = 0; i < baseStas.size(); i++)
    {
        bsls[i].baseName = baseStas[i];
        bsls[i].roverName = "r001";
        calCoordDiff(roverGnssCoord.data, bseCord.coordsMap[baseStas[i]], bsls[i].data);
        bsls[i].time = roverGnssCoord.time;
        bsls[i].writeCsv("../data/result/");
    }
    if (!config.count("bse_coord_xyh"))
    {
        throw("no base coord xyh config");
    }
    bseCord.readData(config["bse_coord_xyh"]);

    lcjData lcj;
    imuData imu;
    if (!config.count("imu_file") || !config.count("lcj_file"))
        throw("no imu or lcj file config");
    lcj.readData(config["lcj_file"]);
    imu.readData(config["imu_file"]);
    imuLcjData imuLcj;
    combineData(lcj, imu, imuLcj);
    imuLcj.findStateChange(1, 0.005, imuLcj.interval);
    vector<imuCoord> imuLcjs;
    // calCoord(imuLcj, imuLcjs);

    imuCoord imuGnssCoord;
    combineData(roverGnssCoord, imuLcj, imuGnssCoord);
    roverGnssCoord.writeCsv("../data/debug/rover.csv");
    imuLcj.writeCsv("../data/debug/imuLcj.csv");
    imuGnssCoord.writeCsv("../data/debug/imuGnssCoord.csv");
    imuGnssCoord.findStateChange(1, 0.2, imuGnssCoord.interval);
    writeData(imuGnssCoord.interval, "../data/debug/res.csv");
    vector<imuCoord> coords;
    for (auto i : imuGnssCoord.interval)
    {
        coords.emplace_back();
        coords.back().time.resize(i[1] - i[0] + 1);
        coords.back().data.resize(i[1] - i[0] + 1);

        std::copy(imuGnssCoord.time.begin() + i[0], imuGnssCoord.time.begin() + i[1], coords.back().time.begin());
        std::copy(imuGnssCoord.data.begin() + i[0], imuGnssCoord.data.begin() + i[1], coords.back().data.begin());
    }
    return 0;
}
