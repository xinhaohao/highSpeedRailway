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

class dataset
{
public:
    vector<baseLine> bsls;
    imuCoord imus;
};

static void copyData(const vector<vector<double>> &src, vector<vector<double>> &des, int is, int ie, int idxs, int idxe)
{
    for (int i = is; i < ie; i++)
    {
        des.emplace_back();
        for (int j = idxs; j < idxe; j++)
        {
            des.back().push_back(src[i][j]);
        }
    }
}

static vector<double> average(const vector<vector<double>> &data, int is, int ie, int idxs, int idxe)
{
    auto res = vector<double>(idxe - idxs);
    for (int i = is; i < ie; i++)
    {
        for (int j = idxs; j < idxe; j++)
        {
            res[j - idxs] += data[i][j];
        }
    }
    for (int j = idxs; j < idxe; j++)
    {
        res[j - idxs] /= (ie - is);
    }
    return res;
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

    // 计算每段的imu坐标
    lcjData lcj;
    imuData imu;
    if (!config.count("imu_file") || !config.count("lcj_file"))
        throw("no imu or lcj file config");
    lcj.readData(config["lcj_file"]);
    imu.readData(config["imu_file"]);
    imuLcjData imuLcj;
    TimeSeriesBase::combineData(lcj, imu, imuLcj);
    // imuLcj.findStateChange(0, 0.005, imuLcj.interval);
    vector<imuCoord> imuLcjs;
    calCoord(imuLcj, imuLcjs);

    // for (int i = 0; i < imuLcjs.size(); i++)
    // {
    //     transCoord(imuLcjs[i].data, , 4);
    // }

    imuCoord imuGnssCoord;
    TimeSeriesBase::combineData(imuLcj, roverGnssCoord, imuGnssCoord);

    imuLcj.data.clear();
    copyData(imuGnssCoord.data, imuLcj.data, 0, imuGnssCoord.data.size(), 0, 4);
    imuLcj.time = imuGnssCoord.time;

    imuGnssCoord.findStateChange(0, 0.005, imuGnssCoord.interval);
    imuLcj.interval = imuGnssCoord.interval;
    calCoord(imuLcj, imuLcjs);
    vector<dataset> datasets;
    for (int i = 0; i < imuGnssCoord.interval.size(); i++)
    {
        datasets.emplace_back();
        vector<vector<double>> stopCoords;
        stopCoords.emplace_back();
        stopCoords.back() = average(imuGnssCoord.data, 0, imuGnssCoord.interval[i][0], 4, 7);
        stopCoords.emplace_back();
        int ie = i + 1 == imuGnssCoord.interval.size() ? imuGnssCoord.data.size() : imuGnssCoord.interval[i + 1][0];
        stopCoords.back() = average(imuGnssCoord.data, imuGnssCoord.interval[i][1], ie, 4, 7);
        datasets[i].imus = std::move(imuLcjs[i]);
        transCoord(datasets[i].imus.data, stopCoords, 4);
        for (int j = 0; j < bsls.size(); j++)
        {
            datasets[i].bsls.emplace_back();
            imuCoord temp;
            TimeSeriesBase::combineData(datasets[i].imus, bsls[j], temp);
            datasets[i].imus.data.clear();

            copyData(temp.data, datasets[i].imus.data, 0, temp.data.size(), 0, 7);
            copyData(temp.data, datasets[i].bsls[j].data, 0, temp.data.size(), 7, 10);
            datasets[i].bsls[j].time = temp.time;
            datasets[i].imus.time = temp.time;
            datasets[i].bsls[j].roverName = bsls[j].roverName;
            datasets[i].bsls[j].baseName = bsls[j].baseName;
            datasets[i].bsls[j].baseCoord = bseCord.coordsMap[bsls[j].baseName];
        }
    }

    return 0;
}
