//
// Created by xhh14 on 2020/9/28.
//
#include "highSpeedRailway.h"

using std::cout;
using std::map;
using std::stof;
/**
 * @brief 基准站坐标
 * 
 */
class baseCoords
{
public:
    /**
     * @brief 读取基准站坐标文件
     * 
     * @param filename 基准站坐标文件
     */
    void readData(string filename);
    void logBaseCoords();
    vector<string> name;
    vector<vector<double>> data;
    map<string, vector<double>> coordsMap;
};

void baseCoords::readData(string filename)
{
    ifstream ifile;
    ifile.open(filename);
    string buff;
    while (getline(ifile, buff))
    {
        auto items = split_str(buff, ",");
        if (items.size() < 4)
            continue;
        name.push_back(items[0]);
        vector<double> coords;
        for (int i = 1; i < 4; i++)
        {
            coords.push_back(stof(items[i]));
        }
        double tmp = coords[1];
        coords[1] = coords[0];
        coords[0] = tmp;
        data.push_back(coords);
        coordsMap[items[0]] = coords;
    }
}

void baseCoords::logBaseCoords()
{
    for (auto i : coordsMap)
    {
        cout << i.first << ":" << i.second[0] << "," << i.second[1] << "," << i.second[2] << endl;
    }
}