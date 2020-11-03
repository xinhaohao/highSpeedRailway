//
// Created by xhh14 on 2020/9/28.
//
#include "highSpeedRailway.h"

using std::cout;
using std::map;
using std::stod;
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
    void readData(string filename, vector<int> coordIdx = {2, 1, 3}, int nameIdx = 0)
    {
        ifstream ifile;
        ifile.open(filename);
        string buff;
        data.clear();
        name.clear();
        if (!ifile.is_open())
            throw("error in open file");
        while (getline(ifile, buff))
        {
            auto items = split_str(buff, ",");
            if (items.size() < 4)
                continue;
            name.push_back(items[0]);
            data.emplace_back();
            auto &coords = data.back();
            for (int i = 0; i < 3; i++)
            {
                coords.push_back(stod(items[coordIdx[i]]));
            }
            coordsMap[items[0]] = coords;
        }
    }
    void logBaseCoords()
    {
        for (auto i : coordsMap)
        {
            cout << i.first << ":" << i.second[0] << "," << i.second[1] << "," << i.second[2] << endl;
        }
    }
    string coordSys;
    vector<string> name;
    vector<vector<double>> data;
    void resetCoordMap()
    {
        for (int i = 0; i < name.size(); i++)
        {
            coordsMap[name[i]] = data[i];
        }
    }
    map<string, vector<double>> coordsMap;
};

