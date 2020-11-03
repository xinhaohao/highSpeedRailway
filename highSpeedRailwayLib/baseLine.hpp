//
// Created by xhh14 on 2020/9/28.
//

#include "highSpeedRailway.h"
using std::stod;

// x,y,z
class baseLine : public TimeSeriesBase
{
public:
    /**
     * @brief 读取基线数据
     * 
     * @param file 文件名
     * @param base 基站坐标
     * @param rover 流动站坐标
     */
    void readData(const string &file, string base, string rover)
    {
        baseName = base;
        float scale = 1;
        roverName = rover;
        ifstream ifile;
        ifile.open(file);
        if (!ifile.is_open())
        {
            throw("error in open baseline data");
            return;
        }
        string buff;
        getline(ifile, buff);
        getline(ifile, buff);

        if (split_str(buff, "\t ")[1].find("(m)") != string::npos)
        {
            scale = 1;
        }
        else if (split_str(buff, "\t ")[1].find("(mm)") != string::npos)
        {
            scale = 1000;
        }
        else
        {
            throw("error in read baseline data");
        }

        while (getline(ifile, buff))
        {
            if (buff[0] == '*')
                continue;
            items = split_str(buff, "\t ");
            if (items.size() < 13)
                continue;
            double eps[6];
            time.emplace_back();
            auto &t = time.back();

            for (int i = 0; i < 6; i++)
            {
                eps[i] = stod(items[i]);
            }
            t.setTime(eps);
            data.emplace_back();
            auto &d = data.back();
            d = vector<double>(3, 0);
            for (int i = 0; i < 3; i++)
            {
                d[i] = stod(items[6 + i * 2]) / scale;
            }
        }
    };
    void readData(const string &filename) override {}
    void writeCsv(string folder)
    {
        TimeSeriesBase::writeCsv(folder + baseName + "_" + roverName + ".csv");
    }

    vector<double> baseCoord;
    string baseName;
    string roverName;
};
