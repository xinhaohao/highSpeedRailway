//
// Created by xhh14 on 2020/9/22.
//

#include "highSpeedRailway.h"
using std::stod;

class imuData : public TimeSeriesBase
{
public:
    imuData()
    {
        items = {"time", "hxj", "fyj", "hgj"};
    }
    // 航向角、俯仰角、横滚角
    void readData(const string &filename) override
    {
        ifstream inFile;
        inFile.open(filename);
        string buff;
        while (getline(inFile, buff))
        {
            dateTime t{};
            double ep[6];
            auto items = split_str(buff, ",");
            if (items.size() < 6)
            {
                cerr << "error in read imu data:" << buff << endl;
                continue;
            }
            if (stoi(items[2]) == 0)
                continue;

            auto eps = split_str(items[0], "- ");
            if (eps.size() < 7)
            {
                cerr << "error in read imu data date:" << items[0] << endl;
                continue;
            }

            for (int i = 0; i < 6; i++)
            {
                ep[i] = stod(eps[i]);
            }
            ep[5] += stod(eps[6]) / 1e3;
            t.setTime(ep);

            this->data.emplace_back();
            auto &item = this->data.back();
            item = vector<double>(3);
            for (int i = 0; i < 3; i++)
            {
                item[i] = stod(items[3 + i]);
            }
            this->time.push_back(t);
        }
        cout << "data num:" << this->time.size() << endl;
    }
};
