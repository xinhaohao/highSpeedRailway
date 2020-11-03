#pragma once
#include "highSpeedRailway.h"
#include "commonFunction.hpp"

class gnssCoord : public TimeSeriesBase
{
public:
    string stationName;
    void readData(const string &filename) override
    {
        throw("还未实现");
    }
    void readData(const string &filename, const string &station)
    {
        vector<int> timeIdx = {0, 1, 2, 3, 4, 5};
        vector<int> dataIdx = {6, 8, 10};
        string sep = "\t ";

        stationName = station;
        double scale = 1;
        ifstream ifile;
        ifile.open(filename);
        if (!ifile.is_open())
        {
            throw("error in open baseline data");
            return;
        }

        string buff;
        getline(ifile, buff);
        auto its = split_str(buff, "\t ");
        items.clear();
        for (auto i : dataIdx)
        {
            items.push_back(its[i]);
        }
        getline(ifile, buff);

        if (split_str(buff, "\t ")[1].find("(m)") != string::npos)
        {
            scale = 1;
        }
        else if (split_str(buff, "\t ")[1].find("(mm)") != string::npos)
        {
            scale = 0.001;
        }
        else
        {
            throw("error in read baseline data");
        }

        readFp(ifile, timeIdx, dataIdx, sep, scale = scale);
    }
};
