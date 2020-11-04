//
// Created by xhh14 on 2020/9/22.
//
#include "dateTime.hpp"
#include "timeSeriesBase.hpp"
#include "iostream"
#include "fstream"
#include "commonFunction.hpp"
using std::stod;

using std::cerr;
using std::endl;
using std::ifstream;
using std::ofstream;

class lcjData : public TimeSeriesBase
{
public:
    lcjData()
    {
        items = {"time", "lcj"};
    }
    void readData(const string &filename) override
    {
        ifstream inFile;
        inFile.open(filename);
        if (!inFile.is_open())
        {
            cerr << "error in open file:" << filename << endl;
        }
        string buff;
        while (getline(inFile, buff))
        {
            dateTime datetime{};
            auto items = split_str(buff, " ,-");
            if (items.size() < 10)
            {
                cerr << "error in read lcjdata line:" << buff << endl;
                continue;
            }
            this->data.emplace_back();
            this->data.back().push_back(stod(items[1]));

            double ep[6];
            for (int i = 0; i < 6; i++)
            {
                ep[i] = stod(items[3 + i]);
            }
            ep[5] += stod(items[9]) / 1000;
            datetime.setTime(ep);
            this->time.push_back(datetime);
        }
    }
};

