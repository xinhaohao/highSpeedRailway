#ifndef TIMESERIESBASE_HPP_
#define TIMESERIESBASE_HPP_
#include <vector>
#include <iostream>
#include "dateTime.hpp"
#include <string>
#include <assert.h>
#include "fstream"
#include "commonFunction.hpp"
#include "math.h"

using std::cerr;
using std::cout;
using std::endl;
using std::ifstream;
using std::ofstream;
using std::ostream;
using std::stod;
using std::string;
using std::vector;

class TimeSeriesBase
{
public:
    vector<vector<double>> data;
    vector<dateTime> time;
    vector<string> items;

    virtual void readData(const string &filename) = 0;
    virtual void writeCsv(const string &filename) final
    {
        ofstream oFile;
        oFile.open(filename);
        oFile.setf(std::ios::fixed);
        if (isRight())
        {
            for (auto i : items)
            {
                oFile << i << ",";
            }
            oFile << std::endl;
            for (int i = 0; i < time.size(); i++)
            {
                oFile << time[i] << ",";
                for (auto j : data[i])
                {
                    oFile << j << ",";
                }
                oFile << endl;
            }
        }
        else
        {
            cerr << "Time size not match data size\n";
        }
        oFile.close();
    }
    static int combineData(TimeSeriesBase &data1, TimeSeriesBase &data2, TimeSeriesBase &res)
    {
        auto &d1 = data1.data;
        auto &d2 = data2.data;
        auto &t1 = data1.time;
        auto &t2 = data2.time;
        int idx1 = 0, idx2 = 1;

        while (idx1 < t1.size() && idx2 < t2.size())
        {
            while (idx1 < t1.size() && t1[idx1] - t2[idx2-1] < 0)
                idx1++;
            while (idx2 < t2.size())
            {
                if (t1[idx1] - t2[idx2 - 1] >= 0 && t2[idx2] - t1[idx1] >= 0)
                    break;
                idx2++;
            }
            cout << t1[idx1] << "\t" << t1[idx1 + 1] << endl;
            cout << t2[idx2 - 1] << "\t" << t2[idx2] << endl;
            if (idx2 < t2.size())
            {
                vector<double> d;
                for (auto i : d1[idx1])
                {
                    d.push_back(i);
                }
                double dt1, dt2;
                dt1 = t1[idx1] - t2[idx2 - 1];
                dt2 = t2[idx2] - t1[idx1];
                for (int i = 0; i < d2[idx2].size(); i++)
                {
                    double dd = d2[idx2 - 1][i] * dt2 + d2[idx2][i] * dt1;
                    if (dt1 + dt2 != 0)
                    {
                        dd /= (dt1 + dt2);
                    }
                    else
                    {
                        dd = d2[idx2][i];
                    }
                    d.push_back(dd);
                }
                res.time.push_back(t1[idx1]);
                res.data.push_back(std::move(d));
                idx1++;
            }
        }
        return res.time.size();
    }
    virtual void readCsv(const string filename) final
    {
        time.clear();
        data.clear();
        ifstream ifile;
        ifile.open(filename);
        if (!ifile.is_open())
        {
            cerr << "error in open file:" << filename << endl;
            return;
        }
        string buff;
        getline(ifile, buff);
        if (buff.size() == 0)
        {
            cerr << "error in imucoord read csv\n";
            return;
        }
        if (buff.back() == '\n')
            buff.pop_back();
        items = split_str(buff, ",");

        while (getline(ifile, buff))
        {
            if (buff.back() == '\n')
                buff.pop_back();
            auto items = split_str(buff, ",");
            time.emplace_back();
            auto ts = split_str(items[0], "- :");
            double ep[6];
            for (int i = 0; i < 6 && i < ts.size(); i++)
            {
                ep[i] = std::stod(ts[i]);
            }
            time.back().setTime(ep);
            data.emplace_back();
            auto &theData = data.back();
            for (int i = 1; i < items.size(); i++)
            {
                theData.push_back(std::stod(items[i]));
            }
        }
        ifile.close();
        cout << time[0] << endl;
        cout << time.back() << endl;
        cout << "imuCoords::readCsv:" << endl;
        cout << "time size:" << time.size() << " data size = " << data.size() << endl;
    }

    void findStateChange(int idx, double threshHold, vector<vector<int>> &interval)
    {
        vector<int> start, stop;
        vector<double> dx(time.size() - 1, 0);
        auto dataDebug = data;
        for (int i = 1; i < time.size(); i++)
        {
            dx[i - 1] = (data[i][idx] - data[i - 1][idx]) / (time[i] - time[i - 1]);
            dx[i - 1] = fabs(dx[i - 1]);
            dataDebug[i - 1].push_back(dx[i - 1]);
        }
        int w = 15 / (time[1] - time[0]);
        int j = 0;
        auto temp = inflectionPointDection(dx, w);
        for (int i = 1; i < temp.size() - 1; i++)
        {
            if (fabs(temp[i]) > 0.4 && ((temp[i] - temp[i - 1]) * (temp[i] - temp[i + 1]) > 0))
            {
                if (temp[i] > 0)
                {
                    for (int j = i; j >= 0; j--)
                    {
                        if (dx[j] < 0.005)
                        {
                            start.push_back(j);
                            break;
                        }
                    }
                }
                else
                {
                    for (int j = i; j < temp.size(); j++)
                    {
                        if (dx[j] < 0.005)
                        {
                            stop.push_back(j);
                            break;
                        }
                    }
                }
            }
        }
        for (int i = 0; i < temp.size(); i++)
        {
            dataDebug[i].push_back(dx[i]);
            dataDebug[i].push_back(temp[i]);
        }
        writeData(dataDebug, "../data/diff.csv");
        if (start.size() == 0 || stop.size() == 0)
            return;

        auto start_bak = start;
        auto stop_bak = stop;
        start.clear();
        stop.clear();
        start.push_back(start_bak[0]);
        stop.push_back(stop_bak[0]);

        for (int i = 1; i < start_bak.size(); i++)
        {
            if (start_bak[i] != start.back())
            {
                start.push_back(start_bak[i]);
            }
        }

        for (int i = 1; i < stop_bak.size(); i++)
        {
            if (stop_bak[i] != stop.back())
            {
                stop.push_back(stop_bak[i]);
            }
        }

        if (start[0] > stop[0])
        {
            start.insert(start.begin(), 0);
        }
        for (int i = 0; i < start.size() && i < stop.size(); i++)
        {
            interval.push_back(vector<int>{start[i], stop[i]});
            cout << start[i] << "\t" << stop[i] << "\t" << endl;
        }
    }
    void downSampling(double timeInterval)
    {
        assert(isRight());
        std::cout << "before downsamplint, data num:" << time.size() << "\t" << data.size() << std::endl;
        vector<vector<double>> data2;
        vector<dateTime> time2;
        data2.swap(data);
        time2.swap(time);
        int i = 0;
        dateTime t;
        t = time2[0];
        while (i < time2.size())
        {
            // cout << "time interval:" << time2[i] -t << endl;
            if (time2[i] - t > timeInterval)
            {
                time.push_back(time2[i]);
                data.push_back(data2[i]);
                t = time2[i];
            }
            i++;
        }
        std::cout << "after downsamplint, data num:" << time.size() << "\t" << data.size() << std::endl;
    }
    void readFp(ifstream &ifile, const vector<int> &timeIdx, const vector<int> &dataIdx, const string &sep, double scale = 1.0, string timeSep = "")
    {
        string buff;
        if (timeIdx.size() != 6 && timeIdx.size() == 1)
        {
            throw("timeIdx size error");
        }
        if (timeIdx.size() == 1)
        {
            if (timeSep == "")
            {
                throw("error in timeSep");
            }
        }
        if (dataIdx.size() <= 0)
        {
            throw("dataIdx size is 0");
        }

        auto maxIdx = *(std::max_element(timeIdx.begin(), timeIdx.end()));
        maxIdx = std::max(maxIdx, *(std::max_element(dataIdx.begin(), dataIdx.end())));

        while (getline(ifile, buff))
        {
            auto items = split_str(buff, sep);
            if (items.size() <= maxIdx)
            {
                cerr << "error in read data:" << buff << std::endl;
                continue;
            }

            double eps[6];

            if (timeIdx.size() == 1)
            {
                auto eps_str = split_str(items[timeIdx[0]], timeSep);
                if (eps_str.size() != 6)
                {
                    throw("error in time str");
                }
                for (int i = 0; i < 6; i++)
                {
                    eps[i] = stod(eps_str[i]);
                }
            }
            else
            {
                for (int i = 0; i < 6; i++)
                {
                    eps[i] = stod(items[timeIdx[i]]);
                }
            }
            time.emplace_back();
            time.back().setTime(eps);

            data.emplace_back();
            for (auto i : dataIdx)
            {
                data.back().push_back(scale * stod(items[i]));
            }
        }
    }
    void readFp(ifstream &ifile, int timeIdx, const vector<int> dataIdx, const string &sep, double scale = 1.0);
    bool isRight() { return time.size() == data.size(); }
};

#endif