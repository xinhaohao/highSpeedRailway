#ifndef TIMESERIESBASE_HPP_
#define TIMESERIESBASE_HPP_
#include <vector>
#include <iostream>
#include "dateTime.hpp"
#include <string>
#include <assert.h>
#include "fstream"
#include "commonFunction.hpp"

using std::cerr;
using std::cout;
using std::endl;
using std::ifstream;
using std::ofstream;
using std::ostream;
using std::stof;
using std::string;
using std::vector;

class TimeSeriesBase
{
public:
    vector<vector<double>> data;
    vector<dateTime> time;
    vector<string> items;

    virtual void readData(const string &filename) = 0;
    virtual void writeCsv(const string &filename) final;
    virtual void readCsv(const string filename) final;
    void findStateChange(TimeSeriesBase &data, int idx, double threshHold,
                         vector<vector<int>> &interval);
    void downSampling(double timeInterval);
    void readFp(ifstream &ifile, const vector<int> &timeIdx, const vector<int> &dataIdx, const string &sep, double scale = 1.0, string timeSep = "");
    void readFp(ifstream &ifile, int timeIdx, const vector<int> dataIdx, const string &sep, double scale = 1.0);
    bool isRight() { return time.size() == data.size(); }
};

// 写入数据
template <typename T>
void writeData(T &data, const string &filename)
{
    ofstream ofile;
    ofile.open(filename);
    for (const auto &i : data)
    {
        for (const auto j : i)
        {
            ofile << j << ",";
        }
        ofile << endl;
    }
    ofile.close();
}

// 从fp读取数据
void TimeSeriesBase::readFp(ifstream &ifile, const vector<int> &timeIdx, const vector<int> &dataIdx, const string &sep, double scale, string timeSep)
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
                eps[i] = stof(eps_str[i]);
            }
        }
        else
        {
            for (int i = 0; i < 6; i++)
            {
                eps[i] = stof(items[timeIdx[i]]);
            }
        }
        time.emplace_back();
        time.back().setTime(eps);

        data.emplace_back();
        for (auto i : dataIdx)
        {
            data.back().push_back(scale * stof(items[i]));
        }
    }
}

// 计算运动和静止状态的分界线
void TimeSeriesBase::findStateChange(TimeSeriesBase &data, int idx, double threshHold,
                                     vector<vector<int>> &interval)
{
    vector<int> start, stop;
    auto &t = data.time;
    auto &d = data.data;
    vector<double> dx(t.size() - 1, 0);
    for (int i = 1; i < t.size(); i++)
    {
        dx[i - 1] = (d[i][idx] - d[i - 1][idx]); // / (t[i] - t[i-1]);
    }
    for (double &i : dx)
    {
        i = abs(i) < threshHold ? 0 : 1;
    }
    while (filter(dx) == 1)
    {
    }
    writeData(d, "../data/diff.csv");

    for (int i = 1; i < dx.size(); i++)
    {
        if (dx[i - 1] == 1 && dx[i] == 0)
        {
            stop.push_back(i);
        }
        else if (dx[i - 1] == 0 && dx[i] == 1)
        {
            start.push_back(i - 1);
        }
    }
    if (start.size() == 0)
        return;
    if (start[0] > stop[0])
    {
        start.insert(start.begin(), 0);
    }
    for (int i = 0; i < start.size() && i < stop.size(); i++)
    {
        interval.push_back(vector<int>{start[i], stop[i]});
    }
    for (int i = 0; i < interval.size() - 1; i++)
    {
        if (interval[i + 1][0] - interval[i][1] < 2000)
        {
            interval[i][1] = interval[i + 1][1];
            interval.erase(interval.begin() + i + 1);
            i--;
        }
    }
}

void TimeSeriesBase::readCsv(const string filename)
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
            ep[i] = std::stof(ts[i]);
        }
        time.back().setTime(ep);
        data.emplace_back();
        auto &theData = data.back();
        for (int i = 1; i < items.size(); i++)
        {
            theData.push_back(std::stof(items[i]));
        }
    }
    ifile.close();
    cout << time[0] << endl;
    cout << time.back() << endl;
    cout << "imuCoords::readCsv:" << endl;
    cout << "time size:" << time.size() << " data size = " << data.size() << endl;
}

void TimeSeriesBase::writeCsv(const string &filename)
{
    ofstream oFile;
    oFile.open(filename);
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
    oFile.close();
}

void TimeSeriesBase::downSampling(double timeInterval)
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

/**
 * @brief 将data1和data2按照时间插值匹配并储存到res中
 * 
 * @param data1 
 * @param data2 
 * @param res 
 */
void matchTimeSeries(TimeSeriesBase &data1, TimeSeriesBase &data2, TimeSeriesBase &res)
{
    assert(data1.isRight());
    assert(data2.isRight());
    res.time.clear();
    res.data.clear();
    auto &d1 = data1.data;
    auto &d2 = data2.data;
    auto &t1 = data1.time;
    auto &t2 = data2.time;
    int idx1 = 0, idx2 = 1;

    while (idx1 < t1.size())
    {
        while (idx1 < t1.size() && t1[idx1] - t2[idx2] < 0)
            idx1++;
        //        cout << "t1 time:" << t1[idx1] << endl;
        while (idx2 < t2.size())
        {
            if (t1[idx1] - t2[idx2 - 1] >= 0 && t2[idx2] - t1[idx1] >= 0)
                break;
            idx2++;
        }
        //        cout << "t2 time:" << t2[idx2] << endl;
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
            res.data.push_back(d);
            idx2++;
            idx1++;
        }
        else
        {
            break;
        }
    }
}

#endif