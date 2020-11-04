//
// Created by xhh14 on 2020/9/22.
//
#pragma once
#include "vector"
#include "string"
#include "map"
#include "algorithm"
#include "iostream"
#include "fstream"
using std::endl;
using std::map;
using std::ofstream;
using std::stod;
using std::string;
using std::vector;

template <typename T>
int filter(T &data)
{
    vector<vector<int>> rec;
    for (int i = 0; i < data.size();)
    {
        int v = data[i];
        int i0 = i;
        while (i < data.size() && data[i] == v)
            i++;
        rec.emplace_back();
        rec.back().push_back(i - i0);
        rec.back().push_back(i0);
    }
    long minlen = INT64_MAX;
    vector<int> *minInterval;
    for (auto &i : rec)
    {
        if (i[0] < minlen)
        {
            minlen = i[0];
            minInterval = &i;
        }
    }
    auto &minI = *minInterval;
    if (minI[0] > 250)
        return 0;
    for (int j = minI[1]; j < minI[1] + minI[0]; j++)
    {
        data[j] = 1 - data[j];
    }
    return 1;
}
static vector<string> split_str(const string &s, const string &seperator)
{
    vector<string> result;
    typedef string::size_type string_size;
    string_size i = 0;

    while (i != s.size())
    {
        //找到字符串中首个不等于分隔符的字母；
        while (i != s.size())
        {
            if (seperator.find(s[i]) == std::string::npos)
                break;
            ++i;
        }

        //找到又一个分隔符，将两个分隔符之间的字符串取出；
        string_size j = i;
        while (j != s.size())
        {
            if (seperator.find(s[j]) != std::string::npos)
            {
                break;
            }
            ++j;
        }
        if (i != j)
        {
            result.push_back(s.substr(i, j - i));
            i = j;
        }
    }
    return result;
}

static vector<double> inflectionPointDection(const vector<double> data, int w)
{
    int i1, i2;
    if (data.size() < 2 * w)
    {
        std::cerr << "error in inflectionPointDection: data size too small" << std::endl;
        throw("error in inflectionPointDection: data size too small");
    }
    vector<double> ave1(data.size(), 0);
    vector<double> ave2(data.size(), 0);
    vector<double> flg(data.size(), 0);
    auto n = data.size();
    for (int i = 0; i < w; i++)
    {
        ave1[w - 1] += data[i];
    }
    ave2[0] = ave1[w - 1];
    for (int i = w; i < ave1.size() - 1; i++)
    {
        ave1[i] = ave1[i - 1] - data[i - w] + data[i];
    }
    for (int i = 1; i < n - w + 1; i++)
    {
        ave2[i] = ave2[i - 1] - data[i - 1] + data[i + w - 1];
    }
    for (int i = w - 1; i < n - w - 1; i++)
    {
        flg[i] = (ave2[i + 1] - ave1[i]) / w;
    }
    return flg;
}

// 按时间对数据进行插值

// 写入数据
template <typename T>
static void writeData(T &data, const string &filename)
{
    ofstream ofile;
    ofile.open(filename);
    ofile.setf(std::ios::fixed);
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

// 计算运动和静止状态的分界线

/**
 * @brief 将data1和data2按照时间插值匹配并储存到res中
 * 
 * @param data1 
 * @param data2 
 * @param res 
 */
