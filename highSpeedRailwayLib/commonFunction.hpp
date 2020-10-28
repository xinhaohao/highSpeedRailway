//
// Created by xhh14 on 2020/9/22.
//
#pragma once
#include "vector"
#include "string"
#include "map"
using std::map;
using std::stof;
using std::string;
using std::vector;

template <typename T>
int filter(T &data)
{
    map<int, int> rec;
    vector<int> l;
    for (int i = 0; i < data.size();)
    {
        int v = data[i];
        int i0 = i;
        while (i < data.size() && data[i] == v)
            i++;
        rec[i - i0] = i0;
    }
    for (auto i : rec)
    {
        if (i.first > 200)
            return 0;
        if (data[i.second] == 1)
            continue;
        for (int j = i.second; j < i.second + i.first; j++)
        {
            data[j] = 1 - data[j];
        }
        return 1;
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
