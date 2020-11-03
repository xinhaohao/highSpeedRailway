//
// Created by xhh14 on 2020/9/23.
//

#include "highSpeedRailway.h"
#include <cmath>
#include <random>
using std::stod;

void calCoord(imuLcjData &imulcj, vector<imuCoord> &res)
{
    for (int intervalIdx = 0; intervalIdx < imulcj.interval.size(); intervalIdx++)
    {
        res.emplace_back();
        auto &coord = res[res.size() - 1];
        int is, ie;
        is = imulcj.interval[intervalIdx][0];
        ie = imulcj.interval[intervalIdx][1];
        cout << "is:" << is << "ie:" << ie << endl;
        cout << "start time: " << imulcj.time[is] << endl;
        cout << "end time: " << imulcj.time[ie] << endl;

        coord.time.push_back(imulcj.time[is]);
        coord.data.push_back(imulcj.data[is]);

        int ic = imulcj.data[is].size();
        coord.ic = ic;

        for (int j = 0; j < 3; j++)
            coord.data[0].push_back(0);

        for (int i = is + 1; i < ie; i++)
        {
            double dx[3];
            double hxj, fyj, dl;
            hxj = (imulcj.data[i][1] + imulcj.data[i - 1][1]) / 2 / 180.0 * M_PI;
            fyj = (imulcj.data[i][2] + imulcj.data[i - 1][2]) / 2 / 180.0 * M_PI;
            dl = (imulcj.data[i][0] - imulcj.data[i - 1][0]);
            dx[0] = dl * sin(hxj) * cos(fyj);
            dx[1] = dl * cos(hxj) * cos(fyj);
            dx[2] = dl * sin(fyj);
            coord.data.push_back(imulcj.data[i]);
            auto &lastCoord = coord.data[i - is - 1];
            for (int k = 0; k < 3; k++)
            {
                coord.data[i - is].push_back(dx[k] + lastCoord[ic + k]);
            }
            coord.time.push_back(imulcj.time[i]);
        }
    }
}

void transCoord(vector<vector<double>> &coords, vector<vector<double>> &staticCoord, int idx)
{
    vector<double> dxyh1(3, 0), dxyh2(3, 0);
    for (int i = 0; i < 3; i++)
    {
        dxyh1[i] = coords[coords.size() - 1][i + idx] - coords[0][i + idx];
        dxyh2[i] = staticCoord[1][i] - staticCoord[0][i];
    }
    double dl1 = 0, dl2 = 0;
    for (int i = 0; i < 3; i++)
    {
        dl1 += dxyh1[i] * dxyh1[i];
        dl2 += dxyh2[i] * dxyh2[i];
    }
    dl1 = pow(dl1, 0.5);
    dl2 = pow(dl2, 0.5);
    double theta = atan2(dxyh2[1], dxyh2[0]) - atan2(dxyh1[1], dxyh1[0]);
    double theta2 = asin(dxyh2[2] / dl2) - asin(dxyh1[2] / dl1);
    double k = dl2 / dl1;
    cout << "k: " << k << endl;
    cout << "theta: " << theta << endl;
    cout << "theta2: " << theta2 << endl;

    for (int i = 0; i < coords.size(); i++)
    {
        coords[i][0] *= k;
        coords[i][1] -= theta;
        coords[i][2] += theta2;
        for (int j = 0; j < 3; j++)
        {
            coords[i][j + idx] *= k;
        }
    }
    for (int i = 0; i < coords.size(); i++)
    {
        double x, y, h;
        x = coords[i][0 + idx];
        y = coords[i][1 + idx];
        h = coords[i][2 + idx];
        double l = pow((pow(x, 2) + pow(y, 2)), 0.5);
        coords[i][0 + idx] = x * cos(theta) - y * sin(theta);
        coords[i][1 + idx] = x * sin(theta) + y * cos(theta);
        coords[i][2 + idx] = l * sin(theta2) + h * cos(theta2);
    }
    for (int i = 0; i < coords.size(); i++)
    {
        for (int j = 0; j < 3; j++)
        {
            coords[i][j + idx] += staticCoord[0][j];
        }
    }
}

void simulate(baseLine &res, imuCoord &coords, double error)
{
    normal_distribution<double> n(0, error);
    default_random_engine e;
    if (res.baseCoord.size() < 3)
        return;
    res.data.clear();
    res.time.clear();
    res.time = coords.time;
    for (int i = 0; i < coords.data.size(); i++)
    {
        vector<double> diff(3, 0);
        for (int j = 0; j < 3; j++)
        {
            diff[j] = (coords.data[i][j + 4] - res.baseCoord[j]) + n(e);
        }
        res.data.push_back(diff);
    }
}
