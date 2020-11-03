#pragma once
#include <vector>
#include <string>
const static double PI = 3.14159265358979;
using std::string;
using std::vector;

struct eli
{
    double a, f, b, e, mu, omega;
};

typedef vector<double> (*coordTransFunc)(const vector<double> &, const vector<double> &, const eli &);

vector<double> getProj(double lon0);
eli geoElliposoid(string name);

vector<double> llh2xyh(const vector<double> &llh, const vector<double> &proj, const eli &ell);
vector<double> xyz2llh(const vector<double> &xyz, const vector<double> &proj, const eli &ell);
vector<double> xyz2xyh(const vector<double> &xyz, const vector<double> &proj, const eli &ell);
void coordTrans(coordTransFunc f, vector<vector<double>> &data, vector<int> &coordIdx, const vector<double> &proj, const eli &ell);