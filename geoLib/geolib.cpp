#include "geolib.h"
#include "map"
#include "math.h"
#include "algorithm"
using std::map;
using std::transform;

static inline double pow2(double d)
{
    return pow(d, 2);
}

map<string, vector<double>> elopsoids =
    {
        {"CLK66", {6378206.4, 294.9786982}},
        {"GRS67", {6378160.0, 298.247167427}},
        {"GRS80", {6378137.0, 298.257222101}},
        {"WGS72", {6378135.0, 298.26}},
        {"WGS84", {6378137.0, 298.257223563}},
        {"ATS77", {6378135.0, 298.257}},
        {"NAD27", {6378206.4, 294.9786982}},
        {"NAD83", {6378137.0, 298.257222101}},
        {"INTER", {6378388.0, 297.0}},
        {"KRASS", {6378245.0, 298.3}},
        {"CGCS2000", {6378137.0, 298.257222101}}};

vector<double> llh2xyh(const vector<double> &llh, const vector<double> &proj, const eli &ell)
{
    if (proj.size() < 7)
    {
        throw("error in proj");
    }
    double a = ell.a + proj[5];
    double lat, lon, h;
    lat = llh[0];
    lon = llh[1];
    h = llh[2];

    double T = pow2(tan(lat));
    double e = ell.e;
    double es = sqrt(pow2(e) / (1 - pow2(e)));

    double lat0 = proj[0];
    double lon0 = proj[1];
    double scale = proj[2];
    double offsetX = proj[3];
    double offsetY = proj[4];
    double anomaly = proj[5];

    double C = pow2(e) * pow2(cos(lat)) / (1 - pow2(e));
    double A = (lon - lon0) * cos(lat);
    double v = a / sqrt(1 - pow2(e) * pow2(sin(lat)));

    double z1 = 1 - pow2(e) / 4 - 3 * pow(e, 4) / 64 - 5 * pow(e, 6) / 256;
    double z2 = 3 * pow2(e) / 8 + 3 * pow(e, 4) / 32 + 45 * pow(e, 6) / 1024;
    double z3 = 15 * pow(e, 4) / 256 + 45 * pow(e, 6) / 1024;
    double z4 = 35 * pow(e, 6) / 3072;

    double M = a * (z1 * lat - z2 * sin(2 * lat) + z3 * sin(4 * lat) - z4 * sin(6 * lat));
    double M0 = a * (z1 * lat0 - z2 * sin(2 * lat0) + z3 * sin(4 * lat0) - z4 * sin(6 * lat0));
    double N = M - M0 + v * tan(lat) * (pow(A, 2) / 2 + (5 - T + 9 * C + 4 * pow(C, 2)) * pow(A, 4) / 24 + (61 - 58 * T + pow(T, 2) + 600 * C - 330 * pow(es, 2)) * pow(A, 6) / 720);
    double E = v * (A + (1 - T + C) * pow(A, 3) / 6 + (5 - 18 * T + pow(T, 2) + 72 * C - 58 * pow(es, 2)) * pow(A, 5) / 120);
    N = N * scale + offsetX;
    E = E * scale + offsetY;
    double H = h - anomaly;
    vector<double> res(3);
    res = {N, E, H};
    return res;
}

vector<double> xyz2xyh(const vector<double> &xyz, const vector<double> &proj, const eli &ell)
{
    auto llh = xyz2llh(xyz, proj, ell);
    auto xyh = llh2xyh(llh, proj, ell);
    return xyh;
}

vector<double> xyz2llh(const vector<double> &xyz, const vector<double> &proj, const eli &ell)
{
    double x = xyz[0];
    double y = xyz[1];
    double z = xyz[2];
    double r = sqrt(x * x + y * y);
    double lat0 = atan2(z, r);
    double eps = 1.e-11;
    int max_iter = 10;
    double e2 = pow2(ell.e);
    double N, lat;
    int i;
    for (i = 0; i < max_iter; i++)
    {
        N = ell.a / sqrt(1.0 - e2 * sin(lat0) * sin(lat0));
        lat = atan2(z + N * e2 * sin(lat0), r);
        if (abs(lat - lat0) < eps)
            break;
        lat0 = lat;
    }
    if (i == max_iter && abs(lat - lat0) >= eps)
    {
        throw("XYZ2LLH can not converge after 10 iterations");
    }
    double lon = atan2(y, x);
    lon = lon - (2 * PI) * int(lon / 2 / PI);
    double h = r / cos(lat) - N;

    vector<double> res = {lat, lon, h};
    return res;
}

vector<double> getProj(double lon0)
{
    vector<double> proj(6, 0);
    proj = {0, lon0, 1.0, 0, 500000, 0, 0};
    return proj;
}

eli geoElliposoid(string name)
{
    transform(name.begin(), name.end(), name.begin(), ::toupper);
    if (!elopsoids.count(name))
    {
        throw("unknow system");
    }
    auto d = elopsoids[name];
    eli ell;
    ell.a = d[0];
    ell.f = 1 / d[1];
    ell.mu = 3.986004418e14;
    ell.omega = 7292115e-11;
    ell.b = ell.a * (1 - ell.f);
    ell.e = sqrt(1 - pow2(1 - ell.f));
    return ell;
}

void coordTrans(coordTransFunc f, vector<vector<double>> &data, vector<int> &coordIdx, const vector<double> &proj, const eli &ell)
{
    if (data.size() == 0)
    {
        throw("data size is 0");
    }

    if (coordIdx.size() < 3)
    {
        throw("error in coordIdx size");
    }

    auto maxIdx = std::max_element(coordIdx.begin(), coordIdx.end());
    if (*maxIdx >= data[0].size())
    {
        throw("coordIdx out of range");
    }

    vector<double> source(3, 0);
    for (auto &item : data)
    {
        for (int i = 0; i < 3; i++)
        {
            source[i] = item[coordIdx[i]];
        }
        auto res = f(source, proj, ell);
        for (int i = 0; i < 3; i++)
        {
            item[coordIdx[i]] = res[i];
        }
    }
}