//
// Created by xhh14 on 2020/9/23.
//

#ifndef HIGHSPEEDRAILWAYDATA_HIGHSPEEDRAILWAY_H
#define HIGHSPEEDRAILWAYDATA_HIGHSPEEDRAILWAY_H
#include <vector>
#include <string>
#include <iostream>
#include "fstream"
#include "map"
#include "algorithm"
#include "random"
using namespace std;
class dateTime {
public:
    dateTime(int y, int mon, int d, int h, int min, int s):year(y),month(mon),day(d),hour(h),min(min),sec(s){
        tick = gettick();
    }
    dateTime() = default;

    dateTime& setTick(double tick);
    double gettick();
    dateTime* operator+(dateTime d2);
    double operator-(dateTime d2);
    void setTime(double ep[6]);
    void operator+=(double d2);
    void operator-=(double d2);
    void operator=(dateTime t2);
    friend ostream &operator << (ostream& output, dateTime datetime);

    static double startTick;
    int year, month, day, hour, min;
    double sec, tick;

};

class imuData {
public:
    void readData(string filePath);
    vector<dateTime> time;
    vector<vector<double>> data;
};
class imuLcjData {
public:
//    void combineData(imuData &imu, data &lcj);
    vector<dateTime> time;
    vector<vector<double>> data;
    vector<string> itemName = {"time", "lcj", "hxj", "fyj", "hgj"};
    vector<vector<int>> interval;
    void toCsv(string file);
};

class lcjData {
public:
    void readData(string file);
    vector<dateTime> time;
    vector<vector<double>> data;
};

class imuCoord {
public:
    vector<vector<double>> data;
    int ic;
    vector<dateTime> time;
    vector<string> itemName = {"time", "lcj", "hxj", "fyj", "hgj", "x", "y", "z"};
    void toCsv(string filename);
    void readCsv(string filename);
    void downsampling(double timeInterval);
};



class staticCoord{
public:
    void readData(const string& file);
    vector<vector<double>> data;
};

class baseLine{
public:
    void readData(const string& file, string base, string rover);
    void writeData(const string& file);
    void readCsv(const string& file);
    void downsampling(double timeInterval);

    vector<double> baseCoord;
    vector<dateTime> time;
    vector<vector<double>> data;
    string baseName;
    string roverName;
};

class baseCoords{
public:
    void readData(string filename);
    void logBaseCoords();
    vector<string> name;
    vector<vector<double>> data;
    map<string, vector<double>> coordsMap;
};

vector<string> split_str(const string &s, const string &seperator);

// 写入数据
template<typename T>
void writeData(T& data, const string& filename){
    ofstream ofile;
    ofile.open(filename);
    for(const auto&  i :data){
        for(const auto j:i){
            ofile << j <<",";
        }
        ofile << endl;
    }
    ofile.close();
}
// 去除最短的数据段
template<typename T>
int filter(T& data){
    map<int ,int> rec;
    vector<int> l;
    for(int i=0;i<data.size();){
        int v = data[i];
        int i0=i;
        while(i<data.size() && data[i]==v) i++;
        rec[i-i0] = i0;
    }
    for(auto i:rec){
        if(i.first >200) return 0;
        if(data[i.second]==1) continue;
        for(int j=i.second;j<i.second+i.first;j++){
            data[j] = 1-data[j];
        }
        return 1;
    }
    return 1;
}


// 计算运动和静止状态的分界线
template<typename T>
void findStateChange(T& data, int idx, double threshHold,
                     vector<vector<int>>& interval){
    vector<int> start, stop;
    auto& t = data.time;
    auto& d = data.data;
    vector<double> dx(t.size()-1, 0);
    for(int i = 1;i<t.size();i++){
        dx[i-1] = (d[i][idx] - d[i-1][idx]);// / (t[i] - t[i-1]);
    }
    for(double & i : dx){
        i = abs(i)<threshHold?0:1;
    }
    while(filter(dx)==1);
    writeData(d, "../data/diff.csv");

    for(int i=1;i<dx.size();i++){
        if(dx[i-1]==1&&dx[i]==0){
            stop.push_back(i);
        }
        else if(dx[i-1]==0&&dx[i]==1){
            start.push_back(i-1);
        }
    }
    if(start.size()==0) return;
    if(start[0]>stop[0]){
        start.insert(start.begin(), 0);
    }
    for(int i=0;i<start.size()&&i<stop.size();i++){
        interval.push_back(vector<int>{start[i], stop[i]});
    }
    for(int i=0;i<interval.size()-1;i++){
        if(interval[i+1][0]-interval[i][1]<2000){
            interval[i][1] = interval[i+1][1];
            interval.erase(interval.begin()+i+1);
            i--;
        }
    }
}



// 按时间对数据进行插值
template<typename T1, typename T2, typename T3>
void combineData(T1* data1, T2* data2, T3* res){
    auto& d1 = data1->data;
    auto& d2 = data2->data;
    auto& t1 = data1->time;
    auto& t2 = data2->time;
    int idx1 = 0, idx2 = 1;

    while(idx1<t1.size()){
        while(idx1<t1.size() && t1[idx1] - t2[idx2] < 0) idx1++;
//        cout << "t1 time:" << t1[idx1] << endl;
        while(idx2<t2.size()){
            if( t1[idx1] - t2[idx2-1]>=0  && t2[idx2]-t1[idx1]>=0)
                break;
            idx2++;
        }
//        cout << "t2 time:" << t2[idx2] << endl;
        if(idx2<t2.size()){
            vector<double> d;
            for(auto i:d1[idx1]){
                d.push_back(i);
            }
            double dt1, dt2;
            dt1 = t1[idx1] - t2[idx2-1];
            dt2 = t2[idx2] - t1[idx1];
            for(int i=0; i<d2[idx2].size();i++){
                double dd = d2[idx2-1][i]*dt2 + d2[idx2][i]*dt1;
                if(dt1+dt2!=0){
                    dd /= (dt1+dt2);
                }
                else{
                    dd = d2[idx2][i];
                }
                d.push_back(dd);
            }
            res->time.push_back(t1[idx1]);
            res->data.push_back(d);
            idx2++;
            idx1++;
        }
        else{
            break;
        }
    }
}

void calCoord(imuLcjData& imulcj, vector<imuCoord>& res);

void transCoord(vector<vector<double>>& coords, vector<vector<double>> &staticCoord, int idx);

void simulate(baseLine &res, imuCoord &coords, double error);
#endif //HIGHSPEEDRAILWAYDATA_HIGHSPEEDRAILWAY_H
