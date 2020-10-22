//
// Created by xhh14 on 2020/9/23.
//
#include "highSpeedRailway.h"
void imuCoord::toCsv(string filename) {
    ofstream ofile;
    ofile.setf(ios::fixed, ios::floatfield);
    ofile.precision(4);
    ofile.open(filename);
    for(auto i:itemName){
        ofile << i << ",";
    }
    ofile << endl;
    for(int i=0;i<time.size();i++){
        ofile << time[i];
        for(int j=0;j<data[i].size();j++){
            ofile << "," << data[i][j];
        }
        ofile << endl;
    }
    ofile.close();
}

void imuCoord::readCsv(string filename){
    time.clear();
    data.clear();
    ifstream ifile;
    ifile.open(filename);
    if(!ifile.is_open()){
        cerr << "error in open file:" << filename << endl;
        return;
    }
    string buff;
    getline(ifile, buff);
    if(buff.size()==0){
        cerr << "error in imucoord read csv\n";
        return;
    }
    if(buff.back() == '\n') buff.pop_back();
    itemName = split_str(buff, ",");
    
    while(getline(ifile, buff)){
        if(buff.back() == '\n') buff.pop_back();
        auto items = split_str(buff, ",");
        time.emplace_back();
        auto ts = split_str(items[0], "- :");
        double ep[6];
        for(int i=0;i<6&&i<ts.size();i++){
            ep[i] = stof(ts[i]);
        }
        time.back().setTime(ep);
        data.emplace_back();
        auto &theData = data.back();
        for(int i=1;i<items.size();i++){
            theData.push_back(stof(items[i]));
        }
    }
    ifile.close();
    cout << time[0] << endl;
    cout << time.back() << endl;
    cout << "imuCoords::readCsv:" << endl;
    cout << "time size:" << time.size() << " data size = " << data.size() << endl;
}

void imuCoord::downsampling(double timeInterval){
    cout << "before downsamplint, data num:" << time.size() << "\t" << data.size() << endl;
    vector<vector<double>> data2;
    vector<dateTime> time2;
    data2.swap(data);
    time2.swap(time);
    int i=0;
    dateTime t;
    t = time2[0];
    while(i<time2.size()){
        // cout << "time interval:" << time2[i] -t << endl;
        if(time2[i]-t>timeInterval){
            time.push_back(time2[i]);
            data.push_back(data2[i]);
            t = time2[i];
        }
        i++;
    }
    cout << "after downsamplint, data num:" << time.size() << "\t" << data.size() << endl;
}