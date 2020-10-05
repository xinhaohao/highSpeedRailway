//
// Created by xhh14 on 2020/9/22.
//

#include "highSpeedRailway.h"

// 航向角、俯仰角、横滚角
void imuData::readData(string filePath) {
    ifstream inFile;
    inFile.open(filePath);
    string buff;
    while(getline(inFile, buff)){
        dateTime t{};
        double ep[6];
        auto items = split_str(buff, ",");
        if(stoi(items[2])==0) continue;

        if(items.size()<6){
            cerr<<"error in read imu data:" << buff <<endl;
            continue;
        }
        auto eps = split_str(items[0], "- ");
        if(eps.size()<7){
            cerr<<"error in read imu data date:"<<items[0]<<endl;
            continue;
        }

        for(int i=0;i<6;i++){
            ep[i] = stof(eps[i]);
        }
        ep[5]+=stof(eps[6]) / 1e3;
        t.setTime(ep);
        vector<double> item(3, 0);
        for(int i=0;i<3;i++){
            item[i] = stof(items[3+i]);
        }
        this->data.push_back(item);
        this->time.push_back(t);
    }
    cout << "data num:" << this->time.size() << endl;
}
