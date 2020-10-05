//
// Created by xhh14 on 2020/9/22.
//

#include "highSpeedRailway.h"

void imuLcjData::toCsv(string file) {
    ofstream ofile;
    ofile.open(file, ios::out);
    auto intervalI = interval.begin();
    for(auto i:itemName){
        ofile<<i<<",";
    }
    ofile<<endl;
    for(int i=0;i<time.size();i++){

        ofile << time[i] << ",";
        for(double j : data[i]){
            ofile << j << ",";
        }
        ofile << endl;
    }
    ofile.close();
}

