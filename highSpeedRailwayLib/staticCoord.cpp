//
// Created by xhh14 on 2020/9/27.
//

#include "highSpeedRailway.h"

void staticCoord::readData(const string& file) {
    ifstream ifile;
    ifile.open(file);
    string buff;
    while (getline(ifile, buff)){
        auto values = split_str(buff, ",");
        vector<double> coord;
        for(int i=0;i<3&&i<values.size();i++){
            coord.push_back(stof(values[i]));
        }
        data.push_back(coord);
    }
}
