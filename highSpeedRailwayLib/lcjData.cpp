//
// Created by xhh14 on 2020/9/22.
//
#include "highSpeedRailway.h"
using namespace std;
void lcjData::readData(string file) {
    ifstream inFile;
    inFile.open(file);
    if(!inFile.is_open()){
        cerr << "error in open file:" << file << endl;
    }
    string  buff;
    while(getline(inFile, buff)){
        dateTime datetime{};
        auto items = split_str(buff, " ,-");
        if(items.size()<10) {
            cerr << "error in read lcjdata line:" << buff << endl;
            continue;
        }

        this->data.push_back(vector<double>{stof(items[1])});

        double ep[6];
        for(int i=0;i<6;i++){
            ep[i] = stof(items[3+i]);
        }
        ep[5] += stof(items[9]) / 1000;
        datetime.setTime(ep);
        this->time.push_back(datetime);
    }
}

