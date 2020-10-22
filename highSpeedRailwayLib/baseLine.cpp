//
// Created by xhh14 on 2020/9/28.
//

#include "highSpeedRailway.h"

void baseLine::writeData(const string& file){
    ofstream ofile;
    ofile.open(file);
    for(int i=0;i<time.size();i++){
        ofile << time[i];
        for(auto j:data[i]){
            ofile << "," << j;
        }
        ofile << endl;
    }
}


void baseLine::readCsv(const string& file){
    ifstream ifile;
    ifile.open(file);
    if(!ifile.is_open()){
        cerr << "error in open file: " << file << endl;
        return;
    }
    string buff;
    while(getline(ifile, buff)){
        auto items = split_str(buff, ",");
        auto str_eps = split_str(items[0], " -:");
        double eps[6];
        for(int i=0;i<6&i<str_eps.size();i++){
            eps[i] = stof(str_eps[i]);
        }
        time.emplace_back();
        time.back().setTime(eps);

        data.emplace_back();
        auto &thd = data.back();
        for(int i=1;i<items.size();i++){
            thd.push_back(stof(items[i]));
        }
    }
    cout << "file:" << file << endl;
    cout << "baseline::readcsv\n";
    cout << "time size: " << time.size() << " coord size: " << data.size() << endl;
    ifile.close();
}

void baseLine::downsampling(double timeInterval){
    cout << "before downsamplint, data num:" << time.size() << "\t" << data.size() << endl;
    vector<vector<double>> data2;
    vector<dateTime> time2;
    data2.swap(data);
    time2.swap(time);
    int i=0;
    dateTime t;
    t = time2[0];
    while(i<time2.size()){
        if(time2[i]-t>timeInterval){
            time.push_back(time2[i]);
            data.push_back(data2[i]);
            t = time2[i];
        }
        i++;
    }
    cout << "after downsamplint, data num:" << time.size() << "\t" << data.size() << endl;
}
