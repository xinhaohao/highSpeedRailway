//
// Created by xhh14 on 2020/9/22.
//

#include "highSpeedRailway.h"

double dateTime::gettick() {
    const int doy[]={1,32,60,91,121,152,182,213,244,274,305,335};
//    gtime_t time={0};
    if(tick!=0) return tick;
    int days;
    double time;
    if (year<1970||2099<year||month<1||12<month) return 0;

    /* leap year if year%4==0 in 1901-2099 */
    days=(year-1970)*365+(year-1969)/4+doy[month-1]+day-2+(year%4==0&&month>=3?1:0);
    time=days*86400+hour*3600+min*60+sec;
    tick = time;
    return time;
}

dateTime &dateTime::setTick(double ticks) {
    const int mday[]={ /* # of days in a month */
            31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
            31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
    };
    tick = ticks;
    int days,secs,mon,d;

    /* leap year if year%4==0 in 1901-2099 */
    days=(int)(ticks/86400);
    secs=(int)(ticks-days*86400);

    for (d= days % 1461, mon=0; mon < 48; mon++) {
        if (d >= mday[mon]) d-=mday[mon]; else break;
    }
    year=1970+days/1461*4+mon/12;
    month=mon%12+1;
    this->day = d + 1;
    hour=secs/3600;
    min=secs%3600/60;
    sec=secs%60+(ticks-(int)ticks);
    return *this;
}

dateTime *dateTime::operator+(dateTime d2) {
    auto* res = new dateTime();
    double ticks = this->tick+d2.tick;
    res->setTick(ticks);
    return res;
}

double dateTime::operator-(dateTime d2) {
    return this->gettick()-d2.gettick();
}

void dateTime::operator+=(double d2) {
    this->setTick(this->gettick()+d2);
}

void dateTime::operator-=(double d2) {
    this->setTick(this->gettick()-d2);
}

void dateTime::setTime(double *ep) {
    year = (int) ep[0];
    month = (int) ep[1];
    day = (int) ep[2];
    hour = (int) ep[3];
    min = (int) ep[4];
    sec = ep[5];
    gettick();
}

ostream &operator<<(ostream &output, dateTime datetime) {
    output.precision(5);
    output << datetime.year << "-" << datetime.month << "-" << datetime.day << " ";
    output << datetime.hour << ":" << datetime.min << ":" << datetime.sec;
    return output;
}

