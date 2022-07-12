#include "pid.h"
pid::pid(double tt, double pp, double ii, double dd){
    t = tt;
    set_pid(pp,ii,dd);
}

void pid::set_pid(double pp,double ii, double dd) {
    p = pp;
    i = ii;
    d = dd;
}

double pid::ctrl(double req, double cur) {
    double res;
    e = req - cur;
    sum += t*e;
    res = p*e + d*(e - prev)/t + sum*i;
    prev  = e;
    return res;
}
void pid::reset(){
    e = 0;
    prev = 0;
    sum = 0;
}