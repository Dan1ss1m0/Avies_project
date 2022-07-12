#pragma once
class pid {
private:
    double prev = 0; //previous error value
    double p = 0;    //p, i, d - coefficients
    double i = 0;
    double d =0;
    double sum=0;    //integral sum
    double e = 0;    //error (difference of current and required values)
    double t;       //time step
public:
    pid(double tt, double pp, double ii, double dd);     //it is necessary to define time step
    double ctrl(double req,double cur); // req - required; cur - current;
    void set_pid(double pp,double ii, double dd);
    void reset();
};



