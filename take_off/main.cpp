#include <iostream>
#include <cmath>
#include "pid.h"

//класс которого нет в редакторе
class servo{
public:
  void write(int pw);
};

//get_hight(...) and get_pitch(...) in INS_plug.cpp

//прототипы функций оторые должны быть написаны нами
double GY();    //перегрука по оси у (вдоль крыла) в ед g
void set_coordinates();//считывает коодинаты по gps

//прототипы функций которых нет в редакторе
void analogWrite(int a, int b);
void delay(int t);
void tone(int a ,int b ,int c);

//глобальные переменнные
int t = 10;  //time step
int mtp = 0; //motor pin
int piezoPin = 3;
servo ser_l;
servo ser_r;
servo ser_pitch;


// процедура автономного взлета
void takeoff(double axx, double h ,double l){
    pid pitch(t,1,2,3); //создаем объекты класса пид регулятора
    pid roll(t,1,2,3);

    double rp = atan(h/l);         //rp - required pitch
    analogWrite(mtp,245);       //engine start
    delay(3000);                   //we wait 3 sec, the propeller spins up

    tone(piezoPin, 1000, 500); //будут поданы три гудка
    tone(piezoPin, 1500, 500);
    tone(piezoPin, 2000, 500);
    delay(100);
    while (get_hight(/*current bar data*/) < h)
    {
        ser_pitch.write(pitch.ctrl(rp, get_pitch(/*current params*/)));
        ser_l.write(roll.ctrl(0,GY()));
        ser_r.write(-roll.ctrl(0,GY()));
    }
    pitch.reset();
    set_coordinates(); //считаем координаты, которые будем использовать в следующем режиме

    for(int i=0; i<30; i++){ //short distance straight flight
        delay(t);
        ser_pitch.write(pitch.ctrl(0, get_pitch(/*current params*/)));
        ser_l.write(roll.ctrl(0,GY()));
        ser_r.write(-roll.ctrl(0,GY()));
    }

}
int main() {
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
