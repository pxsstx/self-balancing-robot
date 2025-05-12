#ifndef motorcontrol_h
#define motorcontrol_h


#include "Arduino.h"


class motorcontrol
{
protected:
    int _ena, _in1, _in2, _enb, _in3, _in4;
    int _currentSpeed;
    double _motorAConst, _motorBConst;
    void _move(int speed, int minAbsSpeed, int bias = 0);
public:
    motorcontrol(int ena, int in1, int in2, int enb, int in3, int in4, double motorAConst, double motorBConst);
    void begin();
    void stop();
    void turnLeft(int speed);
    void turnRight(int speed);
    void move(double speed, double speedoffset, double turnOffset, int minAbsSpeedA, int minAbsSpeedB);

};


#endif // motorcontrol_h
