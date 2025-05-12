#include "motorcontrol.h"
#include "Arduino.h"


motorcontrol::motorcontrol(int ena, int in1, int in2, int enb, int in3, int in4, double motorAConst, double motorBConst){
    _motorAConst = motorAConst; // ตัวคูณความเร็วของมอเตอร์ A
    _motorBConst = motorBConst; 
    
  _ena = ena;
  _in1 = in1;
  _in2 = in2;
  _enb = enb;
  _in3 = in3;
  _in4 = in4;
}

void motorcontrol::begin(){
  pinMode(_ena, OUTPUT);
  pinMode(_in1, OUTPUT);
  pinMode(_in2, OUTPUT);
    
  pinMode(_enb, OUTPUT);
  pinMode(_in3, OUTPUT);
  pinMode(_in4, OUTPUT);
}

void motorcontrol::_move(int speed, int minAbsSpeed, int bias)
{
    int direction = 1; // ตัวคูณถ้าไปหน้าเป็น + ถอยหลังเป็น -
    
    if (speed < 0)
    {
        direction = -1;
        speed = min(speed, -minAbsSpeed);
        speed = max(speed, -255);
    }
    else
    {
        speed = max(speed, minAbsSpeed);
        speed = min(speed, 255);
    }

    // ถ้าความเร็วเปลี่ยนไม่เยอะกับไม่มีการเลี้ยว ไม่ต้องสั่งมอเตอร์
    if (abs(speed - _currentSpeed) < 3 && bias == 0) return;

    int baseSpeed = abs(speed);
    int leftSpeed = baseSpeed + bias;
    int rightSpeed = baseSpeed - bias;

    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    digitalWrite(_in1, speed > 0 ? HIGH : LOW);
    digitalWrite(_in2, speed > 0 ? LOW : HIGH);
    digitalWrite(_in3, speed > 0 ? HIGH : LOW);
    digitalWrite(_in4, speed > 0 ? LOW : HIGH);

    analogWrite(_ena, leftSpeed * _motorAConst);
    analogWrite(_enb, rightSpeed * _motorBConst);

    _currentSpeed = direction * baseSpeed;
}



void motorcontrol::turnLeft(int speed)
{
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, LOW);
    digitalWrite(_in3, LOW);
    digitalWrite(_in4, HIGH);
    
    analogWrite(_ena, speed * _motorAConst);
    analogWrite(_enb, speed * _motorBConst);
}


void motorcontrol::turnRight(int speed)
{
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, HIGH);
    digitalWrite(_in3, HIGH);
    digitalWrite(_in4, LOW);
    
    analogWrite(_ena, speed * _motorAConst);
    analogWrite(_enb, speed * _motorBConst);
}


void motorcontrol::move(double speed, double speedoffset, double turnOffset, int minAbsSpeedA, int minAbsSpeedB)
{
    int controlSpeed = speed + speedoffset;
    int bias = turnOffset;

    // Combine or use either minAbsSpeedA/B as needed, or pass both to a new _move()
    int minAbsSpeed = max(minAbsSpeedA, minAbsSpeedB); // simple approach

    _move(controlSpeed, minAbsSpeed, bias);
}



void motorcontrol::stop() {
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, HIGH);
    digitalWrite(_in3, HIGH);
    digitalWrite(_in4, HIGH);
    analogWrite(_ena, 0);
    analogWrite(_enb, 0);
    _currentSpeed = 0;
}
