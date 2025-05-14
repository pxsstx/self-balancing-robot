#include "motorcontrol.h"
#include "Arduino.h"

motorcontrol::motorcontrol(int ena, int in1, int in2, int enb, int in3, int in4, double motorAConst, double motorBConst) {
    _motorAConst = motorAConst; // ตัวคูณความเร็วของมอเตอร์ A
    _motorBConst = motorBConst;

    _ena = ena;
    _in1 = in1;
    _in2 = in2;
    _enb = enb;
    _in3 = in3;
    _in4 = in4;
}

void motorcontrol::begin() {
    pinMode(_ena, OUTPUT);
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);

    pinMode(_enb, OUTPUT);
    pinMode(_in3, OUTPUT);
    pinMode(_in4, OUTPUT);
}

void motorcontrol::_move(int speed, int minAbsSpeedA, int minAbsSpeedB, int bias) {
    int direction = (speed < 0) ? -1 : 1;
    speed = constrain(speed, -255, 255);

    // ถ้าความเร็วใหม่ต่างจากเดิมไม่ถึง 3 และไม่มี bias ก็ไม่ต้องขยับ
    if (abs(speed - _currentSpeed) < 3 && bias == 0) return;

    int baseSpeed = abs(speed);
    int speedA = baseSpeed + bias;
    int speedB = baseSpeed - bias;

    // ถ้ามีความเร็วต้องให้ความเร็วขั้นต่ำเป็น minAbs
    if (speedA != 0) speedA = max(abs(speedA), minAbsSpeedA);
    if (speedB != 0) speedB = max(abs(speedB), minAbsSpeedB);

    // ปรับไม่ให้เกิน 255
    speedA = constrain(speedA, 0, 255);
    speedB = constrain(speedB, 0, 255);

    // ควบคุมทิศทางทั้งสองล้อให้ไปทางเดียวกันตาม direction
    digitalWrite(_in1, direction > 0 ? HIGH : LOW);
    digitalWrite(_in2, direction > 0 ? LOW : HIGH);
    digitalWrite(_in3, direction > 0 ? HIGH : LOW);
    digitalWrite(_in4, direction > 0 ? LOW : HIGH);

    // ใส่ตัวคูณแล้วปรับไม่ให้เกิน 255
    analogWrite(_ena, constrain(speedA * _motorAConst, 0, 255));
    analogWrite(_enb, constrain(speedB * _motorBConst, 0, 255));

    _currentSpeed = direction * baseSpeed;
}

void motorcontrol::move(int speed, int speedoffset, int turnOffset, int minAbsSpeedA, int minAbsSpeedB) {
    int controlSpeed = speed + speedoffset;
    _move(controlSpeed, minAbsSpeedA, minAbsSpeedB, turnOffset);
}

void motorcontrol::turnLeft(int speed) {
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, LOW);
    digitalWrite(_in3, LOW);
    digitalWrite(_in4, HIGH);

    analogWrite(_ena, constrain(speed * _motorAConst, 0, 255));
    analogWrite(_enb, constrain(speed * _motorBConst, 0, 255));
}

void motorcontrol::turnRight(int speed) {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, HIGH);
    digitalWrite(_in3, HIGH);
    digitalWrite(_in4, LOW);

    analogWrite(_ena, constrain(speed * _motorAConst, 0, 255));
    analogWrite(_enb, constrain(speed * _motorBConst, 0, 255));
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