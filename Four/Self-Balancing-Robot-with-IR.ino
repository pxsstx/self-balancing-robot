#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>
#include <IRremote.h>

// IR Receiver Pin
#define IR_RECEIVE_PIN 4

// Motor Pins
#define IN1 8
#define IN2 9
#define ENA 6
#define IN3 13
#define IN4 12
#define ENB 5

#define MIN_ABS_SPEED 0

// MPU6050
MPU6050 mpu;
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// PID
double originalSetpoint = -2.3;
double setpoint = originalSetpoint;
double input, output;
double Kp = 70, Ki = 380, Kd = 1.4;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Timer
unsigned long lastIRTime = 0;
const unsigned long resetDelay = 200;  // 0.2 sec

// Motor Speed Factors
double motorSpeedFactorLeft = 1.0;
double motorSpeedFactorRight = 1.0;

// IR
IRrecv irrecv(IR_RECEIVE_PIN);
decode_results results;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(-1.70);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(1688);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println(F("DMP ready!"));
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);

  irrecv.enableIRIn();
}

void moveMotors(double pidOutput, int minSpeed) {
  int speed = abs(pidOutput);
  speed = map(speed, 0, 255, minSpeed, 255);

  int leftSpeed = speed * motorSpeedFactorLeft;
  int rightSpeed = speed * motorSpeedFactorRight;

  if (pidOutput > 55) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, rightSpeed);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, leftSpeed);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, rightSpeed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, leftSpeed);
  }
}

void handleIRInput() {
  if (irrecv.decode(&results)) {
    switch (results.value) {
      case 0xFF18E7:  // UP
        setpoint += 5.5;
        lastIRTime = millis();
        break;

      case 0xFF4AB5:  // DOWN
        setpoint -= 3.5;
        lastIRTime = millis();
        break;

      case 0xFF10EF:  // LEFT
        motorSpeedFactorLeft = 0.7;
        motorSpeedFactorRight = 1.0;
        setpoint += 5.5;
        break;

      case 0xFF5AA5:  // RIGHT
        motorSpeedFactorLeft = 1.0;
        motorSpeedFactorRight = 0.7;
        setpoint += 5.5;
        break;

      case 0xFF38C7:  // OK
        motorSpeedFactorLeft = 1.0;
        motorSpeedFactorRight = 1.0;
        setpoint += 5.5;
        break;
    }
    irrecv.resume();
  }
}

void loop() {
  if (!dmpReady) return;

  handleIRInput();

  // Reset setpoint if idle
  if (millis() - lastIRTime > resetDelay && setpoint != originalSetpoint) {
    setpoint = originalSetpoint;
  }

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    input = ypr[2] * 180 / M_PI;

    // 🔒 Reset immediately if about to fall
    if (input > 10 || input < -10) {
      setpoint = originalSetpoint;
      motorSpeedFactorLeft = 1.0;
      motorSpeedFactorRight = 1.0;
    }

    pid.Compute();
    moveMotors(output, MIN_ABS_SPEED);

    // Debug
    Serial.print("Roll: ");
    Serial.print(input);
    Serial.print(" | PID: ");
    Serial.print(output);
    Serial.print(" | Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" | L-Factor: ");
    Serial.print(motorSpeedFactorLeft);
    Serial.print(" | R-Factor: ");
    Serial.println(motorSpeedFactorRight);
  }
}
