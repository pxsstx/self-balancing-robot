#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>
#include <IRremote.h>  // Add IR remote support

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

// MPU DMP variables
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// PID variables
// Original setpoint (default balance point)
double originalSetpoint = -2.3;
double setpoint = originalSetpoint;

unsigned long lastIRTime = 0;
const unsigned long resetDelay = 200;  // 0.2 second

double input, output;
double Kp = 70;
double Ki = 380;
double Kd = 1.4;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Motor speed adjust factors
double motorSpeedFactorLeft = 1.0;
double motorSpeedFactorRight = 1.0;

// For IR remote control
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

  irrecv.enableIRIn();  // Start the IR receiver
}

void moveMotors(double pidOutput, int minSpeed) {
  int speed = abs(pidOutput);
  speed = map(speed, 0, 255, minSpeed, 255);

  int leftSpeed = speed * motorSpeedFactorLeft;
  int rightSpeed = speed * motorSpeedFactorRight;

  if (pidOutput > 55) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, leftSpeed);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, rightSpeed);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftSpeed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightSpeed);
  }
}

void handleIRInput() {
  if (irrecv.decode(&results)) {
    switch (results.value) {
      case 0xFF18E7:  // UP button
        setpoint += 0.5;
        lastIRTime = millis();  // reset timer
        break;
      case 0xFF4AB5:  // DOWN button
        setpoint -= 0.5;
        lastIRTime = millis();  // reset timer
        break;
    }
    irrecv.resume();
  }
}


void loop() {
  if (!dmpReady) return;

  handleIRInput();

  // Reset setpoint after 0.2s without input
  if (millis() - lastIRTime > resetDelay && setpoint != originalSetpoint) {
    setpoint = originalSetpoint;
  }

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    input = ypr[2] * 180 / M_PI;
    pid.Compute();
    moveMotors(output, MIN_ABS_SPEED);

    Serial.print("Roll Angle: ");
    Serial.print(input);
    Serial.print(" | PID Output: ");
    Serial.print(output);
    Serial.print(" | Setpoint: ");
    Serial.println(setpoint);
  }
}
