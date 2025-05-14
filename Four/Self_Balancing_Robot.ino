#include <Wire.h>
#include <MPU6050_tockn.h>
#include <PID_v1.h>

// Motor Pins
#define IN1 8
#define IN2 9
#define ENA 6
#define ENB 5
#define IN3 13
#define IN4 12

#define MIN_ABS_SPEED 30

// MPU6050
MPU6050 mpu(Wire);

// PID
double setpoint = 0;  // target balance angle
double input, output;

double Kp = 80;
double Ki = 320;
double Kd = 1;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Motor Speed Factors
double motorSpeedFactorLeft = 1.0;
double motorSpeedFactorRight = 0.9;

double x_offset = -1.70, y_offset = 0.98, z_offset = -1.44;

// Setup
void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.begin();
  // mpu.calcGyroOffsets(true);  // auto-calibrate
  mpu.setGyroOffsets(x_offset, y_offset, z_offset);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);
}

// Motor control logic
void moveMotors(double pidOutput, int minSpeed) {
  int speed = abs(pidOutput);
  speed = map(speed, 0, 255, minSpeed, 255);  // Ensure minimum speed

  int leftSpeed = speed * motorSpeedFactorLeft;
  int rightSpeed = speed * motorSpeedFactorRight;

  if (pidOutput > 55) {

    // Tilted backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, leftSpeed);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, rightSpeed);
  } else {

    // Tilted forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftSpeed);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightSpeed);
  }
}

// Main loop
void loop() {
  mpu.update();
  input = mpu.getAngleX();  // Roll angle (side-to-side)

  pid.Compute();
  moveMotors(output, MIN_ABS_SPEED);

  // Debugging
  Serial.print("Angle: ");
  Serial.print(input);
  Serial.print(" | Output: ");
  Serial.println(output);

  delay(10);
}
