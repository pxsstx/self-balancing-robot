#include <Wire.h>
#include <MPU6050_tockn.h>
#include <PID_v1.h>
#include "motorcontrol.h"
#include "IRController.h"

#define MIN_ABS_SPEED_A 0
#define MIN_ABS_SPEED_B 0

#define IN1 8
#define IN2 9
#define ENA 6
#define ENB 5
#define IN3 13
#define IN4 12

MPU6050 mpu6050(Wire);
IRController ir(4);

// PID for balance 
double angle_input, motor_output, angle_setpoint = 1.2;
double Kp_bal = 40, Ki_bal = 320, Kd_bal = 1;
PID balancePID(&angle_input, &motor_output, &angle_setpoint, Kp_bal, Ki_bal, Kd_bal, DIRECT);

// PID for velocity
double velocity_input = 0, velocity_output = 0, velocity_setpoint = 0;
double Kp_vel = 5.0, Ki_vel = 0.3, Kd_vel = 0.0;
PID velocityPID(&velocity_input, &velocity_output, &velocity_setpoint, Kp_vel, Ki_vel, Kd_vel, DIRECT);


// motor
double speedOffset = 0, trunOffset = 0;
motorcontrol my_motor(ENA, IN1, IN2, ENB, IN3, IN4, 1.0, 1.0);  // SpeedFactorA, SpeedFactorB

unsigned long last_time = 0;
unsigned long commandTimeout = 200;

double x_offset = -1.90, y_offset = 0.98, z_offset = -1.44;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.setGyroOffsets(x_offset, y_offset, z_offset);

  my_motor.begin();

  balancePID.SetMode(AUTOMATIC);
  balancePID.SetSampleTime(10);
  balancePID.SetOutputLimits(-255, 255);

  velocityPID.SetMode(AUTOMATIC);
  velocityPID.SetSampleTime(10);
  velocityPID.SetOutputLimits(-10, 10); // ปรับเป้าหมายมุมไม่เกิน ±10 องศา

  ir.begin();
  Serial.println("Ready!");
  delay(2000);
}

void loop() {
  mpu6050.update();
  unsigned long now = millis();
  double gyroX = mpu6050.getGyroX();      // ใช้เป็นตัวแทนความเร็ว
  double angle = mpu6050.getAngleX();

  // --- Velocity PID ---
  velocity_input = -gyroX;
  velocityPID.Compute();
  angle_setpoint = 1.0 + velocity_output;  // 1.0 คือมุมสมดุลปกติ

  // --- Balance PID ---
  angle_input = angle;
  balancePID.Compute();

  // --- ตรวจ IR Command ---
  if (ir.available()) {
    IRCommand cmd = ir.getCommand();
    ir.resume();
    ir.updateLastCommandTime();

    switch (cmd) {
      case IRCommand::Forward:
        velocity_setpoint = 20.0;  // เดินหน้า
        break;
        
      case IRCommand::Backward:
        velocity_setpoint = -20.0; // ถอยหลัง
        break;
        
     case IRCommand::Left:
        trunOffset = -50; // เลี้ยวซ้าย
        break;
        
      case IRCommand::Right:
        trunOffset = 50; // เลี้ยวขวา
        break;
        
      default:
        break;
    }
  }
  
  // กลับสู่สภาวะ Default ถ้าไม่มีคำสั่ง IR มานานเกินไป
  if (now - ir.getLastCommandTime() > commandTimeout) {
    velocity_setpoint = 0;
    trunOffset = 0;
  }

  // --- ควบคุมมอเตอร์ ---
  my_motor.move(motor_output, speedOffset, trunOffset, MIN_ABS_SPEED_A, MIN_ABS_SPEED_B);

  // --- Debug Serial ---
  Serial.print("Angle: "); Serial.print(angle);
  Serial.print(" | Setpoint: "); Serial.print(angle_setpoint);
  Serial.print(" | Motor PWM: "); Serial.print(motor_output);
  Serial.print(" | GyroX: "); Serial.print(gyroX);
  Serial.print(" | VelOut: "); Serial.println(velocity_output);

  delay(10);
}