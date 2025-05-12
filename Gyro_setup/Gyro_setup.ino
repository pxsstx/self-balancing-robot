#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  delay(1000);

  // วัดค่า gyro หลาย ๆ ครั้งเก็บค่าเฉลี่ย
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  const int num_readings = 500;

  for (int i = 0; i < num_readings; i++) {
    mpu6050.update();
    gx_sum += mpu6050.getGyroX();
    gy_sum += mpu6050.getGyroY();
    gz_sum += mpu6050.getGyroZ();
    delay(5); // รอให้อ่านได้เสถียร
  }

  float gx_offset = gx_sum / (float)num_readings;
  float gy_offset = gy_sum / (float)num_readings;
  float gz_offset = gz_sum / (float)num_readings;

  Serial.println("Gyro Offsets:");
  Serial.print("X: "); Serial.println(gx_offset);
  Serial.print("Y: "); Serial.println(gy_offset);
  Serial.print("Z: "); Serial.println(gz_offset);

  // ตั้ง offset ที่หาได้
  mpu6050.setGyroOffsets(gx_offset, gy_offset, gz_offset);
}

void loop() {
  mpu6050.update();
  Serial.print("GyroX: ");
  Serial.print(mpu6050.getGyroX());
  Serial.print("\tGyroY: ");
  Serial.print(mpu6050.getGyroY());
  Serial.print("\tGyroZ: ");
  Serial.println(mpu6050.getGyroZ());
  delay(100);
}
