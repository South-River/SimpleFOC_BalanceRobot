#include "Output.h"

void printRPY(const IMU::IMU &imu)
{
  Serial.print("roll: ");  Serial.print(imu.roll*IMU::RAD2ANG);  Serial.print("\t");
  Serial.print("pitch: "); Serial.print(imu.pitch*IMU::RAD2ANG); Serial.print("\t");
  Serial.print("yaw: ");   Serial.print(imu.yaw*IMU::RAD2ANG);   Serial.print("\t");
  Serial.print("\n");
}

void printQuaternion(const IMU::IMU &imu)
{
  Serial.print("w: "); Serial.print(imu.q[0]); Serial.print("\t");
  Serial.print("x: "); Serial.print(imu.q[1]); Serial.print("\t");
  Serial.print("y: "); Serial.print(imu.q[2]); Serial.print("\t");
  Serial.print("z: "); Serial.print(imu.q[3]); Serial.print("\t");
  Serial.print("\n");
}

void printAcc(const IMU::IMU &imu)
{
  Serial.print("ax: "); Serial.print(imu.accel[0]); Serial.print("\t");
  Serial.print("ay: "); Serial.print(imu.accel[1]); Serial.print("\t");
  Serial.print("az: "); Serial.print(imu.accel[2]); Serial.print("\t");
  Serial.print("\n");
}

void printGyro(const IMU::IMU &imu)
{
  Serial.print("gx: "); Serial.print(imu.gyro[0]*IMU::RAD2ANG); Serial.print("\t");
  Serial.print("gy: "); Serial.print(imu.gyro[1]*IMU::RAD2ANG); Serial.print("\t");
  Serial.print("gz: "); Serial.print(imu.gyro[2]*IMU::RAD2ANG); Serial.print("\t");
  Serial.print("\n");
}
