#include "Output.h"

void printRPY(const IMU::IMU &imu)
{
  String str="";
  
  str+=("roll: "+String(imu.roll*IMU::RAD2ANG,3)+"\t");
  str+=("pitch: "+String(imu.pitch*IMU::RAD2ANG,3)+"\t");
  str+=("yaw: "+String(imu.yaw*IMU::RAD2ANG,3)+"\t");
  
  Serial.println(str);
}

void printQuaternion(const IMU::IMU &imu)
{
  String str="";
  
  str+=("w: "+String(imu.q[0],3)+"\t");
  str+=("x: "+String(imu.q[1],3)+"\t");
  str+=("y: "+String(imu.q[2],3)+"\t");
  str+=("z: "+String(imu.q[3],3)+"\t");
  
  Serial.println(str);
}

void printAcc(const IMU::IMU &imu)
{
  String str="";
  
  str+=("ax: "+String(imu.accel[0],3)+"\t");
  str+=("ay: "+String(imu.accel[1],3)+"\t");
  str+=("az: "+String(imu.accel[2],3)+"\t");
  
  Serial.println(str);
}

void printGyro(const IMU::IMU &imu)
{
  String str="";
  
  str+=("gx: "+String(imu.gyro[0]*IMU::RAD2ANG,3)+"\t");
  str+=("gy: "+String(imu.gyro[1]*IMU::RAD2ANG,3)+"\t");
  str+=("gz: "+String(imu.gyro[2]*IMU::RAD2ANG,3)+"\t");
  
  Serial.println(str);
}
