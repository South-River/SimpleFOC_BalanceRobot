#include "Output.h"

void printRPY(const IMU::IMU_6DOF &imu)
{
  String str="";

  str+=("ROLL:  "+String(imu.roll*RAD2ANG,3)+"\t");
  str+=("PITCH: "+String(imu.pitch*RAD2ANG,3)+"\t");
  str+=("YAW:   "+String(imu.yaw*RAD2ANG,3)+"\t");
  
  Serial.println(str);
}

void printQuaternion(const IMU::IMU_6DOF &imu)
{
  String str="";
  
  str+=("w: "+String(imu.q[0],3)+"\t");
  str+=("x: "+String(imu.q[1],3)+"\t");
  str+=("y: "+String(imu.q[2],3)+"\t");
  str+=("z: "+String(imu.q[3],3)+"\t");
  
  Serial.println(str);
}

void printAcc(const IMU::IMU_6DOF &imu)
{
  String str="";
  
  str+=("ax: "+String(imu.accel[0],3)+"\t");
  str+=("ay: "+String(imu.accel[1],3)+"\t");
  str+=("az: "+String(imu.accel[2],3)+"\t");
  
  Serial.println(str);
}

void printGyro(const IMU::IMU_6DOF &imu)
{
  String str="";
  
  str+=("gx: "+String(imu.gyro[0]*RAD2ANG,3)+"\t");
  str+=("gy: "+String(imu.gyro[1]*RAD2ANG,3)+"\t");
  str+=("gz: "+String(imu.gyro[2]*RAD2ANG,3)+"\t");
  
  Serial.println(str);
}

String getRPYStr(const IMU::IMU_6DOF &imu, const String &split_label)
{
  String str="";

  str+=("ROLL:  "+String(imu.roll*RAD2ANG,2)+split_label);
  str+=("PITCH: "+String(imu.pitch*RAD2ANG,2)+split_label);
  str+=("YAW:   "+String(imu.yaw*RAD2ANG,2)+split_label);

  return str;
}

String getQuaternionStr(const IMU::IMU_6DOF &imu, const String &split_label)
{
  String str="";
  
  str+=("w: "+String(imu.q[0],3)+split_label);
  str+=("x: "+String(imu.q[1],3)+split_label);
  str+=("y: "+String(imu.q[2],3)+split_label);
  str+=("z: "+String(imu.q[3],3)+split_label);

  return str;
}

String getAccStr(const IMU::IMU_6DOF &imu, const String &split_label)
{
  String str="";
  
  str+=("ax: "+String(imu.accel[0],3)+split_label);
  str+=("ay: "+String(imu.accel[1],3)+split_label);
  str+=("az: "+String(imu.accel[2],3)+split_label);
  
  return str;
}

String getGyroStr(const IMU::IMU_6DOF &imu, const String &split_label)
{
  String str="";
  
  str+=("gx: "+String(imu.gyro[0]*RAD2ANG,3)+split_label);
  str+=("gy: "+String(imu.gyro[1]*RAD2ANG,3)+split_label);
  str+=("gz: "+String(imu.gyro[2]*RAD2ANG,3)+split_label);

  return str;
}
