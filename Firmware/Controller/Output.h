#ifndef OUTPUT_H
#define OUTPUT_H

#include <Arduino.h>
#include <IMU.h>

void printRPY(const IMU::IMU_6DOF &imu);

void printQuaternion(const IMU::IMU_6DOF &imu);

void printAcc(const IMU::IMU_6DOF &imu);

void printGyro(const IMU::IMU_6DOF &imu);

#endif
