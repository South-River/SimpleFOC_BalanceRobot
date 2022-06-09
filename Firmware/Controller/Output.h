#ifndef OUTPUT_H
#define OUTPUT_H

#include <Arduino.h>
#include <IMU.h>

void printRPY(const IMU::IMU &imu);

void printQuaternion(const IMU::IMU &imu);

void printAcc(const IMU::IMU &imu);

void printGyro(const IMU::IMU &imu);

#endif