#ifndef MPU6050_H
#define MPU6050_H

#include "vector_3d.h"

void mpu6050_init();
vector_3d mpu6050_read_acceleration();
vector_3d mpu6050_read_gyro();

#endif