#pragma once
#if defined(USE_GYRO)
#include <Wire.h>
#include "logging.h"

#include "device.h"
// #include "common.h"

// #include "MPU6050.h"
// #include "MPU6050_6Axis_MotionApps20.h"

extern device_t Gyro_device;
uint16_t rateController(uint8_t ch, uint16_t us);

     
// MPU6050 mpu = MPU6050();
	 

#endif