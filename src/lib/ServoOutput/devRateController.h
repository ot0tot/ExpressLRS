#pragma once
#if defined(USE_GYRO)
#include "I2Cdev.h"
#include <Wire.h>
#include "logging.h"

#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

// #define INTERRUPT_PIN 22
#define PIN_SDA 22
#define PIN_CLK 19

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
     
MPU6050 mpu = MPU6050();
	 
// static void initI2C(void *ignore) {
static void initI2C() {
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)PIN_SDA;
	DBGLN("PIN_SDA: %d", PIN_SDA);
	conf.scl_io_num = (gpio_num_t)PIN_CLK;
	DBGLN("PIN_CLK: %d", PIN_CLK);
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}


// static void initialize_mpu(void*){
static void initialize_mpu(){
	// MPU6050 mpu = MPU6050();
	DBGLN("Initialize_mpu");
	mpu.initialize();
	mpu.dmpInitialize();

	
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

	mpu.setDMPEnabled(true);
}

// static void read_mpu(void*){
static void read_mpu(){	
/*	MPU6050 mpu = MPU6050();
	mpu.initialize();
	mpu.dmpInitialize();
	")
	
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

	mpu.setDMPEnabled(true);

	while(1){ */
	// DBGLN("Read_mpu");
	    mpuIntStatus = mpu.getIntStatus();
		
		fifoCount = mpu.getFIFOCount();

	    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
	        
	        mpu.resetFIFO();

	   
	    } else if (mpuIntStatus & 0x02) {
	        // wait for correct available data length
	        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

	        // read a packet from FIFO

	        mpu.getFIFOBytes(fifoBuffer, packetSize);
	 		mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			DBGLN("YAW: %f, ", ypr[0] * 180/M_PI);
			DBGLN("PITCH: %f, ", ypr[1] * 180/M_PI);
			DBGLN("ROLL: %f", ypr[2] * 180/M_PI);
			DBGLN("Quaternions");
			DBGLN("w: %f ", q.w);
			DBGLN("x: %f ", q.x);
			DBGLN("y: %f ", q.y);
			DBGLN("z: %f ", q.z);
	    }

	// }

}

#endif