#if defined(USE_GYRO)
#include "devRateController.h"
#include "logging.h"

#include "I2Cdev.h"
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "telemetry.h"
extern Telemetry telemetry;

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
VectorInt16 gyro;
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// #define gscale ((250./32768.0)*(M_PI/180.0))  //gyro default 250 LSB per d/s -> rad/s
#define gscale ((250./32768.0))  //gyro default 250 LSB per d/s

float A_cal[6] = {0.0, 0.0, 0.0, 1.000, 1.000, 1.000}; // 0..2 offset xyz, 3..5 scale xyz
float G_off[3] = { 0., 0., 0.}; //raw offsets, determined for gyro at rest
int cal_gyro = 1;  //set to zero to use gyro calibration offsets below.

#define PIN_SDA 22
#define PIN_CLK 19
#define I2C_MASTER_FREQ_HZ 400000

MPU6050 mpu = MPU6050();

static int16_t decidegrees2Radians10000(int16_t angle_decidegree)
{
	while (angle_decidegree > 1800) {
		angle_decidegree -= 3600;
	}
	while (angle_decidegree < -1800) {
		angle_decidegree += 3600;
	}
	return (int16_t)((M_PI / 180.0f) * 1000.0f * angle_decidegree);
}

// PID controller values
const float kP_ail = 0.1;  // Proportional gain
const float kI_ail = 0.01; // Integral gain
const float kD_ail = 0.01; // Derivative gain
const float maxRate_ail = 180.0; // Max roll rate in deg/s

float errorSum_ail = 0.0;
const float errorSum_ail_max = 10.0;
const float error_ail_max = 100.0;
float prevError_ail = 0.0;

const float kP_ele = 0.1;  // Proportional gain
const float kI_ele = 0.01; // Integral gain
const float kD_ele = 0.01; // Derivative gain

const float maxRate_ele = 90.0; // Max pitch rate in deg/s

float errorSum_ele = 0.0;
const float errorSum_ele_max = 10.0;
const float error_ele_max = 100.0;
float prevError_ele = 0.0;

uint16_t rateController(uint8_t ch, uint16_t us)
{
	mpuIntStatus = mpu.getIntStatus();
	fifoCount = mpu.getFIFOCount();
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
	        // DBGLN("Resetting FIFO");
	        mpu.resetFIFO();
	} else if (mpuIntStatus & 0x02) {
	        // wait for correct available data length
			while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
	        // read a packet from FIFO
	        mpu.getFIFOBytes(fifoBuffer, packetSize);
			mpu.dmpGetGyro(&gyro, fifoBuffer);
	 		mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
/*			DBGLN("YAW: %f, ", ypr[0] * 180/M_PI);
			DBGLN("PITCH: %f, ", ypr[1] * 180/M_PI);
			DBGLN("ROLL: %f", ypr[2] * 180/M_PI);
			DBGLN("Quaternions");
			DBGLN("w: %f ", q.w);
			DBGLN("x: %f ", q.x);
			DBGLN("y: %f ", q.y);
			DBGLN("z: %f ", q.z); */
			// DBGLN("Gyro x: %f", gyro.x * gscale);
			// DBGLN("Gyro y: %f", gyro.y * gscale);
			// DBGLN("Gyro z: %f", gyro.z * gscale);

			// Get yaw/pitch/roll in decidegrees and convert to uint16_t
			uint16_t ypr16[3] = {0};
			ypr16[0] = (uint16_t)(ypr[0] * 1800 / M_PI);
			ypr16[1] = (uint16_t)(ypr[1] * 1800 / M_PI);
			ypr16[2] = (uint16_t)(ypr[2] * 1800 / M_PI);

			
			CRSF_MK_FRAME_T(crsf_sensor_attitude_t) crsfAttitude = {0};
			// DBGLN("Pitch: %d", ypr16[1]);
			crsfAttitude.p.pitch = htobe16(decidegrees2Radians10000(ypr16[1]));
			// DBGLN("crsfAttitude.p.pitch: %d", crsfAttitude.p.pitch);
			crsfAttitude.p.roll = htobe16(decidegrees2Radians10000(ypr16[2]));
			// DBGLN("crsfAttitude.p.roll: %d", crsfAttitude.p.roll);
			crsfAttitude.p.yaw = htobe16(decidegrees2Radians10000(ypr16[0]));
			// DBGLN("crsfAttitude.p.yaw: %d", crsfAttitude.p.yaw);
			
			CRSF::SetHeaderAndCrc((uint8_t *)&crsfAttitude, CRSF_FRAMETYPE_ATTITUDE, CRSF_FRAME_SIZE(sizeof(crsf_sensor_attitude_t)), CRSF_ADDRESS_CRSF_TRANSMITTER);
			telemetry.AppendTelemetryPackage((uint8_t *)&crsfAttitude);
	    }


	if (ch == 0){ // Aileron channel
		DBGLN("ch0 %d, %d", ch, us);
		DBGLN("Desired rate: %f", ((us - 1500) * (maxRate_ail / 500)));
		DBGLN("Actual rate: %f", gyro.x * gscale);
		float error = ((us - 1500) * (maxRate_ail / 500)) - (gyro.x * gscale);
		DBGLN("Error: %f", error);
		if (error > error_ail_max) { error = error_ail_max; }
		if (error < -error_ail_max) { error = -error_ail_max; }

		errorSum_ail += error;
		if (errorSum_ail > errorSum_ail_max) { errorSum_ail = errorSum_ail_max; }
		if (errorSum_ail < -errorSum_ail_max) { errorSum_ail = -errorSum_ail_max; }
		
		float errorDiff = error - prevError_ail;
    	prevError_ail = error;
		float output = kP_ail * error + kI_ail * errorSum_ail + kD_ail * errorDiff;
		DBGLN("Output: %f", output);
		us += (uint16_t)output * (500 / maxRate_ail);
		// Constrain the control signal within the range 1000 to 2000
		if (us < 1000) {
			us = 1000;
		} else if (us > 2000) {
			us = 2000;
		}
		
		DBGLN("us %d", us);
	}
	else if (ch == 1){ // Elevator channel
		// DBGLN("ch1 %d, %d", ch, us);
		// DBGLN("Desired rate: %f", ((us - 1500) * (maxRate_ele / 500)));
		// DBGLN("Actual rate: %f", gyro.y * gscale);
		float error = ((us - 1500) * (maxRate_ele / 500)) - (gyro.y * gscale);
		if (error > error_ele_max) { error = error_ele_max; }
		if (error < -error_ele_max) { error = -error_ele_max; }
		// DBGLN("Error: %f", error);

		errorSum_ele += error;
		if (errorSum_ele > errorSum_ele_max) { errorSum_ele = errorSum_ele_max; }
		if (errorSum_ele < -errorSum_ele_max) { errorSum_ele = -errorSum_ele_max; }
		float errorDiff = error - prevError_ele;
    	prevError_ele = error;
		float output = kP_ele * error + kI_ele * errorSum_ele + kD_ele * errorDiff;
		// DBGLN("Output: %f", output);
		us += (uint16_t)output * (500 / maxRate_ele);
		// Constrain the control signal within the range 1000 to 2000
		if (us < 1000) {
			us = 1000;
		} else if (us > 2000) {
			us = 2000;
		}
	}
	
	return us;
}


static void initI2C() {
	DBGLN("initI2C");
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)PIN_SDA;
	DBGLN("PIN_SDA: %d", PIN_SDA);
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = (gpio_num_t)PIN_CLK;
	DBGLN("PIN_CLK: %d", PIN_CLK);
	// conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	// conf.master.clk_speed = 400000;
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	conf.clk_flags = 0;
	// DBGLN("I2C_NUM_0: %d", I2C_NUM_0);
	// DBGLN("conf.master.clk_speed: %d", conf.master.clk_speed);
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
	// vTaskDelete(NULL);
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
	    mpuIntStatus = mpu.getIntStatus();
		
		fifoCount = mpu.getFIFOCount();

	    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
	        // DBGLN("Resetting FIFO");
	        mpu.resetFIFO();

	   
	    } else if (mpuIntStatus & 0x02) {
	        // wait for correct available data length
	        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

	        // read a packet from FIFO

	        mpu.getFIFOBytes(fifoBuffer, packetSize);
			mpu.dmpGetGyro(&gyro, fifoBuffer);
	 		mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
/*			DBGLN("YAW: %f, ", ypr[0] * 180/M_PI);
			DBGLN("PITCH: %f, ", ypr[1] * 180/M_PI);
			DBGLN("ROLL: %f", ypr[2] * 180/M_PI);
			DBGLN("Quaternions");
			DBGLN("w: %f ", q.w);
			DBGLN("x: %f ", q.x);
			DBGLN("y: %f ", q.y);
			DBGLN("z: %f ", q.z); */
			// DBGLN("Gyro x: %f", gyro.x * gscale);
			// DBGLN("Gyro y: %f", gyro.y * gscale);
			// DBGLN("Gyro z: %f", gyro.z * gscale);

			// Get yaw/pitch/roll in decidegrees and convert to uint16_t
			uint16_t ypr16[3] = {0};
			ypr16[0] = (uint16_t)(ypr[0] * 1800 / M_PI);
			ypr16[1] = (uint16_t)(ypr[1] * 1800 / M_PI);
			ypr16[2] = (uint16_t)(ypr[2] * 1800 / M_PI);

			
			CRSF_MK_FRAME_T(crsf_sensor_attitude_t) crsfAttitude = {0};
			// DBGLN("Pitch: %d", ypr16[1]);
			crsfAttitude.p.pitch = htobe16(decidegrees2Radians10000(ypr16[1]));
			// DBGLN("crsfAttitude.p.pitch: %d", crsfAttitude.p.pitch);
			crsfAttitude.p.roll = htobe16(decidegrees2Radians10000(ypr16[2]));
			// DBGLN("crsfAttitude.p.roll: %d", crsfAttitude.p.roll);
			crsfAttitude.p.yaw = htobe16(decidegrees2Radians10000(ypr16[0]));
			// DBGLN("crsfAttitude.p.yaw: %d", crsfAttitude.p.yaw);
			
			CRSF::SetHeaderAndCrc((uint8_t *)&crsfAttitude, CRSF_FRAMETYPE_ATTITUDE, CRSF_FRAME_SIZE(sizeof(crsf_sensor_attitude_t)), CRSF_ADDRESS_CRSF_TRANSMITTER);
			telemetry.AppendTelemetryPackage((uint8_t *)&crsfAttitude);
	    }

	// }

}


static int gyroUpdate(unsigned long now)
{
    // static uint32_t lastUpdate;
			
	read_mpu();

    // return DURATION_IMMEDIATELY;
	return DURATION_NEVER;
}

static void initialize()
{
	initI2C();
}

static int start()
{

	initialize_mpu();

    return DURATION_NEVER;
}

static int event()
{
/*     if (servoMgr == nullptr || connectionState == disconnected)
    {
        // Disconnected should come after failsafe on the RX
        // so it is safe to shut down when disconnected
        return DURATION_NEVER;
    }
    else if (connectionState == wifiUpdate)
    {
        servoMgr->stopAllPwm();
        return DURATION_NEVER;
    } */
    return DURATION_IMMEDIATELY;
}

static int timeout()
{
    return gyroUpdate(millis());
}

device_t Gyro_device = {
    .initialize = initialize,
    .start = start,
    .event = event,
    .timeout = timeout,
};

#endif