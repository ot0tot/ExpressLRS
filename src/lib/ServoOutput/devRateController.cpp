#if defined(USE_GYRO)
#include "devRateController.h"
#include "logging.h"


// MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL


/* // Setup gyro
static void gyroSetup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    DBGLN("Initializing I2C devices...");
    mpu.initialize();
    //pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    DBGLN("Testing device connections...");
    DBGLN(mpu.testConnection() ? ("MPU6050 connection successful") : ("MPU6050 connection failed"));

    // load and configure the DMP
    DBGLN("Initializing DMP...");
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        DBGLN("Enabling DMP...");
        mpu.setDMPEnabled(true);

       // enable Arduino interrupt detection
        DBGLN(F("Enabling interrupt detection (Arduino external interrupt "));
        DBGLN(digitalPinToInterrupt(INTERRUPT_PIN));
        DBGLN(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus(); 

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        DBGLN("DMP ready! Waiting for first interrupt...");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        DBGLN("DMP Initialization failed (code ");
        DBGLN(devStatus);
        DBGLN(")");
    }

    // configure LED for output
    // pinMode(LED_PIN, OUTPUT);
}

static void readGyro() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            DBGLN("quat\t");
            DBGLN(q.w);
            DBGLN("\t");
            DBGLN(q.x);
            DBGLN("\t");
            DBGLN(q.y);
            DBGLN("\t");
            DBGLN(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            DBGLN("euler\t");
            DBGLN(euler[0] * 180/M_PI);
            DBGLN("\t");
            DBGLN(euler[1] * 180/M_PI);
            DBGLN("\t");
            DBGLN(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            DBGLN("ypr\t");
            DBGLN(ypr[0] * 180/M_PI);
            DBGLN("\t");
            DBGLN(ypr[1] * 180/M_PI);
            DBGLN("\t");
            DBGLN(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            DBGLN("areal\t");
            DBGLN(aaReal.x);
            DBGLN("\t");
            DBGLN(aaReal.y);
            DBGLN("\t");
            DBGLN(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            DBGLN("aworld\t");
            DBGLN(aaWorld.x);
            DBGLN("\t");
            DBGLN(aaWorld.y);
            DBGLN("\t");
            DBGLN(aaWorld.z);
        #endif
   
    }
} */



#endif