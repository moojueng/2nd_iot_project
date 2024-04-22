#include "I2Cdev.h"                                   //I2C communication
#include "MPU6050_6Axis_MotionApps20.h"               //Gyroscope
#include <PID_v1.h>
#include "MyMotor.h"

#define DEADZONE 12

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

// MPU control/status
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion
Quaternion q;        // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3];        // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

void dmpDataReady()
{
	mpuInterrupt = true;
}

//PID parameters
double originalSetpoint = 185.35;
double setpoint = originalSetpoint;
double fowardpoint=originalSetpoint+1;
double backwardpoint=originalSetpoint-1;

double movingAngleOffset = 0.3;
double input, output;

//
double Kp = 6;
double Kd = 0.6; //0.4;
double Ki = 110;

double Kp_M=8;
double Kd_M = 0.6; ;
double Ki_M = 0.8;

PID pid( & input, & output, & setpoint, Kp, Ki, Kd, DIRECT);
PID pid_foward( & input, & output, & fowardpoint, Kp_M, Ki_M, Kd_M, DIRECT);
PID pid_backward( & input, & output, & backwardpoint, Kp_M, Ki_M, Kd_M, DIRECT);

void setInputValue(double inpVal, short inpMode) {
	switch (inpMode) {
		case 0:
		originalSetpoint = inpVal;
		break;
		case 1:
		Kp = inpVal;
		break;
		case 2:
		Kd = inpVal;
		break;
		case 3:
		Ki = inpVal;
		break;
	}
}


char joy_BT='s';

void setup()
{
	Serial.begin(9600); // open the serial port at 9600 bps:

	// join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
	#endif

	mpu.initialize();

	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(253);
	mpu.setYGyroOffset(-160);
	mpu.setZGyroOffset(80);
	mpu.setZAccelOffset(506);

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
		
		//setup PID
		pid.SetMode(AUTOMATIC);
		pid.SetSampleTime(10);
		pid.SetOutputLimits(-200, 200);
		
		pid_foward.SetMode(AUTOMATIC);
		pid_foward.SetSampleTime(10);
		pid_foward.SetOutputLimits(-200, 200);
		
		pid_backward.SetMode(AUTOMATIC);
		pid_backward.SetSampleTime(10);
		pid_backward.SetOutputLimits(-200, 200);
	}
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		//Serial.print(F("DMP Initialization failed (code "));
		//Serial.print(devStatus);
		//Serial.println(F(")"));
	}
	pinMode(3,OUTPUT);
	digitalWrite(3,HIGH);
	Motor_Init();

}

void loop()
{
	
	if(Serial.available())
	{
		joy_BT=Serial.read();
	}
	
	
	
	if (!dmpReady) return;
	while (!mpuInterrupt && fifoCount < packetSize)
	{
		if(joy_BT=='s')
		{
			pid.Compute();
			Back_N_Foward(output, DEADZONE);
		}
		else if(joy_BT=='f')
		{
			pid_foward.Compute();
			Back_N_Foward(output, DEADZONE);
		}
		else if(joy_BT=='b')
		{
			pid_backward.Compute();
			Back_N_Foward(output, DEADZONE);
		}
		else
		{
			pid.Compute();
			Back_N_Foward(output, DEADZONE);
		}

	}
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		//Serial.println(F("FIFO overflow!"));

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
		} else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		mpu.dmpGetQuaternion( &q, fifoBuffer);
		mpu.dmpGetGravity( &gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		input = ypr[1] * 180 / M_PI + 180;
	}
	// put your main code here, to run repeatedly:
	
	

}