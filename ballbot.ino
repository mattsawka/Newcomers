#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "Wire.h"
 
#include <Adafruit_MotorShield.h>
#include <Wire.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
 
// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container

// Interrupt detection routine:
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// Arduino pin attatchments
const int mpuInt = 2; 

// Pitch PID variables
const int Pkp = 15; // Proportional: angle from balance point
const int Pki = 2;  // Integral: past trends of values
const int Pkd = 20; // Derivative: angular velocity of robot
int Perror = 0;     // Deviation from desired upright point based on kp, ki, and kd

// Roll PID variables
const int Rkp = 15; // Proportional: angle from balance point
const int Rki = 2;  // Integral: past trends of values
const int Rkd = 20; // Derivative: angular velocity of robot
int Rerror = 0;     // Deviation from desired upright point based on kp, ki, and kd

// Pitch Other variables
float Pangle = 0, PlastAngle = 0; // Current and previous angle values
float PangularVelocity = 0;      // Angular velocity of robot
float PbalancePoint = 0;       // An offset to couteract the misaligned center of gravity
short signed int Pintegral = 0;  // The "integral" of the plotted angles

// Roll Other variables
float Rangle = 0, RlastAngle = 0; // Current and previous angle values
float RangularVelocity = 0;      // Angular velocity of robot
float RbalancePoint = 0;       // An offset to couteract the misaligned center of gravity
short signed int Rintegral = 0;  // The "integral" of the plotted angles

void setup() {
  mpuSetup();
  motorSetup();
}

// ==========================================================================================
//                                       Main Loop
// ==========================================================================================

void loop() {
  mpuLoop();

  // Converting the angles into degrees:
  Pangle = (ypr[2] * 180/M_PI) - PbalancePoint;
  Rangle = (ypr[1] * 180/M_PI) - RbalancePoint;

  // Feeding the angle values into an array and summing them caused
  // errors, so I will use this cheap integral shortcut:
  
  // Pitch
  if(abs(Pintegral) > 5){
    // This limits the magnitude of the integral to +/-5:
    if(Pintegral > 0){
      Pintegral--;
    } else {
      Pintegral++;
    }
  } else if(Pangle < 0){
    // Subtract one from the integral if it is negative:
    Pintegral--;
  } else {
    // Add one to the integral if it is positive:
    Pintegral++;
  }
  
 // Roll
    if(abs(Rintegral) > 5){
    // This limits the magnitude of the integral to +/-5:
    if(Rintegral > 0){
      Rintegral--;
    } else {
      Rintegral++;
    }
  } else if(Rangle < 0){
    // Subtract one from the integral if it is negative:
    Rintegral--;
  } else {
    // Add one to the integral if it is positive:
    Rintegral++;
  }

  // Determining the rate of angle change:
  PangularVelocity = Pangle - PlastAngle;
  RangularVelocity = Rangle - RlastAngle;

  // PID error calculation:
  Perror = (Pkp * Pangle) + (Pki * Pintegral) + (Pkd * PangularVelocity);
  Rerror = (Rkp * Rangle) + (Rki * Rintegral) + (Rkd * RangularVelocity);

  // Save the current angle so it can be used in the next error calculation:
  PlastAngle = Pangle;
  RlastAngle = Rangle;

  int pitchSpeed = map(abs(Perror), 0, 320, 0, 255);
  int rollSpeed = map(abs(Rerror), 0, 320, 0, 255);
  
  if(pitchSpeed > 255){pitchSpeed = 255;}
  if(pitchSpeed < 40){pitchSpeed = 0;}
  if(rollSpeed > 255){rollSpeed = 255;}
  if(rollSpeed < 40){rollSpeed = 0;}

// ==========================================================================================
//                                 Print Statements
// ==========================================================================================

  Serial.print("Angles:\t");
  Serial.print(Pangle);
  Serial.print("\t");
  Serial.print(Rangle);
  Serial.print("\t");
  Serial.print("Errors:\t");
  Serial.print(Perror);
  Serial.print("\t");
  Serial.println(Rerror);
  Serial.print("Speeds:\t");
  Serial.print(pitchSpeed);
  Serial.print("\t");
  Serial.print(rollSpeed);
  Serial.print("\t");

// ==========================================================================================
//                    Setting the speed and direction of the motors
// ==========================================================================================
  

  if(abs(Perror) < 10 || abs(Pangle) > 40){
    // Turn off the motors if tilting too far, or if well-balanced:
    motor1->run(RELEASE);
    motor2->run(RELEASE);
    motor3->run(RELEASE);
  } else if(Perror < 0){
    motor1->run(RELEASE);
    motor2->run(BACKWARD);
    motor3->run(BACKWARD);

    // Setting the motor speeds. PWM outputs go from 0 to 255 so we map
    motor2->setSpeed(pitchSpeed);
    motor3->setSpeed(pitchSpeed);
  } else {
    motor1->run(RELEASE);
    motor2->run(FORWARD);
    motor3->run(FORWARD);

    motor2->setSpeed(pitchSpeed);
    motor3->setSpeed(pitchSpeed);
  }

    // Setting the speed and direction of the motors for Roll:
  if(abs(Rerror) < 10 || abs(Rangle) > 40){
    // Turn off the motors if tilting too far, or if well-balanced:
    motor1->run(RELEASE);
    motor2->run(RELEASE);
    motor3->run(RELEASE);
  } else if(Rerror < 0){
    motor1->run(BACKWARD);
    motor2->run(BACKWARD);
    motor3->run(BACKWARD);
    
    motor1->setSpeed(rollSpeed);
    motor2->setSpeed(rollSpeed/2);
    motor3->setSpeed(rollSpeed/2);
  } else {
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motor3->run(FORWARD);

    motor1->setSpeed(rollSpeed);
    motor2->setSpeed(rollSpeed/2);
    motor3->setSpeed(rollSpeed/2);
  }
}


// ==========================================================================================
//                               Setup Functions and other functions
// ==========================================================================================

void motorSetup(){ 
  // Motor Shield: create with the default frequency 1.6KHz
  AFMS.begin();
 
  // Set the speed to start, from 0 (off) to 255 (max speed)
  motor1->setSpeed(255);
  motor2->setSpeed(255);
  motor3->setSpeed(255);
}

void mpuSetup(){
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize serial communication
  Serial.begin(115200);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(103);
  mpu.setYGyroOffset(36);
  mpu.setZGyroOffset(-20);
  mpu.setZAccelOffset(1115);

  // turn on the DMP, now that it's ready
  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);

  // enable Arduino interrupt detection
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
  attachInterrupt(0, dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();

  // set our DMP Ready flag so the main loop() function knows it's okay to use it
  Serial.println(F("DMP ready! Waiting for first interrupt..."));
  dmpReady = true;

  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
}

void mpuLoop(){
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {}

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
        
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  } 
}
