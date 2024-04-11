#include <Adafruit_MPU6050.h>

#include <Servo.h>
#include "I2Cdev.h"
#include <PID_v1.h>


#include "MPU6050_6Axis_MotionApps20.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

Adafruit_MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
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

bool flag = false;
double launchTime = 0.0;
double deployTime = 15000;
int resetCount = 0;
const int LOOPFORRESET = 10;
const int outPort = 13;
bool end = false;

float yaw = 0, pitch = 0, roll = 0;
const float T_YAW = 0, T_PITCH = 0, T_ROLL = 0;
Servo yawServ, pitchServ;
int yawServVal = 0, pitchServVal = 0;
int yawServPin = 0, pitchServPin = 0;
const float kP = 1.0, kD = 0.0, kI = 0;
double setPointYaw, inputYaw, outputYaw, setPointPitch, inputPitch, outputPitch;
long lastTime = 0;

PID pidYaw(&inputYaw, &outputYaw, &setPointYaw, kP, kI, kD, DIRECT);
PID pidPitch(&inputPitch, &outputPitch, &setPointPitch, kP, kP, kD, DIRECT);

const int PITCH_PIN = 0, YAW_PIN = 0;
const int PITCH_SERV_START = 90, YAW_SERV_START = 90;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
long last_time = 0;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  pinMode(outPort, OUTPUT);
  digitalWrite(outPort, LOW);
  yawServ.attach(4);
  pitchServ.attach(5);
      Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }
  
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

  // Initialize the PID controller
  pidYaw.SetMode(AUTOMATIC);
  pidYaw.SetOutputLimits(-90, 90);
  pidPitch.SetMode(AUTOMATIC);
  pidPitch.SetOutputLimits(-90, 90);

  // Set the yaw target and pitch target
  setPointYaw = T_YAW;
  setPointPitch = T_PITCH;
  Serial.println("Adafruit MPU6050 test!");

}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  long start_time = millis();
  if (!end)
  {
    // if programming failed, don't try to do anything
  sensors_event_t a, g, temp;
  
  mpu.getEvent(&a, &g, &temp);
  resetCount++;
  

  if (a.acceleration.z > 8.0 && !flag) {
      // mpu.setGyroStandby(true, true, true);
    if (resetCount % LOOPFORRESET == 0)
    Serial.println((String) a.acceleration.z);
  }
  else {
    // mpu.setGyroStandby(false, false, false);
    if (!flag) {
      yaw = 0.0;
      pitch = 0.0;
      launchTime =  millis() / 1000.0;
    lastTime = micros();
    }
    flag = true;
  }
  
  if (flag) {
    double dyaw = g.gyro.x;
    double dpitch = g.gyro.y;
    double deadBand = 0.00;
    // if(dyaw <= deadBand && yaw >= -deadBand)
    //   dyaw = 0;
    // if(dpitch <= deadBand && dpitch >= -deadBand)
    //   dpitch = 0;
    dyaw = dyaw * (double)(micros() - lastTime) / 1000000.0;
    dpitch = dpitch * (double)(micros() - lastTime) / 1000000.0;
    yaw += dyaw;
    pitch += dpitch;

    if (resetCount % LOOPFORRESET == 0)
    Serial.println((String) yaw);
    lastTime = micros();

    // Compute PID Loop 
    yawServVal = analogRead(yawServPin);
    pitchServVal = analogRead(pitchServPin);
    inputYaw = yaw;
    inputPitch = pitch;
    outputYaw = inputYaw * kP;
    outputPitch = inputPitch * kP;
    //Serial.println((String)outputYaw);
    // Serial.println((String)outputPitch);
    
    // Command servo to position
    yawServ.write(YAW_SERV_START + (outputYaw * 180.0/M_PI));
    pitchServ.write(PITCH_SERV_START + (outputPitch * 180.0/M_PI));
  }
  }
  if (deployTime < millis() - launchTime && flag) {
    digitalWrite(outPort, HIGH);
    flag = false;
    while(true);
    end = true;
  }
  while(millis() < start_time + 1.0);
}
