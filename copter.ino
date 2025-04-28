#include <vector.h>
#include <quaternion.h>
#include <motors.h>
#include <imu.h>
#include "wifi1.ino"
#include "mavlink1.ino"
#include <rc.h>
#include <estimate.h>
#include <control.h>

#define WIFI_ENABLED 0

double t = NAN; // current step time, s
float dt; // time delta from previous step, s
int16_t channels[16]; // raw rc channels
float controls[16]; // normalized controls in range [-1..1] ([0..1] for throttle)
Vector gyro; // gyroscope data
Vector acc; // accelerometer data, m/s/s
Vector rates; // filtered angular rates, rad/s
Quaternion attitude; // estimated attitude
float motors[4]; // normalized motors thrust in range [-1..1]
bool armed = false; 
#define SDA_PIN 21
#define SCL_PIN 22
extern Vector torqueTarget;

extern Mode current_mode;

//float channelNeutral[16] = {NAN}; // first element NAN means not calibrated
//float channelMax[16];


void setup() {
  Serial.begin(115200);
  setupRC();
  setupParameters();
  setupIMU_BME();
  setupMotors();
  //calibrateMag();
  //calibrateGyro();
  //calibrateAccel();
  //calibrateRC();
  //print("Initializing complete");
  //Serial.println("Testing motor 1");
  //testMotor(0);
  //Serial.println("Testing motor 2");
  //testMotor(1);
  //Serial.println("Testing motor 3");
  //testMotor(2);
  //Serial.println("Testing motor 4");
  //testMotor(3);

}

void loop() {
  readRC();
  readIMU();
  step();
  estimate();
  control();
  sendMotors();
  handleInput();
  printthings2();
  syncParameters();
}
