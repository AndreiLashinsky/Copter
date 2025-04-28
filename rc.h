#ifndef RC_H
#define RC_H

#include <sbus.h>
#include "util.h"
#include <time1.h>
#include <mode.h>

#define SBUS_RX_PIN 4
#define SBUS_TX_PIN -1
extern Quaternion attitude, attitudeTarget;
extern Vector rates, ratesTarget, torqueTarget;
extern Mode current_mode;
float last_check_time = 0;

HardwareSerial SerialSbus(2); 
bfs::SbusRx sbus_rx(&SerialSbus, SBUS_RX_PIN, SBUS_TX_PIN, true);
bfs::SbusData data;

                                                                        //карта каналов радио:
int rollChannel = 0;
int pitchChannel = 1;
int throttleChannel = 2;
int yawChannel = 3;
int armedChannel = 7;
int modeChannel = 5;
int debugChannel = 6;
int seventh_chan = 4;
int eighth_chan = 8;

double controlsTime; // time of the last controls update
float channelMin[16];
float channelNeutral[16] = {NAN}; // first element NAN means not calibrated
float channelMax[16];
extern float controls[16];
extern int16_t channels[16];
int THINGS_TIMER = 100000;


void setupRC() {
	Serial.print("Setup RC\n");
	SerialSbus.begin(100000, SERIAL_8E2, SBUS_RX_PIN, SBUS_TX_PIN);
  sbus_rx.Begin();
}



void normalizeRC() {
	if (isnan(channelNeutral[0])) return; // skip if not calibrated
	for (uint8_t i = 0; i < 16; i++) {
		controls[i] = mapf(channels[i], channelNeutral[i], channelMax[i], 0, 1);
	}
}

bool readRC() {
	if (sbus_rx.Read()) {
		data = sbus_rx.data();
    for (uint8_t i = 0; i < 16; i++) {
      channels[i] = data.ch[i];
    }
		memcpy(channels, data.ch, sizeof(channels)); // copy channels data
		normalizeRC();
		controlsTime = t;
		return true;
	} 
	return false;
}


void printRCCal() {
	Serial.print("Calibration data (min, neutral, max):\n");
    for (int i = 0; i < 16; i++) {
        Serial.printf("Ch %d: Min=%g, Neutral=%g, Max=%g\n", i, channelMin[i], channelNeutral[i], channelMax[i]);
    }
}


void calibrateRC() {
  Serial.println("start calibrateRC? (y/n)");
  static String str = readSerial();
  if (readSerial() == "y" || readSerial() == "Y") {
    float period = 10;
    float now = micros();
    
    Serial.print("Calibrate RC: move all sticks to minimum positions in 10 seconds\n");
    Serial.print("···     ···\n···     ·o·\n·o·     ···\n");
    delay(10000);
    
    while (!readRC() && micros() - now < period);
    
    for (int i = 0; i < 16; i++) {
        channelMin[i] = channels[i];
    }

    Serial.print("Calibrate RC: move all sticks to maximum positions in 10 seconds\n");
    Serial.print("··o     ··o\n···     ···\n···     ···\n");
    delay(10000);
    
    while (!readRC() && micros() - now < 2 * period);
    
    for (int i = 0; i < 16; i++) {
        channelMax[i] = channels[i];
    }

    Serial.print("Calibrate RC: move all sticks to neutral positions in 10 seconds\n");
    Serial.print("···     ···\n···     ·o·\n·o·     ···\n");
    delay(10000);
    
    while (!readRC() && micros() - now < 3 * period);
    
    for (int i = 0; i < 16; i++) {
        channelNeutral[i] = channels[i];
        if (channelMin[i] > channelMax[i]) {
            float temp = channelMin[i];
            channelMin[i] = channelMax[i];
            channelMax[i] = temp;
        }
    }

    printRCCal();
  } else if (str == "n" || str == "N") {
    Serial.println("Calibration aborted");
  } else {
    Serial.println("Wrong command. Try again.");
    calibrateRC();
    }
}

void printthings() {
  unsigned long current_time = micros();
  if (current_time - last_check_time >= THINGS_TIMER || current_time < last_check_time){
    if (controls[debugChannel] < -0.5) {
      return;
    } else if (controls[debugChannel] < 0.5) {
        Serial.printf("Motors power: front left: %0.3f, front right: %0.3f, rear left: %0.3f, rear right: %0.3f.\n", motors[3],motors[2],motors[0],motors[1]);
        Serial.printf("Gyro :  x: %f, y: %f, z: %f.\n", gyro.x, gyro.y, gyro.z);
        Serial.printf("Rates :  wx: %f, wy: %f, wz: %f.\n", rates.x, rates.y, rates.z);
        Serial.printf("Rates target x: %f, y: %f, z: %f.\n", ratesTarget.x, ratesTarget.y, ratesTarget.z);
        Serial.printf("Accel x: %f, y: %f, z: %f.\n", acc.x, acc.y, acc.z);
        //last_check_time = micros();
    } else {
        Serial.printf("Motors power: front left: %0.3f, front right: %0.3f, rear left: %0.3f, rear right: %0.3f.\n", motors[3],motors[2],motors[0],motors[1]);
        Serial.printf("Attitude quaternion: w: %f, x: %f, y: %f, z: %f.\n",attitude.w, attitude.x, attitude.y, attitude.z);
        Serial.printf("Attitude Target quaternion: w: &f, x: %f, y: %f, z: %f.\n",attitudeTarget.w, attitudeTarget.x, attitudeTarget.y, attitudeTarget.z);
        Serial.printf("Torque target x: %f, y: %f, z: %f.\n", torqueTarget.x, torqueTarget.y, torqueTarget.z);
        Vector a = attitude.toEulerZYX(); 
        Serial.printf("Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\n", a.x, a.y, a.z);
        //last_check_time = micros();
    }
    last_check_time = current_time;
  }
}

void printthings2() {
  unsigned long current_time = micros();
  if (current_time - last_check_time >= THINGS_TIMER || current_time < 0.1 * last_check_time){
      //Serial.printf("Motors power: front left: %0.3f, front right: %0.3f, rear left: %0.3f, rear right: %0.3f.\n", motors[3],motors[2],motors[0],motors[1]);
      Serial.printf("Gyro :  x: %f, y: %f, z: %f.\n", gyro.x, gyro.y, gyro.z);
      //Serial.printf("Rates :  wx: %f, wy: %f, wz: %f.\n", rates.x, rates.y, rates.z);
      //Serial.printf("Rates target x: %f, y: %f, z: %f.\n", ratesTarget.x, ratesTarget.y, ratesTarget.z);
      Serial.printf("Accel x: %f, y: %f, z: %f.\n", acc.x, acc.y, acc.z);
      Serial.printf("Attitude quaternion: w: %f, x: %f, y: %f, z: %f.\n",attitude.w, attitude.x, attitude.y, attitude.z);
      Serial.printf("Attitude Target quaternion: w: &f, x: %f, y: %f, z: %f.\n",attitudeTarget.w, attitudeTarget.x, attitudeTarget.y, attitudeTarget.z);
      Serial.printf("Torque target x: %f, y: %f, z: %f.\n", torqueTarget.x, torqueTarget.y, torqueTarget.z);
      String cur_mode = getModeName(current_mode);
      Serial.printf("Mode: %s\n", cur_mode);
      last_check_time = current_time;
  }
}
#endif
