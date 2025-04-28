#ifndef IMU_H
#define IMU_H

#include "vector.h"
#include "util.h"
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>


#ifndef _ESP32_HAL_I2C_H_0
#define SDA_PIN 21
#define SCL_PIN 22


float filteredDirection = 0;
const float alpha = 0.1;

extern Vector acc, gyro, compas;
float magMinX, magMaxX, magMinY, magMaxY, magMinZ, magMaxZ;
float mX, mY, mZ;

Adafruit_BMP280 bme;
MPU9250_asukiaaa imu;


struct SensorData {
  float pos1;
  float pos2;
  float pos3;
}


bme_data = SensorData();

//Vector magVector =  Vector(0, 0, 0);
//Vector accelVector = Vector(0, 0, 0);
//Vector gyroVector = Vector(0, 0, 0);


Vector accBias;
Vector gyroBias;
Vector accScale(1, 1, 1);


void printIMUCal() {
	Serial.printf("gyro bias: %f %f %f\n", gyroBias.x, gyroBias.y, gyroBias.z);
	Serial.printf("accel bias: %f %f %f\n", accBias.x, accBias.y, accBias.z);
	Serial.printf("accel scale: %f %f %f\n", accScale.x, accScale.y, accScale.z);
}


void setupIMU_BME() {
  Serial.println("Setup imu\n");

	#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  imu.setWire(&Wire);
  #else
  Wire.begin(SDA_PIN, SCL_PIN);
  imu.setWire(&Wire);
  #endif

  imu.beginAccel();
  imu.beginGyro();
  imu.beginMag();

  Serial.println("Setup bme\n");
	bme.begin(0x76);
  if (!bme.begin(0x76)) {Serial.println("sensor failed");}
}


void configureIMU() {
  imu.beginAccel(ACC_FULL_SCALE_4_G);
  imu.beginGyro(GYRO_FULL_SCALE_2000_DPS);
  imu.beginMag(MAG_MODE_CONTINUOUS_8HZ);
}


void calibrateAccelOnce() {
  const int samples = 1000; 
  static Vector accMax(-INFINITY, -INFINITY, -INFINITY);
  static Vector accMin(INFINITY, INFINITY, INFINITY);

  acc = Vector(0, 0, 0);
  for (int i = 0; i < samples; i++) {
    if (imu.accelUpdate() == 0) {
      Vector sample;
      sample = Vector(imu.accelX(), imu.accelY(), imu.accelZ());
      acc = acc + sample;
    }
  }
  acc = acc / samples;

  if (acc.x > accMax.x) accMax.x = acc.x;                                
	if (acc.y > accMax.y) accMax.y = acc.y;
	if (acc.z > accMax.z) accMax.z = acc.z;
	if (acc.x < accMin.x) accMin.x = acc.x;
	if (acc.y < accMin.y) accMin.y = acc.y;
	if (acc.z < accMin.z) accMin.z = acc.z;

  accScale = (accMax - accMin) / 2 / ONE_G;
	accBias = (accMax + accMin) / 2;
}


void calibrateAccel() {
  Serial.println("Start calibrate accel? (y/n)");
  static String str = readSerial();
  if (str == "y" || str == "Y") {
    Serial.println("Calibrating accelerometer");
    imu.beginAccel(ACC_FULL_SCALE_2_G);

    Serial.setTimeout(5000);                                                                    // Пошагово запрашивает пользователя разместить устройство в разных ориентациях.
    Serial.print("Place level [enter | 5 sec] \n"); Serial.readStringUntil('\n');                      // прямо вверх
    calibrateAccelOnce();
    Serial.print("Place nose up [enter | 5 sec] \n"); Serial.readStringUntil('\n');                    // носом вверх
    calibrateAccelOnce();
    Serial.print("Place nose down [enter | 5 sec] \n"); Serial.readStringUntil('\n');                  // носом вниз
    calibrateAccelOnce();
    Serial.print("Place on right side [enter | 5 sec] \n"); Serial.readStringUntil('\n');              // на правом боку
    calibrateAccelOnce();
    Serial.print("Place on left side [enter | 5 sec] \n"); Serial.readStringUntil('\n');               // на левом боку
    calibrateAccelOnce();
    Serial.print("Place upside down [enter | 5 sec] \n"); Serial.readStringUntil('\n');                // на спине
    calibrateAccelOnce();

    printIMUCal();
    configureIMU();
  } else if (str == "n" || str == "N") {
    Serial.println("Accel calibration aborted");
  } else {
    Serial.println("Wrong command. Try again.");
    calibrateAccel();
  }
}


void calibrateGyro() {
  const int samples = 1000;
  Serial.println("Calibrating gyro, stand still.");
  imu.beginGyro(GYRO_FULL_SCALE_250_DPS);
  gyroBias = Vector(0, 0, 0);
  for (int i = 0; i < samples; i++) {
    if (imu.gyroUpdate()==0) {
      gyro = Vector(imu.gyroX(), imu.gyroY(), imu.gyroZ());
      gyroBias = gyroBias + gyro;
      //Serial.println(i);
    }
  }

  gyroBias = gyroBias / samples;

  printIMUCal();
  delay(1000);
}


void rotateIMU(Vector& data) {
  data = Vector(data.x, data.y, -data.z);
}


void readIMU() {
  if (imu.accelUpdate() == 0) {
    acc = Vector(imu.accelX(), imu.accelY(), imu.accelZ());
    acc = (acc - accBias) / accScale;
  }
  if (imu.gyroUpdate()==0) {
    gyro = Vector(imu.gyroX(), imu.gyroY(), imu.gyroZ());
    gyro = (gyro - gyroBias);
  }

  rotateIMU(acc);
  rotateIMU(gyro);
}


void calibrateMag() {
  Serial.println("Start calibrate mag? (y/n)");
  static String str = readSerial();
  if (str == "y" || str == "Y") {
    magMaxX = magMinX = imu.magX();
    magMaxY = magMinY = imu.magY();
    magMaxZ = magMinZ = imu.magZ();
    Serial.println("Rotate sensor in all directions for 30 seconds...");

    unsigned long startTime = millis();
    
    while (millis() - startTime < 30000) {
      if (imu.magUpdate() == 0) {
        float x = imu.magX();
        float y = imu.magY();
        float z = imu.magZ();

        magMinX = min(magMinX, x);
        magMaxX = max(magMaxX, x);
        magMinY = min(magMinY, y);
        magMaxY = max(magMaxY, y);
        magMinZ = min(magMinZ, z);
        magMaxZ = max(magMaxZ, z);
      }
      delay(100);
    }

    // Рассчет оффсетов и масштабов
    imu.magXOffset = (magMaxX + magMinX)/2;
    imu.magYOffset = (magMaxY + magMinY)/2;
    imu.magZOffset = (magMaxZ + magMinZ)/2;
  } else if (str == "n" || str == "N") {
    Serial.println("Mag calibration aborted");
  } else {
    Serial.println("Wrong command. Try again.");
    calibrateMag();
    }
}


void readMag() {
  if (imu.magUpdate() == 0) {
  // Применяем калибровочные оффсеты
    mX = imu.magX() - imu.magXOffset;
    mY = imu.magY() - imu.magYOffset;
    mZ = imu.magZ() - imu.magZOffset;

    if (imu.accelUpdate() == 0) {
      float roll = atan2(acc.y, acc.z);
      float pitch = atan2(-acc.x, sqrt(acc.y*acc.y + acc.z*acc.z));

      float cosRoll = cos(roll);
      float sinRoll = sin(roll);
      float cosPitch = cos(pitch);
      float sinPitch = sin(pitch);

      float compensatedX = mX * cosPitch + mY * sinRoll * sinPitch + mZ * cosRoll * sinPitch;
      float compensatedY = mY * cosRoll - mZ * sinRoll;

      float mDirection = atan2(compensatedY, compensatedX) * 180/M_PI;
      if (mDirection < 0) {mDirection += 360;}

      float filteredDirection = (1 - alpha) * filteredDirection + alpha * mDirection;

      Serial.println(filteredDirection);
    }
  }
}

void readBME() {
  if (bme.begin(0x76)) {
    bme_data.pos1 = bme.readTemperature();
    bme_data.pos2 = bme.readPressure() / 3377;
    bme_data.pos3 = bme.readAltitude(1013.25);
    Serial.print(bme_data.pos1); Serial.print(" ");Serial.print(bme_data.pos2); Serial.print(" ");Serial.println(bme_data.pos3);
  }
}
#endif
#endif
