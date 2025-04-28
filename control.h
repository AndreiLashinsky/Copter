// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
#ifndef CONTROL_H
#define CONTROL_H

#include "mode.h"
#include "vector.h"                                 // подтягиваем библиотеки математики, ПИД контроллера, фильтра нижних частот и утилиты // 
#include "quaternion.h"
#include "pid.h"
#include "lfp.h"
#include "util.h"
//#include "photoled.h"
#include "failsafe.h"


#define PITCHRATE_P 0.05                            // определяем константы пид регуляторов для всех осей, положений и скоростей // 
#define PITCHRATE_I 0.2
#define PITCHRATE_D 0.001
#define PITCHRATE_I_LIM 0.3
#define ROLLRATE_P PITCHRATE_P
#define ROLLRATE_I PITCHRATE_I
#define ROLLRATE_D PITCHRATE_D
#define ROLLRATE_I_LIM PITCHRATE_I_LIM
#define YAWRATE_P 0.3
#define YAWRATE_I 0.0
#define YAWRATE_D 0.0
#define YAWRATE_I_LIM 0.3
#define ROLL_P 4.5
#define ROLL_I 0
#define ROLL_D 0
#define PITCH_P ROLL_P
#define PITCH_I ROLL_I
#define PITCH_D ROLL_D
#define YAW_P 3
#define PITCHRATE_MAX radians(360)
#define ROLLRATE_MAX radians(360)
#define YAWRATE_MAX radians(300)
#define TILT_MAX radians(30)

#define RATES_D_LPF_ALPHA 0.2                        // cutoff frequency ~ 40 Hz  


extern Mode current_mode;
enum { YAW, YAW_RATE } yawMode = YAW;                                                              // режим контроля по yaw(положение/скорость)
//bool armed = false;                                                                                // флаг режима полета
extern float controls[16];

PID rollRatePID(ROLLRATE_P, ROLLRATE_I, ROLLRATE_D, ROLLRATE_I_LIM, RATES_D_LPF_ALPHA);            // инициализация ПИД контроллера для скорости roll'a
PID pitchRatePID(PITCHRATE_P, PITCHRATE_I, PITCHRATE_D, PITCHRATE_I_LIM, RATES_D_LPF_ALPHA);       // инициализация ПИД контроллера для скорости pitch'a
PID yawRatePID(YAWRATE_P, YAWRATE_I, YAWRATE_D);                                                   // инициализация ПИД контроллера для скорости yaw'a
PID rollPID(ROLL_P, ROLL_I, ROLL_D);                                                               // инициализация ПИД контроллера для положения по roll'у
PID pitchPID(PITCH_P, PITCH_I, PITCH_D);                                                           // инициализация ПИД контроллера для положения по pitch'у
PID yawPID(YAW_P, 0, 0);                                                                           // инициализация ПИД контроллера для положения по yaw'у
Vector maxRate(ROLLRATE_MAX, PITCHRATE_MAX, YAWRATE_MAX);                                          // инициализация вектора максимальной скорости по эйлеровым углам
float tiltMax = TILT_MAX;                                                                          // ограничение на вычисление желаемого положения

Quaternion attitudeTarget;                                                                         // определение кватерниона переменной желаемого положения
Vector ratesTarget;                                                                                // определение вектора желаемых скоростей
Vector torqueTarget;                                                                               // определение вектора желаемого крутящего момента 
float thrustTarget = controls[throttleChannel];                                                                             // не имею понятия нахрена это надо, потом знаю !!! 

extern const int MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT;           // подтягиваем массив моторов 
extern int rollChannel, pitchChannel, throttleChannel, yawChannel, armedChannel, modeChannel;      // подтягиваем каналы пульта 


void controlRate() {                                                                               // высчитывает желаемые моменты для достижения желаемых угловых скоростей 
	if (!armed) {
		rollRatePID.reset();                                                                       // использует ПИД регуляторы по скорости roll'а pitch'а и yaw'а для вычисления угловой скорости
		pitchRatePID.reset();
		yawRatePID.reset();
		return;
	}

	Vector error = ratesTarget - rates;

	// Calculate desired torque, where 0 - no torque, 1 - maximum possible torque
	torqueTarget.x = rollRatePID.update(error.x, dt);
	torqueTarget.y = pitchRatePID.update(error.y, dt);
	torqueTarget.z = yawRatePID.update(error.z, dt);
}




void controlTorque() {                                                                             // распределяет крутящий момент и тягу по моторам
	if (!armed) {
		memset(motors, 0, sizeof(motors));
		return;
	}

	motors[MOTOR_FRONT_LEFT] = thrustTarget + torqueTarget.x - torqueTarget.y + torqueTarget.z;    // миксер моторов
	motors[MOTOR_FRONT_RIGHT] = thrustTarget - torqueTarget.x - torqueTarget.y - torqueTarget.z;
	motors[MOTOR_REAR_LEFT] = thrustTarget + torqueTarget.x + torqueTarget.y - torqueTarget.z;
	motors[MOTOR_REAR_RIGHT] = thrustTarget - torqueTarget.x + torqueTarget.y + torqueTarget.z;

	motors[0] = constrain(motors[0], 0, 1);
	motors[1] = constrain(motors[1], 0, 1);
	motors[2] = constrain(motors[2], 0, 1);
	motors[3] = constrain(motors[3], 0, 1);
}


void interpretRC() {                                                                               // интерпритатор команд пульта. определяет запущенны ли моторы, определяет режим работы, -\- желаемые скорости и углы 
	armed = controls[throttleChannel] >= 0.05 &&
		(controls[armedChannel] >= 0.5 || isnan(controls[armedChannel])); // assume armed if armed channel is not defined

	// NOTE: put ACRO or MANUAL modes there if you want to use them
	if (controls[modeChannel] < 0) {
		current_mode = STAB;
	//} else if (controls[modeChannel] < 0.5) {
	//	current_mode = PHOTO_MAX_HOLD;
	} else if (controls[modeChannel] < 0.75){
		current_mode = ACRO;
	} else {
		//mode = MANUAL;
    current_mode = MANUAL;
	}

	//thrustTarget = controls[throttleChannel];

	if (current_mode == ACRO) {
		yawMode = YAW_RATE;
		ratesTarget.x = controls[rollChannel] * maxRate.x;
		ratesTarget.y = controls[pitchChannel] * maxRate.y;
		ratesTarget.z = -controls[yawChannel] * maxRate.z; // positive yaw stick means clockwise rotation in FLU

	} else if (current_mode == STAB) {
		yawMode = controls[yawChannel] == 0 ? YAW : YAW_RATE;

		attitudeTarget = Quaternion::fromEulerZYX(Vector(
			controls[rollChannel] * tiltMax,
			controls[pitchChannel] * tiltMax,
			attitudeTarget.getYaw()));
		ratesTarget.z = -controls[yawChannel] * maxRate.z; // positive yaw stick means clockwise rotation in FLU

	}
	/*else if (current_mode == PHOTO_MAX_HOLD) {
		yawMode = YAW_RATE;
		attitudeTarget = Quaternion::fromEulerZYX(Vector(0, 0, attitudeTarget.getYaw()));
		ratesTarget.z = -controls[yawChannel] * maxRate.z;
	}*/
	else if (current_mode == MANUAL) {
		// passthrough mode
		yawMode = YAW_RATE;
		torqueTarget = Vector(controls[rollChannel], controls[pitchChannel], -controls[yawChannel]) * 0.01;
	}

	if (yawMode == YAW_RATE || !motorsActive()) {
		// update yaw target as we don't have control over the yaw
		attitudeTarget.setYaw(attitude.getYaw());
	}
}

/*
const char* getModeName(Mode m) {                                                    // возвращает строку с названием режима работы коптера
	switch (m) {
		case MANUAL: return "MANUAL";
		case ACRO: return "ACRO";
		case STAB: return "STAB";
		//case PHOTO_MAX_HOLD: return "PHOTO_MAX_HOLD";
		default: return "UNKNOWN";
	}
}


/*
void controlPhotoMaxHold() {
	if (!armed) {
		return;
	}

	// Чтение текущего значения с фотодиода
	int currentSignal = photo_diod_read();

	// Логика для корректировки положения коптера
	// Например, можно использовать ПИД-регулятор для корректировки углов наклона
	// в зависимости от изменения сигнала фотодиода

	// Пример простой логики:
	static int lastSignal = 0;
	if (currentSignal > lastSignal) {
		// Если сигнал увеличился, продолжаем двигаться в том же направлении
		// Например, увеличиваем угол наклона по pitch или roll
		attitudeTarget = Quaternion::fromEulerZYX(Vector(
			attitudeTarget.getRoll() + 0.01,
			attitudeTarget.getPitch() + 0.01,
			attitudeTarget.getYaw()));
	}
	else {
		// Если сигнал уменьшился, меняем направление
		attitudeTarget = Quaternion::fromEulerZYX(Vector(
			attitudeTarget.getRoll() - 0.01,
			attitudeTarget.getPitch() - 0.01,
			attitudeTarget.getYaw()));
	}

	lastSignal = currentSignal;
}

*/

void controlAttitude() {                                                                               // высчитывает желаемые угловые скорости для достижения желаемого положения
	if (!armed) {
		rollPID.reset();                                                                               // использует ПИД регуляторы по положению roll'а pitch'а и yaw'а для вычисления угловой скорости
		pitchPID.reset();
		yawPID.reset();
		return;
	}

	const Vector up(0, 0, 1);
	Vector upActual = attitude.rotateVector(up);
	Vector upTarget = attitudeTarget.rotateVector(up);

	Vector error = Vector::angularRatesBetweenVectors(upTarget, upActual);

	ratesTarget.x = rollPID.update(error.x, dt);
	ratesTarget.y = pitchPID.update(error.y, dt);

	if (yawMode == YAW) {
		float yawError = wrapAngle(attitudeTarget.getYaw() - attitude.getYaw());
		ratesTarget.z = yawPID.update(yawError, dt);
	}
}


void control() {                                                                                   // управляет потоком данных в зависимости от режима работы 
	interpretRC();
	failsafe();
	if (current_mode == STAB) {                                                                            // режим стабилизации: стабилизация положения, регулированиен скорости, распределение крутящего момента
		controlAttitude();
		controlRate();
		controlTorque();
	} else if (current_mode == ACRO) {                                                                     // режим полуручного контроля: пропускает стабилизации положения, стабилизация по скорости и моментам
		controlRate();
		controlTorque();
	}/* else if (current_mode == PHOTO_MAX_HOLD) {                                                           // режим максимизации интенсивности считывая с датчика освещенности
		controlPhotoMaxHold();
		controlRate();
		controlTorque();
	}*/
  else if (current_mode == MANUAL) {                                                                   // режим ручного контроля : пропускает стабилизацию положения и скоростей, стабилизация моментов включена
		controlTorque();
	}
}


#endif
