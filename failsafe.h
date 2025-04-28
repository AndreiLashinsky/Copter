// Copyright (c) 2024 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
#ifndef FAILSAFE_H
#define FAILSAFE_H

#include "mode.h"


#define RC_LOSS_TIMEOUT 0.2                                                                             // время (в секундах), после которого система считает, что связь с пультом управления потеряна
#define DESCEND_TIME 3.0                                                                                // время (в секундах), за которое дрон должен плавно снизиться с полного газа до нуля
#define PHOTODIODE_NO_CHANGE_TIMEOUT 5.0


extern double controlsTime;                                                                             // время последнего получения данных с пульта управления
extern int rollChannel, pitchChannel, throttleChannel, yawChannel, minRawValue, maxRawValue;                                      // аналы управления (крен, тангаж, газ, рыскание)
extern bool armed;            

void descend() {                               // устанавливает режим стабилизации положения скоростей и моментов, обнуляет положения по всем углам Эйлера
	current_mode = STAB;
	controls[rollChannel] = 0;
	controls[pitchChannel] = 0;
	controls[yawChannel] = 0;
	controls[throttleChannel] -= dt / DESCEND_TIME;                       // Плавно уменьшает газ (throttleChannel) на величину dt / DESCEND_TIME (где dt — время между кадрами)
	if (controls[throttleChannel] < 0) controls[throttleChannel] = 0;
}


void rcLossFailsafe() {                               // Если разница между текущим временем (t) и временем последнего получения данных (controlsTime)
	if (t - controlsTime > RC_LOSS_TIMEOUT) {         // превышает RC_LOSS_TIMEOUT, активируется функция descend
		descend();
	}
}

/*
void photodiodeFailsafe() {
  double lastPhotodiodeChangeTime;
	if (sig > minRawValue * 1.2) {
		int last_sig = sig;
		lastPhotodiodeChangeTime = micros();
	}
	else if (micros() - lastPhotodiodeChangeTime > PHOTODIODE_NO_CHANGE_TIMEOUT) {
		descend();
	}
}
*/

void armingFailsafe() {                                                                                                  // предотвращает включение (arming) дрона без нулевого газа
	static double zeroThrottleTime;                                                                                      // время, когда газ был близок к нулю (менее 0.05)
	static double armingTime;                                                                                            // время последнего выключения дрона
	if (!armed) armingTime = t; // stores the last time when the drone was disarmed, therefore contains arming time
	if (controlsTime > 0 && controls[throttleChannel] < 0.05) zeroThrottleTime = controlsTime;
	if (armingTime - zeroThrottleTime > 0.1) armed = false; // prevent arming if there was no zero throttle for 0.1 sec
}


void failsafe() {
	armingFailsafe();                                                                                   // предотвращает включение (arming) дрона без нулевого газа
	rcLossFailsafe();                                                                                   // активирует аварийный режим при потере связи с пультом
	//photodiodeFailsafe();
}


#endif
