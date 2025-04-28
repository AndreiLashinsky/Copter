#ifndef PID_H
#define PID_H

#include "lfp.h"

class PID {
public:
	float p = 0;
	float i = 0;
	float d = 0;
	float windup = 0;

	float derivative = 0;
	float integral = 0;

	LowPassFilter<float> lpf;                                                   // фильтр нижних частот

	PID(float p, float i, float d, float windup = 0, float dAlpha = 1) : p(p), i(i), d(d), windup(windup), lpf(dAlpha) {};

	float update(float error, float dt) {
		integral += error * dt;

		if (isfinite(prevError) && dt > 0) {
			derivative = (error - prevError) / dt;                                  // рачет производнорй


			derivative = lpf.update(derivative);                                    // применяем фильтр к производной
		}

		prevError = error;

		return p * error + constrain(i * integral, -windup, windup) + d * derivative; // PID
	}

	void reset() {
		prevError = NAN;
		integral = 0;
		derivative = 0;
	}

private:
	float prevError = NAN;
};
#endif
