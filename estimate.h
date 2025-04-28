#include "quaternion.h"                                                                // Определение положения в пространстве по датчикам
#include "vector.h"
#include "lfp.h"
#include "util.h"

#define WEIGHT_ACC 0.5f                                                                // вес поправки акселерометра
#define RATES_LFP_ALPHA 0.2                                                            // константа сглаживания частота отсечки ~ 40 Hz

LowPassFilter<Vector> ratesFilter(RATES_LFP_ALPHA);                                    // инициализация фильтра нижних частот по скорости
extern Vector rates, gyro, acc;
extern Quaternion attitude;

void applyGyro() {                                                                     
	rates = ratesFilter.update(gyro);                                                         // фильтр нижних частот применяем к гироскопу чтобы устанить высокочастотный шум


	// apply rates to attitude
	attitude = attitude.rotate(Quaternion::fromAngularRates(rates * dt));  
                                                       // апдейт положения коптера 
}

void applyAcc() {                                                                      // Акселерометр используется для коррекции смещения при оценке ориентации. Это делается путем сравнения измеренного вектора ускорения (acc) с ожидаемым вектором силы тяжести (up).
	float accNorm = acc.norm();                                                                    //считаем норму показаний акселерометра
	bool landed = !motorsActive() && abs(accNorm - ONE_G) < ONE_G * 0.1f;						   // флаг поосадки (моторы не активны и результирующий вектор ускорения меньше 10% от g)

	if (!landed){return;}


	Vector up = attitude.rotateVector(Vector(0, 0, 1));                                            // вектор силы тяжести
	Vector correction = Vector::angularRatesBetweenVectors(acc, up) * dt * WEIGHT_ACC;             // вектор коррекции положения 
  

	attitude = attitude.rotate(Quaternion::fromAngularRates(correction));                          // применение коррекции
}

void estimate() {
	applyGyro();
	applyAcc();
}
