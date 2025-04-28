float loopRate;                                                                                  // Частота выполнения цикла в герцах Hz

extern double t;
extern float dt;

void computeLoopRate() {                         // вычисляет частоту выполнения цикла
	static double windowStart = 0;
	static uint32_t rate = 0;
	rate++;
	if (t - windowStart >= 1) { // 1 second window
		loopRate = rate;
		windowStart = t;
		rate = 0;
	}
}

void step() {                                                  // шаг времени
	double now = micros() / 1000000.0;          // время в секундах
	dt = now - t;                               // разница между сейчас и предыдущим вызовом т
	t = now;                                    // обновление времени 

	if (!(dt > 0)) {
		dt = 0; // assume dt to be zero on first step and on reset
	}

	computeLoopRate();
}
