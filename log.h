#include "vector.h"

#define LOG_RATE 100                                      // частота логирования в герцах
#define LOG_DURATION 10                                   // длительность записи в секундах
#define LOG_PERIOD 1.0 / LOG_RATE                         // период лога
#define LOG_SIZE LOG_DURATION * LOG_RATE                  // размер лога 

float tFloat;
Vector attitudeEuler;                                     // хранить будем положение действительных и желаемых углов эйлера 
Vector attitudeTargetEuler;

struct LogEntry {                                 // структура для хранения данных
	const char *name;
	float *value;
};

LogEntry logEntries[] = {                                                    // массив структур
	{"t", &tFloat},                                       // время
	{"rates.x", &rates.x},                                // скорости реальные
	{"rates.y", &rates.y},
	{"rates.z", &rates.z},
	{"ratesTarget.x", &ratesTarget.x},                    // скорости желаемые
	{"ratesTarget.y", &ratesTarget.y},
	{"ratesTarget.z", &ratesTarget.z},
	{"attitude.x", &attitudeEuler.x},                     // положение по Эйлеру
	{"attitude.y", &attitudeEuler.y},
	{"attitude.z", &attitudeEuler.z},
	{"attitudeTarget.x", &attitudeTargetEuler.x},         // желаемое положение по Эйлеру
	{"attitudeTarget.y", &attitudeTargetEuler.y},
	{"attitudeTarget.z", &attitudeTargetEuler.z},
	{"thrustTarget", &thrustTarget}
};

const int logColumns = sizeof(logEntries) / sizeof(logEntries[0]);
float logBuffer[LOG_SIZE][logColumns];                                        // массив символов для хранения 

void prepareLogData() {                                                       // готовит данные для логирования преобазуя ориентацию в углы Эйлера  
	tFloat = t;
	attitudeEuler = attitude.toEulerZYX();
	attitudeTargetEuler = attitudeTarget.toEulerZYX();
}

void logData() {                                                              // данные логируются если система включена и период записи не истек
	if (!armed) return;
	static int logPointer = 0;
	static double logTime = 0;
	if (t - logTime < LOG_PERIOD) return;
	logTime = t;

	prepareLogData();

	for (int i = 0; i < logColumns; i++) {
		logBuffer[logPointer][i] = *logEntries[i].value;
	}

	logPointer++;
	if (logPointer >= LOG_SIZE) {
		logPointer = 0;
	}
}

void dumpLog() {                                                              // выводит лог

	for (int i = 0; i < logColumns; i++) {
		print("%s%s", logEntries[i].name, i < logColumns - 1 ? "," : "\n");
	}

	for (int i = 0; i < LOG_SIZE; i++) {
		if (logBuffer[i][0] == 0) continue; // skip empty records
		for (int j = 0; j < logColumns; j++) {
			print("%g%s", logBuffer[i][j], j < logColumns - 1 ? "," : "\n");
		}
	}
}
