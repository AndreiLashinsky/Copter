#ifndef UTIL_H
#define UTIL_H

#include <math.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>

String input;
const float ONE_G = 9.80665;

float mapf(long x, long in_min, long in_max, float out_min, float out_max) {
	return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

float mapff(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float wrapAngle(float angle) {                                        // угол в пределы от -пи до пи
	angle = fmodf(angle, 2 * PI);
	if (angle > PI) {
		angle -= 2 * PI;
	} else if (angle < -PI) {
		angle += 2 * PI;
	}
	return angle;
}


void disableBrownOut() {                                             // отключение сброса по напряжению
	REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
}


void splitString(String& str, String& token0, String& token1, String& token2) {  
	str.trim();
	char chars[str.length() + 1];
	str.toCharArray(chars, str.length() + 1);
	token0 = strtok(chars, " ");
	token1 = strtok(NULL, " "); 
	token2 = strtok(NULL, "");
}


String readSerial() {
  while (Serial.available() == 0) {                                   // Ждём, пока появятся данные
    delay(10);                                                        // небольшая задержка для избежания перегрузки процессора
  }
  input.clear();

  input = Serial.readStringUntil('\n');                               // Когда данные появились — читаем строку
  input.trim();                                                       // Удаляем лишние пробелы, \r и т.д.

  if (input.length() == 0) {
    Serial.println("Ошибка: введена пустая строка. Повторите ввод.");
    return "Данных нет.";
  }

  return input;
}
  


#endif
