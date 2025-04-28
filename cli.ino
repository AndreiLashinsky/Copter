#include "pid.h"
#include "vector.h"
#include "util.h"
#include "imu.h"
#include "control.h"
#include "failsafe.h"
#include "log.h"
extern const int MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT;  // ссылки на моторы (массив моторов) //
//extern float loopRate, dt; 
extern double t; 
extern int rollChannel, pitchChannel, throttleChannel, yawChannel, armedChannel, modeChannel, debugChannel; // индексы управления пульта //
extern bool handlerInputcame;
const char* motd =  

"\nWelcome to\n"
"  ___    ___   ___   _____   ____    ___\n"  
" /   \  /   \ |   \    |    |       |   \\n" 
"|       |   | |    |   |    |       |    |\n"
"|       |   | |___/    |    |____   |___/\n"
"|       |   | |        |    |       | \\n"
" \___/  \___/ |        |    |____   |  \\n"
"Commands:\n\n"
"help - show help\n"
"p - show all parameters\n"
"p <name> - show parameter\n"
"p <name> <value> - set parameter\n"
"preset - reset parameters\n"
"time - show time info\n"
"ps - show pitch/roll/yaw\n"
"psq - show attitude quaternion\n"
"imu - show IMU data\n"
"rc - show RC data\n"
"mot - show motor output\n"
"log - dump in-RAM log\n"
"cr - calibrate RC\n"
"cg - calibrate gyro\n"
"ca - calibrate accel\n"
"mfr, mfl, mrr, mrl - test motor (remove props)\n"
"reset - reset drone's state\n"
"reboot - reboot the drone\n";
extern Mode curent_mode;

void print(const char* format, ...) { // обертка для сериалпринта, опционально отправляет информацию по wifi используя mavlinkPrint //
	char buf[1000];
	va_list args;
	va_start(args, format);
	vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);
	Serial.print(buf);
//#if WIFI_ENABLED
	//mavlinkPrint(buf);
//#endif
}

void doCommand(String str, bool echo = false) { // парсит и выполняет команды, введенные пользователем //
	String command, arg0, arg1; // парсит команду : делит запрос на команду и дополнительно вводит два аргумента  //
	splitString(str, command, arg0, arg1);

	if (echo && !command.isEmpty()) {  // команда эхо, проверка на наличие команды // 
		print("> %s\n", str.c_str());
	}

	if (command == "help" || command == "motd") {                        // выполнение команды // 
		print("%s\n", motd);
	} else if (command == "p" && arg0 == "") {                           // выводит параметры //
		printParameters();
	} else if (command == "p" && arg0 != "" && arg1 == "") {             // выводит конкретный параметр arg0=param.name() // 
		print("%s = %g\n", arg0.c_str(), getParameter(arg0.c_str()));
	} else if (command == "p") {                                         // устанавливает насчение параметра с arg0=param.name() равное arg1 //                     
		bool success = setParameter(arg0.c_str(), arg1.toFloat());
		if (success) {    
			print("%s = %g\n", arg0.c_str(), arg1.toFloat());
		} else {
			print("Parameter not found: %s\n", arg0.c_str());
		}
	} else if (command == "preset") {                                    // скидывает все параметры на дефолтное значение // 
		resetParameters();
	} else if (command == "time") {                                      // отображает настоящие время, частоту цикла и временной шаг //
		print("Time: %f\n", t);
		print("Loop rate: %f\n", loopRate);
		print("dt: %f\n", dt);
	} else if (command == "ps") {                                        // отображает положение дрона через углы эйлера (roll, pitch, yaw)// 
		Vector a = attitude.toEulerZYX();
		print("roll: %f pitch: %f yaw: %f\n", degrees(a.x), degrees(a.y), degrees(a.z));
	} else if (command == "psq") {                                                                // отображает положение дрона через кватернион // 
		print("qx: %f qy: %f qz: %f qw: %f\n", attitude.x, attitude.y, attitude.z, attitude.w);
	} else if (command == "imu") {                                                                // отображает показания датчиков (гироскоп, акселерометр) //
		print("gyro: %f %f %f\n", rates.x, rates.y, rates.z);
		print("acc: %f %f %f\n", acc.x, acc.y, acc.z);
		printIMUCal();
		print("rate: %f\n", loopRate);
	} else if (command == "rc") {                                                                 // Отображает необработанные и обработанные значения RC-канала.//
		print("Raw: throttle %d yaw %d pitch %d roll %d armed %d mode %d debug %d\n",
			channels[throttleChannel], channels[yawChannel], channels[pitchChannel],
			channels[rollChannel], channels[armedChannel], channels[modeChannel], channels[debugChannel]);
		print("Control: throttle %g yaw %g pitch %g roll %g armed %g mode %g debug %g\n",
			controls[throttleChannel], controls[yawChannel], controls[pitchChannel],
			controls[rollChannel], controls[armedChannel], controls[modeChannel], controls[debugChannel]);
		print("Mode: %s\n", getModeName(current_mode));
	} else if (command == "mot") {                                                                // отображает величину, что выдает на мотор // 
		print("Motors: front-right %g front-left %g rear-right %g rear-left %g\n",
			motors[MOTOR_FRONT_RIGHT], motors[MOTOR_FRONT_LEFT], motors[MOTOR_REAR_RIGHT], motors[MOTOR_REAR_LEFT]);
	} else if (command == "log") {                                                                // удаляет журнал лога внутриоперативноей памяти //
		dumpLog();
	} else if (command == "cr") {                                                                 // калибровка rc контроллера //
		calibrateRC();
	} else if (command == "cg") {                                                                 // калибровка гироскопа //
		calibrateGyro();
	} else if (command == "ca") {                                                                 // калибровка акселерометра //
		calibrateAccel();
	} else if (command == "mfr") {                                                                // тест мотора (передний правый) //
		testMotor(MOTOR_FRONT_RIGHT);
	} else if (command == "mfl") {                                                                // тест мотора (передний левый) //
		testMotor(MOTOR_FRONT_LEFT);
	} else if (command == "mrr") {                                                                // тест мотора (задний правый) //
		testMotor(MOTOR_REAR_RIGHT);
	} else if (command == "mrl") {                                                                // тест мотора (задний левый) //
		testMotor(MOTOR_REAR_LEFT);
	} else if (command == "reset") {                                                              // переводит режим дрона в нейтральное состояние //
		attitude = Quaternion();
	} else if (command == "reboot") {                                                             // перезагружает дрон //
		ESP.restart();
	} else if (command == "") {
		// do nothing
	} else {
		print("Invalid command.");
	}
}

void handleInput() {                                                                              // хэндлер ввода //
	static bool showMotd = true;
	String input;
  
	if (showMotd) {
		print("%s\n", motd);
		showMotd = false;
	}

	while (Serial.available()) {
		input = readSerial();
		doCommand(input);
		input.clear();
		}
  
}
