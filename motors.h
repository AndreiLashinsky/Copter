#include "util.h"

#define MOTOR_0_PIN 18 // rear left
#define MOTOR_1_PIN 5 // rear right
#define MOTOR_2_PIN 17 // front right
#define MOTOR_3_PIN 16 // front left

#define PWM_FREQUENCY 400
#define PWM_RESOLUTION 12
#define PWM_STOP 1000
#define PWM_MIN 1100
#define PWM_MAX 1800

extern float motors[4];
const int MOTOR_REAR_LEFT = 0;
const int MOTOR_REAR_RIGHT = 1;
const int MOTOR_FRONT_RIGHT = 2;
const int MOTOR_FRONT_LEFT = 3;


int getDutyCycle(float value) {
    value = constrain(value, 0, 1);
    float pwm = mapff(value, 0, 1, PWM_MIN, PWM_MAX);
    if (value == 0) {pwm = PWM_STOP;}
    float duty = mapff(pwm, 0, 1000000 / PWM_FREQUENCY, 0, (1 << PWM_RESOLUTION) - 1);
    return round(duty);
}

void sendMotors() {
    ledcWrite(MOTOR_0_PIN, getDutyCycle(motors[0]));
    ledcWrite(MOTOR_1_PIN, getDutyCycle(motors[1]));
    ledcWrite(MOTOR_2_PIN, getDutyCycle(motors[2]));
    ledcWrite(MOTOR_3_PIN, getDutyCycle(motors[3]));
}


void setupMotors() {
    Serial.println("Setup Motors\n");

    ledcAttach(MOTOR_0_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(MOTOR_1_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(MOTOR_2_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(MOTOR_3_PIN, PWM_FREQUENCY, PWM_RESOLUTION);

    sendMotors();
    Serial.println("Motors initialized\n");
}


bool motorsActive() {
    return motors[0] != 0 || motors[1] != 0 || motors[2] != 0 || motors[3] != 0;
}

void testMotor(uint8_t n) {
    Serial.printf("Testing motor %d\n", n);
    motors[n] = 1;
    delay(500); 
    sendMotors();
    delay(3000);
    motors[n] = 0;
    sendMotors();
    Serial.println("Done\n");
}
