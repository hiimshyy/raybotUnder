#include "motor_control.h"

MotorControl::MotorControl(uint8_t  pwm1Pin, uint8_t pwm2Pin, uint8_t enPin, uint16_t  pwmFreq, uint8_t  pwmRes) {
    _pwm1Pin = pwm1Pin;
    _pwm2Pin = pwm2Pin;
    _enPin = enPin;
    _pwmFreq = pwmFreq;
    _pwmRes = pwmRes;
    ledcAttachPin(_pwm1Pin, PWM_CHANNEL1);
    ledcAttachPin(_pwm2Pin, PWM_CHANNEL2);
    ledcSetup(PWM_CHANNEL1, _pwmFreq, _pwmRes);
    ledcSetup(PWM_CHANNEL2, _pwmFreq, _pwmRes);
    pinMode(_enPin, OUTPUT);
}

uint8_t MotorControl::detecTarget(uint8_t maxSpeed, uint8_t distance) {
    if (distance > 60) target = maxSpeed;
	else if (distance < 60 && distance > 30) target = (distance*maxSpeed)/60;
	else target = 0;
	return target;
}

void MotorControl::open(uint8_t speed) {
    ledcWrite(PWM_CHANNEL1, speed);
    ledcWrite(PWM_CHANNEL2, 0);
}

void MotorControl::close(uint8_t speed) {
    ledcWrite(PWM_CHANNEL1, 0);
    ledcWrite(PWM_CHANNEL2, speed);
}

void MotorControl::stop() {
    ledcWrite(PWM_CHANNEL1, 0);
    ledcWrite(PWM_CHANNEL2, 0);
}

void MotorControl::disable() {
    digitalWrite(_enPin, LOW);
}

void MotorControl::enable() {
    digitalWrite(_enPin, HIGH);
}



