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

float MotorControl::detecTarget(int distanceCM) {

    return 0;
}

void MotorControl::open(int speed) {
    ledcWrite(PWM_CHANNEL1, speed);
    ledcWrite(PWM_CHANNEL2, 0);
}

void MotorControl::close(int speed) {
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



