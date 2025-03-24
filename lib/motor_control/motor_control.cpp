#include "motor_control.h"

MotorControl::MotorControl(uint8_t pwm1Pin, uint8_t pwm2Pin, uint8_t enPin, uint16_t pwmFreq, uint8_t pwmRes) {
    _pwm1Pin = pwm1Pin;
    _pwm2Pin = pwm2Pin;
    _enPin = enPin;
    _pwmFreq = pwmFreq;
    _pwmRes = pwmRes;

    ledcSetup(PWM_CHANNEL1, _pwmFreq, _pwmRes);
    ledcSetup(PWM_CHANNEL2, _pwmFreq, _pwmRes);

    ledcAttachPin(_pwm1Pin, PWM_CHANNEL1);
    ledcAttachPin(_pwm2Pin, PWM_CHANNEL2);

    ledcWrite(PWM_CHANNEL1, 0);
    ledcWrite(PWM_CHANNEL2, 0);

    pinMode(_enPin, OUTPUT);
    digitalWrite(_enPin, HIGH);
}

uint8_t MotorControl::detecTarget(uint8_t maxSpeed, uint8_t distance) {
    if (distance > 60) target = maxSpeed;
	else if (distance < 60 && distance > 30) target = (distance*maxSpeed)/60;
	else target = 0;
	return target;
}

void MotorControl::open(uint8_t speed) {
    //over 90% speed to open the door
    uint32_t _speed = map(speed, 0, 100, 0, 1023);
    // Serial.printf("Motor control - Open speed: %d\n", _speed);
    ledcWrite(PWM_CHANNEL1, 0);
    ledcWrite(PWM_CHANNEL2, _speed);
}

void MotorControl::close(uint8_t speed) {
    //over 70% speed to close the door
    uint32_t _speed = map(speed, 0, 100, 0, 1023);
    // Serial.printf("Motor control - Close speed: %d\n", _speed);
    ledcWrite(PWM_CHANNEL1, _speed);
    ledcWrite(PWM_CHANNEL2, 0);
}

void MotorControl::stop() {
    // Serial.println("Motor control - Stop");
    ledcWrite(PWM_CHANNEL1, 0);
    ledcWrite(PWM_CHANNEL2, 0);
}

void MotorControl::disable() {
    stop();
    digitalWrite(_enPin, LOW);
}

void MotorControl::enable() {
    digitalWrite(_enPin, HIGH);
}



