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
    digitalWrite(_enPin, LOW);
}

uint8_t MotorControl::detecTarget(uint8_t maxSpeed, uint8_t distance) {
    if (distance > 60) target = maxSpeed;
	else if (distance < 60 && distance > 30) target = (distance*maxSpeed)/60;
	else target = 0;
	return target;
}

void MotorControl::open(uint8_t speed) {
    //over 90% speed to open the door
    uint32_t _speed = map(speed, 0, 100, 1023, 0);
    // Serial.printf("Motor control - Open speed: %d\n", _speed);
    ledcWrite(PWM_CHANNEL1, 1023);
    // ledcWrite(PWM_CHANNEL2, 1023 - _speed);
    rampSpeed(1023, _speed, 10, PWM_CHANNEL2);

}

void MotorControl::close(uint8_t speed) {
    //over 70% speed to close the door
    uint32_t _speed = map(speed, 0, 100, 1023, 0);
    // Serial.printf("Motor control - Close speed: %d\n", _speed);
    rampSpeed(1023, _speed, 10, PWM_CHANNEL1);
    // ledcWrite(PWM_CHANNEL1, 1023 - _speed);
    ledcWrite(PWM_CHANNEL2, 1023);

}

void MotorControl::stop() {
    // Serial.println("Motor control - Stop");
    ledcWrite(PWM_CHANNEL1, 1023);
    ledcWrite(PWM_CHANNEL2, 1023);
}

void MotorControl::hold() {
    // Serial.println("Motor control - Hold");
    ledcWrite(PWM_CHANNEL1, 0);
    ledcWrite(PWM_CHANNEL2, 0);
}

void MotorControl::rampSpeed(uint32_t startSpeed, uint32_t targetSpeed, uint32_t steps, uint8_t channel) {
    // Serial.printf("Motor control - Ramp speed: %d\n", speed);
    int32_t speedDiff = targetSpeed - startSpeed;
    uint32_t delayMs = 50; // 50ms between steps
    for (uint8_t i = 0; i <= steps; i++) {
        uint32_t currentSpeed = startSpeed + (speedDiff * i / steps);
        ledcWrite(channel, currentSpeed);
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

void MotorControl::disable() {
    stop();
    digitalWrite(_enPin, LOW);
}

void MotorControl::enable() {
    digitalWrite(_enPin, HIGH);
}



