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
    Serial.printf("Motor control - Open speed: %d\n", _speed);
    ledcWrite(PWM_CHANNEL1, 1023);
    // ledcWrite(PWM_CHANNEL2, 1023 - _speed);
    rampSpeed(800, _speed, 50, PWM_CHANNEL2);

}

void MotorControl::close(uint8_t speed) {
    //over 70% speed to close the door
    uint32_t _speed = map(speed, 0, 100, 1023, 0);
    Serial.printf("Motor control - Close speed: %d\n", _speed);
    ledcWrite(PWM_CHANNEL1, 1023);
    ledcWrite(PWM_CHANNEL2, 1023);
    rampSpeed(1023, _speed, 50, PWM_CHANNEL1);
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
    // Ép kiểu sang int32_t để cho phép số âm
     int32_t sSpeed = (int32_t)startSpeed;
     int32_t tSpeed = (int32_t)targetSpeed;
    // int32_t speedDiff = tSpeed - sSpeed;
    int16_t acc= (sSpeed - tSpeed) / steps;



    Serial.printf("Motor control - Start speed: %d\n", sSpeed);
    Serial.printf("Motor control - Target speed: %d\n", tSpeed);
    //Serial.printf("Motor control - Speed diff: %d\n", speedDiff);
    Serial.printf("Motor control - Steps: %d\n", steps);
    Serial.printf("Motor control - Channel: %d\n", channel);

    uint16_t delayMs = 100; // 50ms between steps
    for (int8_t i = 0; i <= steps; i++) {
      //  int32_t currentSpeed = sSpeed + (int32_t)(speedDiff * ((float)i / steps));
        int32_t currentSpeed = sSpeed - (acc*i);
        if (currentSpeed <=0 )
        {
           currentSpeed = 0;
            /* code */
        }
        
        // // Clamp lại trong khoảng hợp lệ [0, 1023]
        // if (currentSpeed < 0) currentSpeed = 0;
        // if (currentSpeed > 1023) currentSpeed = 1023;

        Serial.printf("Motor control - Current speed: %d\n", currentSpeed);
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

void MotorControl::setMotorState(MotorState newState) {
    _state = newState;
   // Serial.printf("Motor control - State changed to: %d\n", (int)_state);
}