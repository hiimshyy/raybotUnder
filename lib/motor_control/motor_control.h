#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

#define PWM_CHANNEL1   0
#define PWM_CHANNEL2   1

enum doorStateType
{
    OPEN,
    CLOSE,
    ERROR
} doorState;

enum motorStateType
{
    FORWARD,
    BACKWARD,
    STOP
} motorState;

class MotorControl {
public:
    MotorControl(uint8_t pwm1Pin, uint8_t  pwm2Pin, uint8_t enPin, uint16_t pwmFreq, uint8_t pwmRes);
    void disable();
    void enable();
private:
    uint8_t  _pwm1Pin;
    uint8_t  _pwm2Pin;
    uint8_t  _enPin;
    uint16_t  _pwmFreq;
    uint8_t  _pwmRes;

    void open(int speed);
    void close(int speed);
    void stop();
    float detecTarget(int distanceCM);
};

#endif