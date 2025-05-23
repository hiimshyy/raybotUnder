#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

#define PWM_CHANNEL1   0
#define PWM_CHANNEL2   1
#define MAX_SPEED      1023
#define MIN_SPEED      0

typedef struct motor_control_data
{
    uint8_t  speed;
    uint8_t  direction;
    uint8_t  step;
} motor_control_data_t;


enum class MotorState {
    STOPPED,
    OPENING,
    CLOSING,
    HOLDING
};

class MotorControl {
public:
    MotorControl(uint8_t pwm1Pin, uint8_t  pwm2Pin, uint8_t enPin, uint16_t pwmFreq, uint8_t pwmRes);
    void disable();
    void enable();
    void open(uint8_t speed);
    void close(uint8_t speed);
    void stop();
    void hold();
private:
    uint8_t     _pwm1Pin;
    uint8_t     _pwm2Pin;
    uint8_t     _enPin;
    uint16_t    _pwmFreq;
    uint8_t     _pwmRes;
    uint8_t     target;
    MotorState  _state;
    uint8_t     _direction;

    void rampSpeed(uint32_t startSpeed, uint32_t targetSpeed, uint32_t steps, uint8_t channel);
    uint8_t detecTarget(uint8_t maxSpeed, uint8_t distanceCM);
    //chuong
    void setMotorState(MotorState newState);
};

#endif