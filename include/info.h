#ifndef INFO_H
#define INFO_H

#include <cstdint>

struct  InfoType
{    
    uint8_t  doorState;
    uint8_t  motorState;
    uint8_t  motorSpeed;
    uint8_t  distanceForward;
    uint8_t  distanceBackward;
    String   qrCode;
    uint8_t  limitSwitch1;
    uint8_t  limitSwitch2;
    bool     isSafe;
} boxInfo;

enum StateType{
    MOTOR_STATE = 6,
    QR_STATE,
    SENSOR_STATE,
} stateType;

#endif

