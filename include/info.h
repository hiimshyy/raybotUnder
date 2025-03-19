#ifndef INFO_H
#define INFO_H

#include <cstdint>

struct  InfoType
{    
    uint8_t  doorState;     // 0: close, 1: open
    uint8_t  motorState;    // 0: disable, 1: enable
    uint8_t  motorSpeed;    // 0-100
    uint8_t  distanceUnder;
    uint8_t  distanceBackward;
    String   qrCode;       
    uint8_t  limitSwitchOpen; 
    uint8_t  limitSwitchClose;
    bool     isSafe;
    bool     isOpen = false;
} boxInfo;

enum StateType{
    DOOR_STATE = 0,
    MOTOR_STATE ,
    QR_STATE,
    SENSOR_STATE,
} state_type;

#endif

