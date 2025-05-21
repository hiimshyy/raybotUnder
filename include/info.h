#ifndef INFO_H
#define INFO_H

#include <cstdint>

struct  InfoType
{    
    uint8_t  doorState;     // 0: close, 1: open, 2 : idle
    uint8_t  motorState;    // 0: disable, 1: enablxe
    uint8_t  motorSpeed;    // 0-100
    uint8_t  distanceUnder;
    uint8_t  distanceObject;
    String   qrCode;       
    uint8_t  limitSwitchOpen; 
    uint8_t  limitSwitchClose;
    bool     isOpen;        // true: open, false: close
} boxInfo;

enum StateType{
    DOOR_STATE = 0,
    MOTOR_STATE ,
    QR_STATE,
    SENSOR_STATE,
} state_type;

// enum DoorState{
//     CLOSE = 0,
//     OPEN,
//     IDLE,
// } door_state;

#endif

