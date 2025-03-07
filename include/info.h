#include <cstdint>

struct  InfoType
{
    uint8_t  motorState;
    uint8_t  motorSpeed;
    uint8_t  doorState;
    int      distanceForward;
    int      distanceBackward;
    uint8_t  qrCode;
    bool     isSafe;
};

