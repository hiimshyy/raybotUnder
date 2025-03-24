#ifndef HANDLE_SERIAL_H
#define HANDLE_SERIAL_H

#include <Arduino.h>
#include <ArduinoJson.h>

class HandleSerial {
public:
    struct boxParams {
        int state;
        int speed;
        int enable;
    };

    HandleSerial();
    void sendMsg(const uint8_t state_type, const JsonObject data);                                   
    boxParams handleMsg(const char* msg);
private:
    void sendACK(const std::string id, const uint8_t status);

};

#endif