#ifndef HANDLE_SERIAL_H
#define HANDLE_SERIAL_H

#include <Arduino.h>
#include <ArduinoJson.h>

class HandleSerial {
public:
    HandleSerial();
    void sendMsg(const uint8_t state_type, const JsonObject data);                                   
    void senACK(const std::string id, const uint8_t status);
    void handleMsg(const char* msg);
private:

};

#endif