#ifndef HANDLE_SERIAL_H
#define HANDLE_SERIAL_H

#include <Arduino.h>

class HandleSerial {
public:
    HandleSerial();
    void serialEvent();
    void sendMsg(String messageToSend);
    String receiveMsg();
private:
    String _incomingMsg;
};

#endif