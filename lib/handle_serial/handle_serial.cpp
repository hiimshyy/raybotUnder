#include "handle_serial.h"
#include "info.h"

HandleSerial::HandleSerial() {
    Serial.println("HandleSerial object created");
}

void HandleSerial::sendMsg(const uint8_t state_type, const JsonObject data) {
    StaticJsonDocument<200> doc;
    doc["type"] = 0;
    doc["state_type"] = state_type;
    doc["data"] = data;
    
    String jsonString;
    serializeJson(doc, jsonString);
    
    std::string formattedMsg = ">" + std::string(jsonString.c_str()) + "\r\n";
    Serial.println(formattedMsg.c_str());
}

void HandleSerial::senACK(const std::string id, const uint8_t status) {
    StaticJsonDocument<200> doc;
    doc["type"] = 1;
    doc["id"] = id;
    doc["status"] = status;
    
    String jsonString;
    serializeJson(doc, jsonString);
    
    std::string formattedMsg = ">" + std::string(jsonString.c_str()) + "\r\n";
    Serial.println(formattedMsg.c_str());
}

void HandleSerial::handleMsg(const char* msg) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
        Serial.printf("JSON parse error: %s\n", error.c_str());
        return;
    }
    const char* id = doc["id"];
    uint8_t type = doc["type"];
    JsonObject data = doc["data"];

    if (!id || !data) {
        Serial.println("Missing 'id' or 'data' field");
        return;
    }

    switch (type)
    {
    case 3: 
        boxInfo.doorState = data["state"];
        boxInfo.motorSpeed = data["speed"];
        boxInfo.motorState = data["enable"];
        break;
    
    default:
        Serial.println("I need msg type 3!");
        break;
    }
}