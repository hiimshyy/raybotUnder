#include "handle_serial.h"

HandleSerial::HandleSerial() {
    Serial.println("HandleSerial object created");
}

void HandleSerial::sendMsg(const uint8_t state_type, const JsonObject data) {
    JsonDocument doc;
    doc["type"] = 0;
    doc["state_type"] = state_type;
    doc["data"] = data;
    
    String jsonString;
    jsonString.clear();
    serializeJson(doc, jsonString);
    
    std::string formattedMsg = ">" + std::string(jsonString.c_str()) + "\r\n";
    Serial.print(formattedMsg.c_str());
}

void HandleSerial::sendACK(const std::string id, const uint8_t status) {
    JsonDocument doc;
    doc["type"] = 1;
    doc["id"] = id;
    doc["status"] = status;
    
    String jsonString;
    jsonString.clear();
    serializeJson(doc, jsonString);
    
    std::string formattedMsg = ">" + std::string(jsonString.c_str()) + "\r\n";
    Serial.print(formattedMsg.c_str());
}

HandleSerial::boxParams HandleSerial::handleMsg(const char* msg) {
    boxParams params = {-1, -1, -1};
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
        // ESP_LOGE("HandleSerial", "JSON parse error: %s", error.c_str());
    } 
    else {
        const char* id = doc["id"];
        uint8_t type = doc["type"];
        JsonObject data = doc["data"];

        if (!id || !data) {
            // Serial.println("Missing 'id' or 'data' field");
            sendACK(id, 0);
        }

        if (type == 0) {
            sendACK(id, 1);
            params.state = data["state"];
            params.speed = data["speed"];
            params.enable = data["enable"];
            return params;
        } else {
            sendACK(id, 0);
        }
    }
    
    return params;
}