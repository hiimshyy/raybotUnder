#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "config.h"
#include "info.h"
#include "distance_sensor.h"
#include "motor_control.h"
#include "handle_serial.h"

// DistanceSensor BACKWARD_SENSOR(ADC1_PIN, 1080);
DistanceSensor UNDER_SENSOR(ADC2_PIN, 1080);

MotorControl motorControl(PWM1_PIN, PWM2_PIN,EN_MT_PIN, PWM_FREQ, PWM_RESOLUTION);

HandleSerial handleSerial;

TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t handleMesTaskHandle = NULL;

QueueHandle_t receiveMsgQueue = NULL;
QueueHandle_t sendMsgQueue = NULL;

void readLimitSwitch();

void init_peripherals() {
    Serial.begin(UART_BAUD_RATE);
    Serial2.begin(QR_BAUD_RATE, SERIAL_8N1, 16, 17);
    if (!Serial || !Serial2) {
        Serial.println("Serial not started");
        while (1);
    }

    pinMode(LS1_PIN, INPUT_PULLUP);
    pinMode(LS2_PIN, INPUT_PULLUP);
}

void motor_control_task(void *pvParameters) {
    static bool lastRunning = false;
    static uint8_t lastSpeed = 0;
    static bool lastEnable = false;
    static bool lastDoorState = false;
    JsonDocument doc;

    while (1) {
        readLimitSwitch();
        bool isRunning = false;
        uint8_t currentSpeed = 0;
        bool stateChanged = false;

        // Kiểm tra trạng thái motor
        if(boxInfo.motorState == 1) {
            if (lastEnable != boxInfo.motorState) {
                motorControl.enable();
                lastEnable = boxInfo.motorState;
                stateChanged = true;
            }

            if (boxInfo.doorState == 0 && boxInfo.isOpen == true) {
                currentSpeed = 70;
                isRunning = true;
                motorControl.close(currentSpeed);
            } else if (boxInfo.doorState == 1 && boxInfo.isOpen == false) {
                currentSpeed = 80;
                isRunning = true;
                motorControl.open(currentSpeed);
            } else {
                motorControl.stop();
            }
        } else {
            if (lastEnable != boxInfo.motorState) {
                motorControl.disable();
                lastEnable = boxInfo.motorState;
                stateChanged = true;
            }
        }

        // Chỉ gửi message khi có thay đổi
        if (lastRunning != isRunning || 
            lastSpeed != currentSpeed || 
            lastDoorState != boxInfo.doorState || 
            stateChanged) {
            
            doc.clear();
            doc["state"] = boxInfo.doorState;
            doc["enable"] = boxInfo.motorState;
            doc["speed"] = currentSpeed;
            doc["is_running"] = isRunning;
            handleSerial.sendMsg(DOOR_STATE, doc.as<JsonObject>());

            // Cập nhật trạng thái
            lastRunning = isRunning;
            lastSpeed = currentSpeed;
            lastDoorState = boxInfo.doorState;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task đọc sensor
void sensor_task(void *pvParameters) {
    static JsonDocument sensorData;
    static uint8_t lastDistance = 0;
    while (1) {
        uint8_t currentDistance = UNDER_SENSOR.getDistance();
        boxInfo.distanceBackward = currentDistance;
        // Serial.printf("Distance: %d\n", currentDistance);

        // Only send if distance changed
        if (lastDistance != currentDistance) {
            sensorData.clear();
            sensorData["under"] = currentDistance;
            
            handleSerial.sendMsg(SENSOR_STATE, sensorData.as<JsonObject>());
            lastDistance = currentDistance;   
        }
    
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
}

// Task đọc QR code
void serialEvent2() {
    char data[BUFFER_SIZE];
    while (Serial2.available()) {
        int len = Serial2.readBytesUntil('\n', data, BUFFER_SIZE - 1);
        if (len > 0) {
            data[len] = '\0';
            String dataStr = String(data);
            dataStr.trim();
            JsonDocument qrData;
            qrData["code"] = dataStr;
            
            // Send formatted message using HandleSerial
            handleSerial.sendMsg(QR_STATE, qrData.as<JsonObject>());
        }
    }
}

void serialEvent() {
    char buffer[BUFFER_SIZE];
    while (Serial.available()) {
        int len = Serial.readBytesUntil('\n', buffer, BUFFER_SIZE - 1);
        if (len > 0) {
            buffer[len] = '\0';
            char* jsonStart = buffer;
            if (buffer[0] == '>') {
                jsonStart = buffer + 1;
            }
            char* message = strdup(jsonStart);
            Serial.printf("Serial Event - Received message: %s\n", message);
            if (xQueueSend(receiveMsgQueue, &message, pdMS_TO_TICKS(100)) != pdPASS) {
                Serial.println("Queue full, message dropped");
                free(message);
            }
        }
    }
}

void handleMesTask(void *pvParameters) {
    char* message;
    while (1) {
        // Serial.println("HandleMes - In handle message task");
        if (xQueueReceive(receiveMsgQueue, &message, pdMS_TO_TICKS(100)) == pdPASS) {
            HandleSerial::boxParams params = handleSerial.handleMsg(message);
            if (params.state != -1 || params.speed != -1 || params.enable != -1) {
                boxInfo.doorState = params.state;
                boxInfo.motorSpeed = params.speed;
                boxInfo.motorState = params.enable;
                // Serial.printf("HandleMes - Door State: %d, Motor Speed: %d, Motor State: %d\n", 
                //               boxInfo.doorState, boxInfo.motorSpeed, boxInfo.motorState);
            }
            free(message);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    init_peripherals();
    // Tạo queue
    receiveMsgQueue = xQueueCreate(100, sizeof(String));
    sendMsgQueue = xQueueCreate(100, sizeof(String));

    xTaskCreatePinnedToCore(
        motor_control_task,  // Hàm task
        "motor_task",        // Tên task
        2048,               // Stack size
        NULL,               // Tham số truyền vào
        1,                  // Độ ưu tiên
        &motorTaskHandle,   // Handle của task
        0                   // Gắn vào lõi 0
    );

    xTaskCreatePinnedToCore(
        sensor_task,
        "sensor_task",
        2048,
        NULL,
        1,
        &sensorTaskHandle,
        1                   // Gắn vào lõi 1
    );

    xTaskCreatePinnedToCore(
        handleMesTask,
        "handle_message_task",
        2048,
        NULL,
        1,
        &handleMesTaskHandle,
        1
    );

    Serial.println("Application started");
}

void loop() {
    // Để trống vì FreeRTOS quản lý các task
    // vTaskDelay(pdMS_TO_TICKS(1000));  // Tránh watchdog timeout
}

void readLimitSwitch() {
    boxInfo.limitSwitchOpen = digitalRead(LS1_PIN);
    boxInfo.limitSwitchClose = digitalRead(LS2_PIN);
    static bool currentState;
    // Serial.printf("Limit Switch Open: %d, Limit Switch Close: %d\n", boxInfo.limitSwitchOpen, boxInfo.limitSwitchClose);

    // Send formatted message using HandleSerial
    if (boxInfo.limitSwitchOpen == 0 && boxInfo.limitSwitchClose == 0) {
        currentState = true;
        // Serial.println(currentState);
        // Serial.println("Door is open");
    } else if (boxInfo.limitSwitchClose == 1 && boxInfo.limitSwitchOpen == 1) {
        currentState = false;
        // Serial.println(currentState);
        // Serial.println("Door is close");
    } 
    if(currentState != boxInfo.isOpen) {
        boxInfo.isOpen = currentState;
        JsonDocument doorState;
        doorState["state"] = boxInfo.isOpen;
        handleSerial.sendMsg(DOOR_STATE, doorState.as<JsonObject>());
    }
}
