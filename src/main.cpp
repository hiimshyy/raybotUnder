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
TaskHandle_t qrTaskHandle = NULL;
TaskHandle_t handleMesTaskHandle = NULL;
TaskHandle_t serialEventTaskHandle = NULL;

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

// Task điều khiển motor
void motor_control_task(void *pvParameters) {
    while (1) {
        // readLimitSwitch();
        // Serial.println();
        // xử lý trường hợp enable motor
        if(boxInfo.motorState == 1) {
            motorControl.enable();
            if (boxInfo.doorState == 0 && boxInfo.isOpen == true) {
                motorControl.close(boxInfo.motorSpeed);
            } else if (boxInfo.doorState == 1 && boxInfo.isOpen == false) {
                motorControl.open(boxInfo.motorSpeed);
            }
        } else {
            motorControl.disable();
        }
        // xử lý trường hợp limit switch
        if(boxInfo.limitSwitchOpen == 1 || boxInfo.limitSwitchClose == 1) {
            motorControl.stop();
        } 
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task đọc sensor
void sensor_task(void *pvParameters) {
    // static JsonDocument sensorData;
    while (1) {
        // boxInfo.distanceForward = FORWARD_SENSOR.getDistance();
        boxInfo.distanceBackward = UNDER_SENSOR.getDistance();

        // Send formatted message using HandleSerial
        // sensorData.clear();
        // sensorData["under"] = boxInfo.distanceUnder;
        // sensorData["backward"] = boxInfo.distanceBackward;
        
        // handleSerial.sendMsg(SENSOR_STATE, sensorData.as<JsonObject>());

        Serial.printf("Distance Under: %d cm\n", boxInfo.distanceUnder);
        // Serial.printf("Distance Backward: %d cm\n", boxInfo.distanceBackward);
    
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task đọc QR code
void serialEvent2() {
    char data[BUFFER_SIZE];
    while (Serial2.available()) {
        int len = Serial2.readBytesUntil('\n', data, BUFFER_SIZE - 1);
        if (len > 0) {
            data[len] = '\0';
            // Serial.printf("QR task - QR Code: %s\n", data);
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
                Serial.printf("HandleMes - Door State: %d, Motor Speed: %d, Motor State: %d\n", 
                              boxInfo.doorState, boxInfo.motorSpeed, boxInfo.motorState);
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

    // xTaskCreatePinnedToCore(
    //     sensor_task,
    //     "sensor_task",
    //     1024,
    //     NULL,
    //     1,
    //     &sensorTaskHandle,
    //     1                   // Gắn vào lõi 1
    // );

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

    // Send formatted message using HandleSerial
    if (boxInfo.limitSwitchOpen == 1) {
        boxInfo.isOpen = true;
    } else if (boxInfo.limitSwitchClose == 1) {
        boxInfo.isOpen = false;
    } 
}
