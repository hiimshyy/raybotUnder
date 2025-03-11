#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "config.h"
#include "info.h"
#include "distance_sensor.h"
#include "motor_control.h"
#include "handle_serial.h"

DistanceSensor BACKWARD_SENSOR(ADC1_PIN, 1080);
DistanceSensor FORWARD_SENSOR(ADC2_PIN, 1080);

MotorControl motorControl(PWM1_PIN, PWM2_PIN,EN_MT_PIN, PWM_FREQ, PWM_RESOLUTION);

HandleSerial handleSerial;

TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t qrTaskHandle = NULL;
TaskHandle_t handleMesTaskHandle = NULL;

QueueHandle_t receiveMsgQueue = NULL;
QueueHandle_t sendMsgQueue = NULL;

void init_peripherals() {
    Serial.begin(UART_BAUD_RATE);
    Serial2.begin(UART_BAUD_RATE);
    if (!Serial) {
        Serial.println("Serial not started");
        while (1);
    }

    pinMode(LS1_PIN, INPUT_PULLUP);
    pinMode(LS2_PIN, INPUT_PULLUP);
}

// Task điều khiển motor
void motor_control_task(void *pvParameters) {
    while (1) {
        readLimitSwitch();
        // xử lý trường hợp enable motor
        if(boxInfo.motorState == 1) {
            motorControl.enable();
            uint8_t speed = motorControl.detecTarget(boxInfo.motorSpeed, boxInfo.distanceBackward);
            if (boxInfo.doorState == 0) {
                motorControl.close(speed);
            } else {
                motorControl.open(speed);
            }
        } else {
            motorControl.disable();
        }
        // xử lý trường hợp limit switch
        if(boxInfo.limitSwitch1 == 1 || boxInfo.limitSwitch2 == 1) {
            motorControl.stop();
        } 
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task đọc sensor
void sensor_task(void *pvParameters) {
    while (1) {
        boxInfo.distanceForward = FORWARD_SENSOR.getDistance();
        boxInfo.distanceBackward = BACKWARD_SENSOR.getDistance();

        Serial.printf("Distance Forward: %d cm\n", boxInfo.distanceForward);
        Serial.printf("Distance Backward: %d cm\n", boxInfo.distanceBackward);
    
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task đọc QR code
void qr_task(void *pvParameters) {
    char data[BUFFER_SIZE];
    while (1) {
        if (Serial2.available()) {
            int len = Serial2.readBytesUntil('\n', data, BUFFER_SIZE - 1);
            if (len > 0) {
                data[len] = '\0';
                // Serial.printf("QR Code: %s\n", data);
                ESP_LOGI("QR", "QR Code: %s", data);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void send_queue(void *pvParameters) {
    std::string messageToSend; 
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void serialEvent() {
    char buffer[BUFFER_SIZE];
    while (1) {
        if (Serial.available()) {
            int len = Serial.readBytesUntil('\n', buffer, BUFFER_SIZE - 1);
            if (len > 0) {
                buffer[len] = '\0';
                // Bỏ ký tự '>' ở đầu nếu có
                char* jsonStart = buffer;
                if (buffer[0] == '>') {
                    jsonStart = buffer + 1;
                }
                char* message = strdup(jsonStart);
                if (xQueueSend(receiveMsgQueue, &message, pdMS_TO_TICKS(100)) != pdPASS) {
                    Serial.println("Queue full, message dropped");
                    free(message);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void handleMesTask(void *pvParameters) {
    char* message;
    while (1) {
        if (xQueueReceive(receiveMsgQueue, &message, pdMS_TO_TICKS(100)) == pdPASS) {
            handleSerial.handleMsg(message);
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
        512,
        NULL,
        1,
        &sensorTaskHandle,
        1                   // Gắn vào lõi 1
    );

    xTaskCreatePinnedToCore(
        qr_task,
        "qr_task",
        2048,
        NULL,
        1,
        &qrTaskHandle,
        1                   // Gắn vào lõi 1
    );

    Serial.println("Application started");
}

void loop() {
    // Để trống vì FreeRTOS quản lý các task
    vTaskDelay(pdMS_TO_TICKS(1000));  // Tránh watchdog timeout
}

void readLimitSwitch() {
    boxInfo.limitSwitch1 = digitalRead(LS1_PIN);
    boxInfo.limitSwitch2 = digitalRead(LS2_PIN);
}
