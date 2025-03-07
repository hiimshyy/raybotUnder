#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "config.h"
#include "info.h"
#include "distance_sensor.h"
#include "motor_control.h"

DistanceSensor BACKWARD_SENSOR(ADC1_PIN, 1080);
DistanceSensor FORWARD_SENSOR(ADC2_PIN, 1080);

MotorControl motorControl(PWM1_PIN, PWM2_PIN,EN_MT_PIN, PWM_FREQ, PWM_RESOLUTION);

InfoType boxInfo;

TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t qrTaskHandle = NULL;

QueueHandle_t receiveMsgQueue = NULL;
QueueHandle_t sendMsgQueue = NULL;

void init_peripherals() {
    // Cấu hình GPIO
    pinMode(EN_MT_PIN, OUTPUT);
    pinMode(LS1_PIN, INPUT_PULLUP);
    pinMode(LS2_PIN, INPUT_PULLUP);

    // Cấu hình PWM
    ledcSetup(PWM_CHANNEL1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL2, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWM1_PIN, PWM_CHANNEL1);
    ledcAttachPin(PWM2_PIN, PWM_CHANNEL2);

    // Cấu hình ADC (Arduino tự động xử lý)
    analogReadResolution(12);  // 12-bit ADC

    // Cấu hình UART1 cho debug (Serial)
    Serial.begin(UART_BAUD_RATE);

    // Cấu hình UART2 cho QR code (Serial2)
    Serial2.begin(UART_BAUD_RATE);
}

// Task điều khiển motor
void motor_control_task(void *pvParameters) {
    while (1) {
        digitalWrite(EN_MT_PIN, HIGH);
        ledcWrite(PWM_CHANNEL1, 512);  // 50% duty (1023 max for 10-bit)
        ledcWrite(PWM_CHANNEL2, 0);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task đọc sensor
void sensor_task(void *pvParameters) {
    while (1) {
        boxInfo.distanceForward = FORWARD_SENSOR.getDistance();
        boxInfo.distanceBackward = BACKWARD_SENSOR.getDistance();

        Serial.printf("Distance Forward: %d cm\n", boxInfo.distanceForward);
        Serial.printf("Distance Backward: %d cm\n", boxInfo.distanceBackward);
    
        vTaskDelay(pdMS_TO_TICKS(500));
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
                Serial.printf("QR Code: %s\n", data);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void send_queue(void *pvParameters) {
    std::string messageToSend; 
    while (1) {
        if (xQueueReceive(sendMsgQueue, &messageToSend, 0) == pdPASS) {
            Serial.println(messageToSend.c_str());
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void serialEvent() {
    static String incomingMsg;
    while (Serial.available()) {
        incomingMsg = Serial.readStringUntil('\n');
        incomingMsg.trim();
        if (incomingMsg.length() > 0) {
            ESP_LOGI("Serial", "Received: %s", incomingMsg.c_str());
            xQueueSend(receiveMsgQueue, &incomingMsg, 0);
            incomingMsg = "";
        }
        vTaskDelay(pdMS_TO_TICKS(1));
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
