#ifndef CONFIG_H
#define CONFIG_H

// Định nghĩa các chân GPIO
#define EN_MT_PIN      19
#define PWM1_PIN       18
#define PWM2_PIN       5
#define LS1_PIN        27
#define LS2_PIN        26
#define ADC1_PIN       32
#define ADC2_PIN       33

// Cấu hình PWM
#define PWM_FREQ       5000
#define PWM_RESOLUTION 10  // 10-bit resolution

// Cấu hình UART
#define UART_BAUD_RATE 115200
#define QR_BAUD_RATE   9600
#define BUFFER_SIZE    128

#endif