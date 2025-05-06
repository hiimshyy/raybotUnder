#ifndef CONFIG_H
#define CONFIG_H

// Định nghĩa các chân GPIO
#define EN_MT_PIN      21
#define PWM1_PIN       19
#define PWM2_PIN       18
#define LS1_PIN        34
#define LS2_PIN        35
#define ADC1_PIN       26
#define ADC2_PIN       27

// Cấu hình PWM
#define PWM_FREQ       5000
#define PWM_RESOLUTION 10  // 10-bit resolution

// Cấu hình UART
#define UART_BAUD_RATE 9600
#define QR_BAUD_RATE   9600
#define BUFFER_SIZE    128

#endif