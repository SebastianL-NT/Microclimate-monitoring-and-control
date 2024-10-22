/*
Common settings for projects

Written by PLSBX
*/
#ifndef SETTINGS_H
#define SETTINGS_H

// UART SETTINGS
#define UART_MESSAGE_SIZE_BUFF 64 // Size of buffer for xStream in RTOS
#define UART_NUM UART_NUM_0 // Set what UART port to use
#define UART_TX 1 // U2 TXD: 17, U0 TXD: 1
#define UART_RX 3 // U2 RXD: 16, U0 RXD: 3
#define UART_RTS UART_PIN_NO_CHANGE
#define UART_CTS UART_PIN_NO_CHANGE
#define UART_BUFFER_LENGTH (1024 * 2)

// NORMAL GPIO SETTINGS
#define PIN_LED 22 // GPIO22

// I2C SETTINGS
#define I2C_PORT_NUM 0

// WIFI Settings
#define WIFI_SSID "mywifi"
#define WIFI_PASSWORD "mypassword"
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define EXAMPLE_ESP_MAXIMUM_RETRY  10

// MQTT Settings
#define CONFIG_BROKER_URL "mqtt://192.168.1.12/"

// Task delays
#define ATH20_DELAY 2000
#define BMP280_DELAY 2000
#define BH1750_DELAY 2000

// ADC Settings
#define ADC_BUFFER_LEN 64

// ESP Status check macro
#define err_check( status ) if(status != ESP_OK) { return ESP_FAIL; }

#endif
