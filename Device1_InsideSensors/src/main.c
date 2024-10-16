/* Some informations:
Author: Sebastian Łuczak

Tech info:
Core 0 - wifi conn, mqtt and events related
Core 1 - sensors, uart, led, gpio and other not time-based functions
*/

// Includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "main.h"
#include "bmp280.h"
#include "ath20.h"
#include "i2c.h"
#include "connection.h"

// Variables
static const char *TAG = "MAIN";
StreamBufferHandle_t uartMessageBuff;
void *streamTxData;
char streamRxData[UART_MESSAGE_SIZE_BUFF];
uart_config_t uart_settings;
QueueHandle_t uart_queue;
char message[64];

// Functions prototypes
esp_err_t uartInit();
void uartSend(void *uartTX);
void hearthbeatLED(void *pvParameter);
void taskCheckATH20(void *pvParameter);
void taskCheckBMP280(void *pvParameter);
void taskInitWifi(void *pvParameter);
void initGPIOout(uint16_t pinNumber, uint32_t state);

// Main
void app_main()
{
    // INIT SECTION -----------------------------
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Just wait
    initGPIOout(PIN_LED, 0); // Init LED with state 1
    uartInit();
    ESP_LOGI(TAG, "START OF SERVER");
    // Init NVS
    esp_err_t nvs_return = nvs_flash_init();
    if (nvs_return == ESP_ERR_NVS_NO_FREE_PAGES || nvs_return == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        err_check(nvs_flash_erase());
        nvs_return = nvs_flash_init();
    }
    err_check(nvs_return);
    // Init WIFI
    //xTaskCreatePinnedToCore(&taskInitWifi, "taskInitWifi", 4096, NULL, tskIDLE_PRIORITY, NULL, PRO_CPU_NUM);
    taskInitWifi(NULL);
    // TODO  // Check connection to MQTT Broker
    i2c_init(I2C_PORT_NUM);                     // I2C Setup
    err_check(ath20_init(I2C_PORT_NUM));  // Init ATH20
    err_check(bmp280_init(I2C_PORT_NUM)); // Init BMP280
    // TODO // Init ADC for Air quality sensor

    // Start of heartbeat LED
    xTaskCreatePinnedToCore(&hearthbeatLED, "hearthbeatLED", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);
    // INIT END -------------------------------

    // Read data continuosly
    xTaskCreatePinnedToCore(&taskCheckATH20, "taskCheckATH20", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(&taskCheckBMP280, "taskCheckBMP280", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);
    // TODO: Task for measuring air quality

    while (1)
    {
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
        ESP_LOGI( TAG, "HEARTHBEAT");
    }
}

// Additional functions ------------------
// UART Functions
esp_err_t uartInit()
{
    // uart_config_t uart_settings;
    uart_settings.baud_rate = 115200;
    uart_settings.data_bits = UART_DATA_8_BITS; // TODO: Check
    uart_settings.parity = UART_PARITY_DISABLE;
    uart_settings.stop_bits = UART_STOP_BITS_1;
    uart_settings.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_settings.source_clk = UART_SCLK_DEFAULT;

    if (uart_param_config(UART_NUM, &uart_settings) != ESP_OK)
    {
        return ESP_FAIL;
    }
    if (uart_set_pin(UART_NUM, UART_TX, UART_RX, UART_RTS, UART_CTS) != ESP_OK)
    {
        return ESP_FAIL;
    }
    if (uart_driver_install(UART_NUM, UART_BUFFER_LENGTH, UART_BUFFER_LENGTH, 10, &uart_queue, 0) != ESP_OK)
    {
        return ESP_FAIL;
    }
    return ESP_OK;
}
void uartSend(void *uartTX)
{
    uart_write_bytes(UART_NUM, (char *)uartTX, strlen(uartTX));
    vTaskDelete(NULL);
}

// GPIO Functions
void initGPIOout(uint16_t pinNumber, uint32_t state)
{
    gpio_config_t pin;
    pin.intr_type = GPIO_INTR_DISABLE;
    pin.mode = GPIO_MODE_OUTPUT;
    pin.pin_bit_mask = (1ULL << pinNumber);
    pin.pull_down_en = 0;
    pin.pull_up_en = 0;
    gpio_config(&pin);
    gpio_set_level(pinNumber, state);
}
void hearthbeatLED(void *pvParameter)
{
    uint16_t pinNumber = PIN_LED;
    while (1)
    {
        gpio_set_level(pinNumber, 1);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        gpio_set_level(pinNumber, 0);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

// Tasks Functions
void taskCheckATH20(void *pvParameter)
{
    while (true)
    {
        float temperature;
        float humidity;
        ath20_read(I2C_PORT_NUM, &temperature, &humidity);
        ESP_LOGI(TAG, "ATH20 - Temp: %f, Hum: %f", temperature, humidity);
        vTaskDelay(ATH20_DELAY / portTICK_PERIOD_MS);
    }
}

void taskCheckBMP280(void *pvParameter)
{
    while (true)
    {
        float temperature;
        float pressure;
        bmp280_read(I2C_PORT_NUM, &temperature, &pressure);
        ESP_LOGI(TAG, "BMP280 - Temp: %f, Press: %f", temperature, pressure);
        vTaskDelay(BMP280_DELAY / portTICK_PERIOD_MS);
    }
}

void taskInitWifi(void *pvParameter)
{
    while (wifiInit() == ESP_FAIL)
    {
        ESP_LOGI(TAG, "WIFI INIT FAILED, RETRY");
    }
    ESP_LOGI(TAG, "WIFI INIT OKEY\n\r");
}

void taskInitBH1750(void *pvParameter)
{
    
}