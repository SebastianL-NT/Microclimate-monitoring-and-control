/* Some informations:
Author: Sebastian Łuczak

Tech info:
Core 0 - wifi conn, mqtt and events related
Core 1 - sensors, uart, led, gpio and other not time-based functions
*/

// Includes
#include "settings.h"
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
//TODO #include "bh1750.h"
#include "i2c.h"
#include "connection.h"

// Variables
static const char *TAG = "MAIN";
static uart_config_t uart_settings;
static QueueHandle_t uart_queue;

// Functions prototypes
esp_err_t uartInit();
static void hearthbeatLED(void *pvParameter);
static void taskCheckATH20(void *pvParameter);
static void taskCheckBMP280(void *pvParameter);
void taskInitWifi(void *pvParameter);
void initGPIOout(uint16_t pinNumber, uint32_t state);

// Main
void app_main()
{
    // INIT SECTION -----------------------------
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Just wait
    initGPIOout(PIN_LED, 0);
    ESP_ERROR_CHECK(uartInit());
    ESP_LOGI(TAG, "START OF SERVER");
    // Init NVS
    esp_err_t nvs_return = nvs_flash_init();
    if (nvs_return == ESP_ERR_NVS_NO_FREE_PAGES || nvs_return == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_return = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_return);
    // Init WIFI
    //xTaskCreatePinnedToCore(&taskInitWifi, "taskInitWifi", 4096, NULL, tskIDLE_PRIORITY, NULL, PRO_CPU_NUM);
    taskInitWifi(NULL);
    mqttClientStart();  // Init MQTT Client
    ESP_ERROR_CHECK(i2c_init(I2C_PORT_NUM));    // I2C Setup
    ESP_ERROR_CHECK_WITHOUT_ABORT(ath20_init(I2C_PORT_NUM));  // Init ATH20
    ESP_ERROR_CHECK_WITHOUT_ABORT(bmp280_init(I2C_PORT_NUM)); // Init BMP280
    // TODO // Init DH1750

    // Start of heartbeat LED
    xTaskCreatePinnedToCore(&hearthbeatLED, "hearthbeatLED", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);
    // INIT END -------------------------------

    // Read data continuosly
    xTaskCreatePinnedToCore(&taskCheckATH20, "taskCheckATH20", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(&taskCheckBMP280, "taskCheckBMP280", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);
    // TODO: Task for measuring daylight

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
    uart_settings.baud_rate = 9600;
    uart_settings.data_bits = UART_DATA_8_BITS;
    uart_settings.parity = UART_PARITY_DISABLE;
    uart_settings.stop_bits = UART_STOP_BITS_1;
    uart_settings.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_settings.source_clk = UART_SCLK_DEFAULT;

    err_check(uart_param_config(UART_NUM, &uart_settings));
    err_check(uart_set_pin(UART_NUM, UART_TX, UART_RX, UART_RTS, UART_CTS));
    err_check(uart_driver_install(UART_NUM, UART_BUFFER_LENGTH, UART_BUFFER_LENGTH, 10, &uart_queue, 0));

    return ESP_OK;
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
static void hearthbeatLED(void *pvParameter)
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
static void taskCheckATH20(void *pvParameter)
{
    while (true)
    {
        float temperature;
        float humidity;
        char message[6];

        ath20_read(I2C_PORT_NUM, &temperature, &humidity);
        ESP_LOGI(TAG, "ATH20 - Temp: %f, Hum: %f", temperature, humidity);
        
        sprintf(message, "%2.1f", temperature);
        mqttPublish("outside/ath20/temp", message);
        sprintf(message, "%2.1f", humidity);
        mqttPublish("outside/ath20/hum", message);
        vTaskDelay(ATH20_DELAY / portTICK_PERIOD_MS);
    }
}

static void taskCheckBMP280(void *pvParameter)
{
    while (true)
    {
        float temperature;
        float pressure;
        char message[7];

        bmp280_read(I2C_PORT_NUM, &temperature, &pressure);
        ESP_LOGI(TAG, "BMP280 - Temp: %f, Press: %f", temperature, pressure);

        sprintf(message, "%2.1f", temperature);
        mqttPublish("outside/bmp280/temp", message);
        sprintf(message, "%4.1f", pressure);
        mqttPublish("outside/bmp280/press", message);
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