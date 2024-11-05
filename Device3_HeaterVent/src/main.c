/* Some informations:
Author: https://github.com/SebastianL-NT

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
//#include "bmp280.h"
//#include "ath20.h"
#include "i2c.h"
#include "connection.h"
#include "onewire.h"
#include "ds18x20.h"
#include "driver/ledc.h"

// Local defines
#define TOPIC_HEATER_TEMP "scada/heater_temp"
#define TOPIC_HEATER_FAN "scada/heater_fan"
#define TOPIC_HEATER_TEMP_IN "outside/ath20/temp"
#define PIN_FANCTRL 33
#define PIN_TEMPOUT 23
#define PIN_HEATERPWM 32

// Variables
static const char *TAG = "MAIN";
uart_config_t uart_settings;
QueueHandle_t uart_queue;
SemaphoreHandle_t i2c_semaphore = NULL;
uint64_t uptime = 0;
float heater_temp_in, heater_temp_out, heater_temp_req = 0;
int heater_fan_status, heater_fan_req = 0; // Set of variables for heater
onewire_search_t onewire_search;
onewire_addr_t temp_out_addr;
ledc_timer_config_t gpio_pwm_timer_conf;
ledc_channel_config_t gpio_pwm_channel_conf;

// Functions prototypes
esp_err_t uartInit();
void uartSend(void *uartTX);
void hearthbeatLED(void *pvParameter);
//static void taskCheckATH20(void *pvParameter);
//static void taskCheckBMP280(void *pvParameter);
void taskInitWifi(void *pvParameter);
void initGPIOout(uint16_t pinNumber, uint32_t state);
static void taskControlHeater(void *pvParameter);
static void taskHeaterOutTemp(void *pvParameter);
void mqttInterrupt(esp_mqtt_event_handle_t event);

// Main
void app_main()
{
    static char uptime_str[65];
    // Configs
    gpio_pwm_timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    gpio_pwm_timer_conf.duty_resolution = LEDC_TIMER_10_BIT;
    gpio_pwm_timer_conf.freq_hz = 20 * 1000; // 20kHz
    gpio_pwm_timer_conf.clk_cfg = LEDC_USE_APB_CLK;

    gpio_pwm_channel_conf.gpio_num = PIN_HEATERPWM;
    gpio_pwm_channel_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    gpio_pwm_channel_conf.channel = LEDC_CHANNEL_0;
    gpio_pwm_channel_conf.intr_type = LEDC_INTR_DISABLE;
    gpio_pwm_channel_conf.timer_sel = LEDC_TIMER_0;
    gpio_pwm_channel_conf.duty = 0;
    gpio_pwm_channel_conf.hpoint = 0;  

    // INIT SECTION -----------------------------
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Just wait
    initGPIOout(PIN_LED, 0);               // Init LED with state 0 (LED ENABLED)
    initGPIOout(PIN_FANCTRL, 0);               // Init LED with state 0 (FAN DISABLED)
    uartInit();
    mqttInterruptFunc = mqttInterrupt; // Hook up func to pointer
    ESP_LOGI(TAG, "INIT START");

    ESP_ERROR_CHECK(ledc_timer_config(&gpio_pwm_timer_conf));
    ESP_ERROR_CHECK(ledc_channel_config(&gpio_pwm_channel_conf));

    // Init NVS
    esp_err_t nvs_return = nvs_flash_init();
    if (nvs_return == ESP_ERR_NVS_NO_FREE_PAGES || nvs_return == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_return = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_return);
    // Init WIFI
    taskInitWifi(NULL);
    while(mqttClientStart() != ESP_OK) // Init MQTT Client, and retry if not working
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        ESP_LOGE(TAG, "MQTT Retrying to connect");
    }
    esp_mqtt_client_subscribe_single(mqttClient, TOPIC_HEATER_TEMP, 1); // Subscribe to topic
    esp_mqtt_client_subscribe_single(mqttClient, TOPIC_HEATER_FAN, 1); // Same
    esp_mqtt_client_subscribe_single(mqttClient, TOPIC_HEATER_TEMP_IN, 1);

    temp_out_addr = onewire_search_next(&onewire_search, PIN_TEMPOUT);
    if(temp_out_addr == ONEWIRE_NONE)
    {
        ESP_LOGI(TAG, "Onewire no devices found");
    }
    //else
    //{
    //    ESP_LOGI(TAG, "Onewire &lld", temp_out_addr);
    //}

    xTaskCreatePinnedToCore(&hearthbeatLED, "hearthbeatLED", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM); // Start of heartbeat LED

    i2c_semaphore = xSemaphoreCreateBinary(); // Create semaphore to sync tasks (mostly to avoid interrupts between sensors and display)
    xSemaphoreGive(i2c_semaphore);
    ESP_LOGI(TAG, "INIT END");
    // INIT END -------------------------------
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Create tasks
    //xTaskCreatePinnedToCore(&taskCheckATH20, "taskCheckATH20", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);
    //xTaskCreatePinnedToCore(&taskCheckBMP280, "taskCheckBMP280", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(&taskControlHeater, "taskControlHeater", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(&taskHeaterOutTemp, "taskHeaterOutTemp", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        uptime += 1;
        ESP_LOGI(TAG, "HEARTHBEAT");
        sprintf(uptime_str, "%llu", uptime);
        mqttPublish("heater/uptime", uptime_str);
    }
}

// Additional functions ------------------
// UART Functions
esp_err_t uartInit()
{
    // uart_config_t uart_settings;
    uart_settings.baud_rate = 115200;
    uart_settings.data_bits = UART_DATA_8_BITS;
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
/*static void taskCheckATH20(void *pvParameter)
{
    while (true)
    {
        if (i2c_semaphore != NULL)
        {
            if (xSemaphoreTake(i2c_semaphore, 1000 / portTICK_PERIOD_MS) == pdTRUE)
            {
                float temperature;
                float humidity;
                char message[6];

                ath20_read(I2C_PORT_NUM, &temperature, &humidity);
                ESP_LOGI(TAG, "ATH20 - Temp: %.1f, Hum: %.1f", temperature, humidity);
                ath20_temp = temperature;
                ath20_hum = humidity;

                sprintf(message, "%.1f", temperature);
                mqttPublish("heater/ath20/temp", message);
                sprintf(message, "%.1f", humidity);
                mqttPublish("heater/ath20/hum", message);

                dispLastRun = 0;
                xSemaphoreGive(i2c_semaphore);

                vTaskDelay(ATH20_DELAY / portTICK_PERIOD_MS);
            }
        }
    }
}

static void taskCheckBMP280(void *pvParameter)
{
    while (true)
    {
        if (i2c_semaphore != NULL)
        {
            if (xSemaphoreTake(i2c_semaphore, 1000 / portTICK_PERIOD_MS) == pdTRUE)
            {
                float temperature;
                float pressure;
                char message[7];

                bmp280_read(I2C_PORT_NUM, &temperature, &pressure);
                ESP_LOGI(TAG, "BMP280 - Temp: %.1f, Press: %.1f", temperature, pressure);
                bmp280_temp = temperature;
                bmp280_press = pressure;

                sprintf(message, "%.1f", temperature);
                mqttPublish("heater/bmp280/temp", message);
                sprintf(message, "%.1f", pressure);
                mqttPublish("heater/bmp280/press", message);

                dispLastRun = 0;
                xSemaphoreGive(i2c_semaphore);

                vTaskDelay(BMP280_DELAY / portTICK_PERIOD_MS);
            }
        }
    }
}*/

void taskInitWifi(void *pvParameter)
{
    while (wifiInit() == ESP_FAIL)
    {
        ESP_LOGI(TAG, "WIFI INIT FAILED, RETRY");
    }
    ESP_LOGI(TAG, "WIFI INIT OKEY\n\r");
}

static void taskControlHeater(void *pvParameter)
{
    while (true)
    {
        // TODO: Control for heater using simple K regulator.
        float delta_temp = heater_temp_req - heater_temp_out; // 10 celsius should be equal 100% pwm 
        int pwm = delta_temp * 100; // Min 0 (off), Max 100 (Full power)
        if (pwm > 100) { pwm = 100; }
        else if (pwm < 0) { pwm = 0; }
        ESP_LOGI(TAG, "In %f Out %f PWM %d Delta %f", heater_temp_in, heater_temp_out, pwm, delta_temp);
        //ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t) ((1024 / 100) * pwm), 0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t) ((1024 / 100) * pwm));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

        //if (pwm > 10) { heater_fan_req = 1; } // TEMPORARY FOR TESTS
        //else if (pwm < 10) { heater_fan_req = 0; } 

        if(heater_fan_status != heater_fan_req) // Do some checks if status is not equal
        { // TODO: Create GPIO control function
            if(heater_fan_req == 0) // disable fan
            {
                if(gpio_set_level(PIN_FANCTRL, 0) == ESP_OK) { heater_fan_status = 0; }
                else { ESP_LOGE(TAG, "Cannot control FAN"); }
            }
            else if(heater_fan_req == 1) // enable fan
            {
                if(gpio_set_level(PIN_FANCTRL, 1) == ESP_OK) { heater_fan_status = 1; }
                else { ESP_LOGE(TAG, "Cannot control FAN"); }
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void taskHeaterOutTemp(void *pvParameter)
{
    char msg[10];
    while (true)
    {
        ds18b20_measure_and_read(PIN_TEMPOUT, temp_out_addr, &heater_temp_out);
        sprintf(msg, "%.1f", heater_temp_out);
        mqttPublish("heater/temp_out", msg);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

// Interrupts
void mqttInterrupt(esp_mqtt_event_handle_t event)
{
    char topic[65];
    sprintf(topic, "%.*s", event->topic_len, event->topic);
    char data[65];
    sprintf(data, "%.*s", event->data_len, event->data);

    //ESP_LOGI(TAG, "%s", topic);
    
    if(strcmp(topic, TOPIC_HEATER_TEMP) == 0)
    {
        heater_temp_req = strtol(data, NULL, 10);
    }
    else if(strcmp(topic, TOPIC_HEATER_FAN) == 0)
    {
        heater_fan_req = strtol(data, NULL, 10); // 0 = disabled, 1 = enabled, other = error
    }
    else if(strcmp(topic, TOPIC_HEATER_TEMP_IN) == 0)
    {
        heater_temp_in = strtol(data, NULL, 10);
    }
}