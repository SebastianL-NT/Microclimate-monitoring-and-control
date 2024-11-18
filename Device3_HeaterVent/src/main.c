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
#include "i2c.h"
#include "connection.h"
#include "onewire.h"
#include "ds18x20.h"
#include "driver/ledc.h"
#include "math.h"

// Local defines
#define TOPIC_HEATER_TEMP "scada/heater_temp"
#define TOPIC_HEATER_FAN "scada/heater_fan"
#define TOPIC_HEATER_TEMP_IN "outside/aht20/temp"
#define TOPIC_HEATER_SCADA_EN "scada/heater_enable"
#define PIN_FANCTRL 33
#define PIN_TEMPOUT 23
#define PIN_HEATERPWM 32
#define SCADA_TIMEOUT 30

// Variables
static const char *TAG = "MAIN";
uart_config_t uart_settings;
QueueHandle_t uart_queue;
SemaphoreHandle_t i2c_semaphore = NULL;
uint64_t uptime = 0;
float heater_temp_in, heater_temp_out, heater_temp_req = 0;
int heater_fan_status, heater_fan_req, heater_fan_req_force, heater_enable = 0; // Set of variables for heater
onewire_search_t onewire_search;
onewire_addr_t temp_out_addr;
ledc_timer_config_t gpio_pwm_timer_conf;
ledc_channel_config_t gpio_pwm_channel_conf;
uint64_t timer = 0;
uint64_t scada_timeout = 0;

// Functions prototypes
esp_err_t uartInit();
void uartSend(void *uartTX);
void hearthbeatLED(void *pvParameter);
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
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    initGPIOout(PIN_LED, 0);
    initGPIOout(PIN_FANCTRL, 0);
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
    mqttInterruptEnabled = true;
    esp_mqtt_client_subscribe_single(mqttClient, TOPIC_HEATER_TEMP, 1);
    esp_mqtt_client_subscribe_single(mqttClient, TOPIC_HEATER_FAN, 1);
    esp_mqtt_client_subscribe_single(mqttClient, TOPIC_HEATER_TEMP_IN, 1);
    esp_mqtt_client_subscribe_single(mqttClient, TOPIC_HEATER_SCADA_EN, 1);

    temp_out_addr = onewire_search_next(&onewire_search, PIN_TEMPOUT);
    if(temp_out_addr == ONEWIRE_NONE)
    {
        ESP_LOGI(TAG, "Onewire no devices found");
    }

    xTaskCreatePinnedToCore(&hearthbeatLED, "hearthbeatLED", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);

    i2c_semaphore = xSemaphoreCreateBinary(); // Create semaphore to sync tasks (mostly to avoid interrupts between sensors and display)
    xSemaphoreGive(i2c_semaphore);
    ESP_LOGI(TAG, "INIT END");
    // INIT END -------------------------------
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Create tasks
    xTaskCreatePinnedToCore(&taskControlHeater, "taskControlHeater", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(&taskHeaterOutTemp, "taskHeaterOutTemp", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        uptime += 1;
        ESP_LOGI(TAG, "HEARTHBEAT");
        sprintf(uptime_str, "%llu", uptime);
        mqttPublish("heater/uptime", uptime_str);

        // 
        timer += 1;
        if (scada_timeout <= timer)
        {
            heater_enable = 0;
            heater_fan_req = 0;
            heater_temp_req = 0;
        }
    }
}

// Additional functions ------------------
// UART Functions
esp_err_t uartInit()
{
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
    char sPower[4];
    int K = 5;
    float delta_temp;
    int pwm = 0;
    uint64_t timer = 0;
    uint64_t time = 0;

    while (true)
    {
        if (heater_enable == 1)
        {
            if ((heater_temp_req > heater_temp_in) && (heater_fan_req == 0) && (timer == 0))
            {
                timer = time + 15000; // It is time that device will wait until enabling fan. It is for faster heater start-up
            }

            // PWM Calculating
            delta_temp = heater_temp_req - heater_temp_out;
            if (delta_temp > 1.0)
            {
                pwm = floor(((float) delta_temp * delta_temp)) * K;
                if (pwm > 100 ) {pwm = 100;} else if (pwm < 0) {pwm = 0;}
                if (time >= timer)
                {
                    heater_fan_req = 1;
                    timer = 0;
                }
                else
                {
                    heater_fan_req = 0;
                }
            }
            else if (delta_temp < -1.0)
            {
                pwm = 0;
                if (((float) heater_temp_in - heater_temp_out) < 0)
                {
                    heater_fan_req = 1;
                }
                else
                {
                    heater_fan_req = 0;
                }
            }
            else
            {
                heater_fan_req = 0;
            }
        }
        else
        {
            pwm = 0;
            heater_fan_req = 0;
            timer = 0;
        }

        if (heater_fan_req_force == 1)
        {
            heater_fan_req = 1;
        }

        // Heater Control
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t) ((1024 / 100) * pwm));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        sprintf(sPower, "%d", pwm);
        mqttPublish("heater/power", sPower);

        // Fan Control
        if(heater_fan_status != heater_fan_req)
        {
            if(heater_fan_req == 0) // disable fan
            {
                if(gpio_set_level(PIN_FANCTRL, 0) == ESP_OK) { heater_fan_status = 0; mqttPublish("heater/fan_fb", "0");}
                else { ESP_LOGE(TAG, "Cannot control FAN"); }
            }
            else if(heater_fan_req == 1) // enable fan
            {
                if(gpio_set_level(PIN_FANCTRL, 1) == ESP_OK) { heater_fan_status = 1; mqttPublish("heater/fan_fb", "1"); }
                else { ESP_LOGE(TAG, "Cannot control FAN"); }
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
        time = time + 100;
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
        heater_fan_req_force = strtol(data, NULL, 10); // 0 = disabled, 1 = enabled, other = error
    }
    else if(strcmp(topic, TOPIC_HEATER_SCADA_EN) == 0)
    {
        heater_enable = strtol(data, NULL, 10);
        scada_timeout = timer + SCADA_TIMEOUT;
    }
    else if(strcmp(topic, TOPIC_HEATER_TEMP_IN) == 0)
    {
        heater_temp_in = strtol(data, NULL, 10);
    }
}