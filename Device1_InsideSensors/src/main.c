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
#include "esp_adc/adc_continuous.h"

// Variables
static const char *TAG = "MAIN";
uart_config_t uart_settings;
QueueHandle_t uart_queue;
adc_continuous_handle_cfg_t adc_init_config = {
    .max_store_buf_size = 1024,
    .conv_frame_size = ADC_BUFFER_LEN
};
static adc_continuous_handle_t adc_handle;
static adc_continuous_config_t adc_config;
static adc_digi_pattern_config_t adc_digi_conf;
static adc_channel_t adc_channel = ADC_CHANNEL_0;
static TaskHandle_t s_task_handle;
static uint8_t adc_results[ADC_BUFFER_LEN] = {0};

// Functions prototypes
esp_err_t uartInit();
void uartSend(void *uartTX);
void hearthbeatLED(void *pvParameter);
static void taskCheckATH20(void *pvParameter);
static void taskCheckBMP280(void *pvParameter);
static void taskCheckAIR(void *pvParameter);
void taskInitWifi(void *pvParameter);
void initGPIOout(uint16_t pinNumber, uint32_t state);
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);

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
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_return = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_return);
    // Init WIFI
    //xTaskCreatePinnedToCore(&taskInitWifi, "taskInitWifi", 4096, NULL, tskIDLE_PRIORITY, NULL, PRO_CPU_NUM);
    taskInitWifi(NULL);
    // TODO  // Check connection to MQTT Broker
    i2c_init(I2C_PORT_NUM);                     // I2C Setup
    ESP_ERROR_CHECK(ath20_init(I2C_PORT_NUM));  // Init ATH20
    ESP_ERROR_CHECK(bmp280_init(I2C_PORT_NUM)); // Init BMP280

    // Init ADC for Air quality sensor
    s_task_handle = xTaskGetCurrentTaskHandle();
    memset(adc_results, 0xCC, ADC_BUFFER_LEN);

    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_init_config, &adc_handle));

    adc_digi_conf.atten = ADC_ATTEN_DB_0;
    adc_digi_conf.channel = adc_channel & 0x7;
    adc_digi_conf.unit = ADC_UNIT_1;
    adc_digi_conf.bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    adc_config.pattern_num = 1;
    adc_config.adc_pattern = &adc_digi_conf;
    adc_config.sample_freq_hz = 20000; // Hz
    adc_config.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    adc_config.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1;

    adc_continuous_config(adc_handle, &adc_config);
    adc_continuous_evt_cbs_t adc_cbs; // ?
    adc_cbs.on_conv_done = s_conv_done_cb;

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &adc_cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));    

    // Start of heartbeat LED
    xTaskCreatePinnedToCore(&hearthbeatLED, "hearthbeatLED", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);
    // INIT END -------------------------------

    // Read data continuosly
    xTaskCreatePinnedToCore(&taskCheckATH20, "taskCheckATH20", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(&taskCheckBMP280, "taskCheckBMP280", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(&taskCheckAIR, "taskCheckAIR", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);

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

// ADC Functions
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
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
        mqttPublish("inside/ath20/temp", message);
        sprintf(message, "%2.1f", humidity);
        mqttPublish("inside/ath20/hum", message);
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
        mqttPublish("inside/bmp280/temp", message);
        sprintf(message, "%4.1f", pressure);
        mqttPublish("inside/bmp280/press", message);
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

static void taskCheckAIR(void *pvParameter)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (1)
        {
            //uint32_t results[ADC_BUFFER_LEN] = {0};
            uint64_t result_avg = 0;
            uint32_t results_count = 0;
            char message[64];
            esp_err_t ret;

            ret = adc_continuous_read(adc_handle, adc_results, ADC_BUFFER_LEN, &results_count, 0);
            if (ret == ESP_OK)
            {
                for (int i = 0; i < results_count; i++)
                {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&adc_results[i];
                    result_avg += ((p)->type1.data);
                }
                result_avg = result_avg / 10;
                ESP_LOGI(TAG, "ADC Read: %llu", result_avg);

                sprintf(message, "%llu", result_avg);
                mqttPublish("inside/air/quality", message);

            }
            else
            {
                ESP_LOGE(TAG, "ADC Error: %d", ret);
                break;
            }

        }
    }
}