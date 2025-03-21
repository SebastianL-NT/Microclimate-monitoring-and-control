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
#include "bmp280.h"
#include "aht20.h"
#include "i2c.h"
#include "connection.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "ssd1306.h"

// Variables
static const char *TAG = "MAIN";
uart_config_t uart_settings;
QueueHandle_t uart_queue;
/*adc_continuous_handle_cfg_t adc_init_config = {
    .max_store_buf_size = 1024,
    .conv_frame_size = ADC_BUFFER_LEN
};
static adc_continuous_handle_t adc_handle;
adc_continuous_config_t adc_config;
adc_digi_pattern_config_t adc_digi_conf[SOC_ADC_PATT_LEN_MAX] = {0};
static TaskHandle_t s_task_handle;
static uint8_t adc_results[ADC_BUFFER_LEN] = {0};
static adc_continuous_evt_cbs_t adc_cbs; // ?
adc_cali_handle_t adc_cali_handle;*/
float bmp280_temp, bmp280_press, aht20_temp, aht20_hum = 0;
SemaphoreHandle_t i2c_semaphore = NULL;
int task_queue = 0;
uint8_t dispLastRun = 0;
uint64_t uptime = 0;

// Functions prototypes
esp_err_t uartInit();
void uartSend(void *uartTX);
void hearthbeatLED(void *pvParameter);
static void taskCheckaht20(void *pvParameter);
static void taskCheckBMP280(void *pvParameter);
//static void taskCheckAIR(void *pvParameter);
static void taskDisplay(void *pvParameter);
void taskInitWifi(void *pvParameter);
void initGPIOout(uint16_t pinNumber, uint32_t state);
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);
void mqttInterrupt(esp_mqtt_event_handle_t event);

// Main
void app_main()
{
    static char uptime_str[65];
    // INIT SECTION -----------------------------
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Just wait
    initGPIOout(PIN_LED, 0);               // Init LED with state 1
    uartInit();
    ESP_LOGI(TAG, "INIT START");

    // Init I2C and related sensors (moved before WIFI just to have nice start-up screen longer)
    ESP_ERROR_CHECK(i2c_init(I2C_PORT_NUM));    // I2C Setup
    ESP_ERROR_CHECK(aht20_init(I2C_PORT_NUM));  // Init aht20
    ESP_ERROR_CHECK(bmp280_init(I2C_PORT_NUM)); // Init BMP280
    ESP_ERROR_CHECK(ssd1306_init()); // Init oled screen
    xTaskCreate(&task_ssd1306_display_clear, "ssd1306_display_clear", 2048, NULL, 6, NULL);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    xTaskCreate(&task_ssd1306_display_text, "ssd1306_display_text", 2048, (void *)"Inside Sensor\nStart up", 6, NULL);

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
    ESP_ERROR_CHECK(mqttClientStart()); // Init MQTT Client

    // Init ADC for Air quality sensor
    /*adc_cali_line_fitting_config_t adc_cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_0,
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&adc_cali_cfg, &adc_cali_handle));
    //initGPIOout(PIN_AIR_EN, 0); // Init LED with state 1
    s_task_handle = xTaskGetCurrentTaskHandle();
    memset(adc_results, 0xCC, ADC_BUFFER_LEN);

    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_init_config, &adc_handle));

    adc_digi_conf[0].atten = ADC_ATTEN_DB_0;
    adc_digi_conf[0].channel = ADC_CHANNEL_5 & 0x7;
    adc_digi_conf[0].unit = ADC_UNIT_1;
    adc_digi_conf[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    adc_config.pattern_num = 1;
    adc_config.adc_pattern = &adc_digi_conf;
    adc_config.sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_HIGH; // Hz
    adc_config.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    adc_config.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1;

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_config));
    adc_cbs.on_conv_done = s_conv_done_cb;

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &adc_cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle)); */

    xTaskCreatePinnedToCore(&hearthbeatLED, "hearthbeatLED", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM); // Start of heartbeat LED

    i2c_semaphore = xSemaphoreCreateBinary(); // Create semaphore to sync tasks (mostly to avoid interrupts between sensors and display)
    xSemaphoreGive(i2c_semaphore);
    ESP_LOGI(TAG, "INIT END");
    // INIT END -------------------------------
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Read data continuosly
    xTaskCreatePinnedToCore(&taskCheckaht20, "taskCheckaht20", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(&taskCheckBMP280, "taskCheckBMP280", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);
    // xTaskCreatePinnedToCore(&taskCheckAIR, "taskCheckAIR", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(&taskDisplay, "taskDisplay", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        uptime += 1;
        ESP_LOGI(TAG, "HEARTHBEAT");
        sprintf(uptime_str, "%llu", uptime);
        mqttPublish("inside/uptime", uptime_str);
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

// ADC Functions
/*static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}*/

// Tasks Functions
static void taskCheckaht20(void *pvParameter)
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

                aht20_read(I2C_PORT_NUM, &temperature, &humidity);
                ESP_LOGI(TAG, "aht20 - Temp: %.1f, Hum: %.1f", temperature, humidity);
                aht20_temp = temperature;
                aht20_hum = humidity;

                sprintf(message, "%.1f", temperature);
                mqttPublish("inside/aht20/temp", message);
                sprintf(message, "%.1f", humidity);
                mqttPublish("inside/aht20/hum", message);

                dispLastRun = 0;
                xSemaphoreGive(i2c_semaphore);

                vTaskDelay(AHT20_DELAY / portTICK_PERIOD_MS);
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
                mqttPublish("inside/bmp280/temp", message);
                sprintf(message, "%.1f", pressure);
                mqttPublish("inside/bmp280/press", message);

                dispLastRun = 0;
                xSemaphoreGive(i2c_semaphore);

                vTaskDelay(BMP280_DELAY / portTICK_PERIOD_MS);
            }
        }
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

/*static void taskCheckAIR(void *pvParameter)
{
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
            uint32_t result_avg = 0;
            uint32_t results_count = 0;
            int voltage;

            char message[64];
            esp_err_t ret;

            ret = adc_continuous_read(adc_handle, adc_results, ADC_BUFFER_LEN, &results_count, 0);
            if (ret == ESP_OK)
            {
                for (int i = 0; i < results_count; i += SOC_ADC_DIGI_RESULT_BYTES)
                {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&adc_results[i];
                    result_avg = ((p)->type1.data);
                }
                //result_avg = (result_avg / 10);


                ///////////////////// TODO: CHECK What method is giving correct results!
                //ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, result_avg, &voltage));
                //ESP_LOGI(TAG, "ADC Read: %d", voltage);
                //ESP_LOGI(TAG, "ADC Read: %f", (float) result_avg * 3.3 / 4095);
                /////////////////////

                //sprintf(message, "%llu", result_avg);

                //voltage = ;

                sprintf(message, "%lu", result_avg);
                mqttPublish("inside/air/quality", message);
            }
            else
            {
                ESP_LOGE(TAG, "ADC Error: %d", ret);
                //break;
            }

    }
}*/

static void taskDisplay(void *pvParameter)
{
    static char char_array[(128 * 64) + 1] = {0};
    char mqtt_status[10] = "N/A";

    while (1)
    {
        if ((i2c_semaphore != NULL) && (dispLastRun == 0))
        {
            if (xSemaphoreTake(i2c_semaphore, 1000 / portTICK_PERIOD_MS) == pdTRUE)
            {
                ESP_LOGI(TAG, "DISPLAY UPDATE");
                dispLastRun = 1;

                if (mqtt_ret == ESP_OK)
                    sprintf(mqtt_status, "Connected");
                else
                    sprintf(mqtt_status, "Not Conn");

                sprintf(char_array, "MQTT: %s\nActual reading\nTemp1: %.1f C\nTemp2: %.1f C\nPress: %.1fhPa\nHum: %.1f %%\nUP: %llu", mqtt_status, bmp280_temp, aht20_temp, bmp280_press, aht20_hum, uptime);

                xTaskCreatePinnedToCore(&task_ssd1306_display_text, "displayText", 2048, (void *)char_array, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);
                //task_ssd1306_display_text((void *)char_array);

                while ( ext_i2c_busy == 1 )
                {
                    //ESP_LOGI(TAG, "DISPLAY LOCK");
                    vTaskDelay(1 / portTICK_PERIOD_MS);
                }

                xSemaphoreGive(i2c_semaphore);

                vTaskDelay((1000 / DISPLAY_HZ) / portTICK_PERIOD_MS);
            }
            //else
            //{
            //    ESP_LOGI(TAG, "Semaphore pdFALSE");
            //}
        }
        //else
        //{
        //    ESP_LOGI(TAG, "Semaphore is NULL");
        //}
    }
}

// Interrupts
void mqttInterrupt(esp_mqtt_event_handle_t event)
{
    char topic[65];
    sprintf(topic, "%.*s", event->topic_len, event->topic);
    char data[65];
    sprintf(data, "%.*s", event->data_len, event->data);

    ESP_LOGI(TAG, "%s", topic);
    
    /*if(strcmp(topic, TOPIC_HEATER_TEMP) == 0)
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
    }*/
}