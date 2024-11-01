/*
I2C controll instructions

Author: https://github.com/SebastianL-NT
*/

// Includes
#include "i2c.h"

// Varialbes
int ext_i2c_busy = 0; // It is extern flag that will allow to control some of tasks that use i2c. I needed it to control semaphores in display's task

// Public functions
esp_err_t i2c_init(i2c_port_t i2c_num) {
    // Creating config for i2c
    i2c_config_t esp_i2c_config;
    esp_i2c_config.mode = I2C_MODE_MASTER;
    esp_i2c_config.sda_io_num = 23;
    esp_i2c_config.sda_pullup_en = GPIO_PULLUP_DISABLE;
    esp_i2c_config.scl_io_num = 19;
    esp_i2c_config.scl_pullup_en = GPIO_PULLUP_DISABLE;
    esp_i2c_config.master.clk_speed = 400000;
    esp_i2c_config.clk_flags = 0;
    // Loading config
    err_check(i2c_param_config(i2c_num, &esp_i2c_config));
    // Installing driver
    err_check(i2c_driver_install(i2c_num, I2C_MODE_MASTER, 0, 0, 0));

    return ESP_OK;
}

void i2c_hex_to_uint8(uint8_t hex, uint8_t * binary) { // It only works for 0x00 format{
    for(int i = 7; i >= 0; i--) {
        binary[7-i] = (hex & (1<<i)) ? 1 : 0;
    }
}