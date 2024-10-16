/*
It is libary to communicate with BMP280 sensor

written by PLSBX
*/
#ifndef BMP280_h
#define BMP280_h

// Includes
#include "settings.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "i2c.h"

// Public Functions
esp_err_t bmp280_init(i2c_port_t i2c_num);
esp_err_t bmp280_read(i2c_port_t i2c_num, float * f_temperature, float * f_humidity);

#endif