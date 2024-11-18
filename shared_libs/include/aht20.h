/*
It is libary to communicate with aht20 sensor

Author: https://github.com/SebastianL-NT
*/
#ifndef aht20_H
#define aht20_H

// Includes
#include "settings.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "i2c.h"

// Public Functions
esp_err_t aht20_init(i2c_port_t i2c_num);
esp_err_t aht20_read(i2c_port_t i2c_num, float * f_temperature, float * f_humidity);

#endif