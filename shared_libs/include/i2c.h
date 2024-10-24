/*
I2C controll instructions

Author: https://github.com/SebastianL-NT
*/
#ifndef I2C_H
#define I2C_H
// Includes
#include "settings.h"
#include "esp_err.h"
#include "driver/i2c.h"

// Public functions
esp_err_t i2c_init(i2c_port_t i2c_num);
void i2c_hex_to_uint8(uint8_t hex, uint8_t * binary); // hex - number in hex (0x00) to convert, binary - array of 8 to store data

#endif // I2C_H