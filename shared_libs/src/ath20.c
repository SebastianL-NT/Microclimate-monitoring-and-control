/*
It is libary to communicate with ATH20 sensor

Written by PLSBX
*/

// Includes
#include "ath20.h"


// Defines
#define ATH20_ADDRESS 0x38 // 7 bit
#define ATH20_INIT 0xBE
#define ATH20_MEASURE 0xAC
#define ATH20_WAIT_AFTER_INIT 40 // Wait after power on [ms]
#define ATH20_WAIT_MEASURMENT 80 // Wait after sending ATH20_MEASURE [ms]

// Prototypes private functions
uint8_t ath20_is_ready(uint8_t * state);

// Public functions
esp_err_t ath20_init(i2c_port_t i2c_num) {
    // Variables
    uint8_t data_to_send[3];
    uint8_t sensor_state[1];
    uint8_t converted_hex[8];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Initialization of sensor
    err_check(i2c_master_start(cmd));
    err_check(i2c_master_write_byte(cmd, ATH20_ADDRESS<<1, true)); // Send device's address
    data_to_send[0] = ATH20_INIT; data_to_send[1] = 0x08; data_to_send[2] = 0x00; // Setting up an init commands
    err_check(i2c_master_write(cmd, data_to_send, 3, true)); // Send init data
    err_check(i2c_master_stop(cmd));
    err_check(i2c_master_cmd_begin(i2c_num, cmd, 10));
    i2c_cmd_link_delete(cmd);

    // Start first measurement
    vTaskDelay( ATH20_WAIT_AFTER_INIT / portTICK_PERIOD_MS );
    cmd = i2c_cmd_link_create();
    err_check(i2c_master_start(cmd));
    err_check(i2c_master_write_byte(cmd, ATH20_ADDRESS<<1, true)); // Send device's address
    data_to_send[0] = ATH20_MEASURE; data_to_send[1] = 0x33; data_to_send[2] = 0x00; // Setting up a measure commands
    err_check(i2c_master_write(cmd, data_to_send, 3, true)); // Send data
    err_check(i2c_master_stop(cmd));
    err_check(i2c_master_cmd_begin(i2c_num, cmd, 10));
    i2c_cmd_link_delete(cmd);

    // Check if sensor is initialized correctly
    vTaskDelay( ATH20_WAIT_AFTER_INIT / portTICK_PERIOD_MS );
    cmd = i2c_cmd_link_create();
    err_check(i2c_master_start(cmd));
    err_check(i2c_master_write_byte(cmd, (ATH20_ADDRESS<<1) + 0x01, true)); // Change address to read data
    err_check(i2c_master_read(cmd, sensor_state, 1, I2C_MASTER_ACK));
    err_check(i2c_master_stop(cmd));
    err_check(i2c_master_cmd_begin(i2c_num, cmd, 1000));
    i2c_cmd_link_delete(cmd);

    // Check if received state is correct
    i2c_hex_to_uint8(sensor_state[0], converted_hex);
    if(ath20_is_ready(converted_hex) == 0) {
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t ath20_read(i2c_port_t i2c_num, float * f_temperature, float * f_humidity) {
    // Variables
    uint8_t data_to_send[1];
    uint8_t received_data[6];
    uint8_t converted_hex[8];
    uint8_t try_num = 0;
    uint32_t humidity = 0;
    uint32_t temperature = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Start measurment
    err_check(i2c_master_start(cmd));
    err_check(i2c_master_write_byte(cmd, ATH20_ADDRESS<<1, true)); // Send device's address
    data_to_send[0] = ATH20_MEASURE; data_to_send[1] = 0x33; data_to_send[2] = 0x00; // Setting up a measure commands
    err_check(i2c_master_write(cmd, data_to_send, 3, true)); // Send data
    err_check(i2c_master_stop(cmd));
    err_check(i2c_master_cmd_begin(i2c_num, cmd, 10));
    i2c_cmd_link_delete(cmd);

    // Wait 80ms
    vTaskDelay( ATH20_WAIT_MEASURMENT / portTICK_PERIOD_MS );

    // Read state
    while (try_num < 5) { // Check for status can be repeated maximum of 5 times
        cmd = i2c_cmd_link_create();
        err_check(i2c_master_start(cmd));
        err_check(i2c_master_write_byte(cmd, (ATH20_ADDRESS<<1) + 0x01, true)); // Change address to read data
        err_check(i2c_master_read(cmd, received_data, 1, I2C_MASTER_ACK));
        //err_check(i2c_master_stop(cmd));
        err_check(i2c_master_cmd_begin(i2c_num, cmd, 100));
        i2c_cmd_link_delete(cmd);
        i2c_hex_to_uint8(received_data[0], converted_hex);
        if(ath20_is_ready(converted_hex) == 0) {
            cmd = i2c_cmd_link_create();
            err_check(i2c_master_read(cmd, received_data, 6, I2C_MASTER_ACK)); // Received data will be filled from 0
            err_check(i2c_master_stop(cmd));
            err_check(i2c_master_cmd_begin(i2c_num, cmd, 100));
            i2c_cmd_link_delete(cmd);
            break;
        } else {
            cmd = i2c_cmd_link_create();
            err_check(i2c_master_stop(cmd));
            err_check(i2c_master_cmd_begin(i2c_num, cmd, 100));
            i2c_cmd_link_delete(cmd);
            try_num++;
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }

    // If try_num is equal or greater than 4 -> It is fail
    if (try_num >= 4) { i2c_cmd_link_delete(cmd); return ESP_FAIL; }

    // Parse data, based on ATH20 demo
    // Byte 1, 2 and first half of 3rd are humidity data
    // Second half of 3rd, 4 and 5 are temperature data
    humidity = (humidity|received_data[0])<<8;
    humidity = (humidity|received_data[1])<<8;
    humidity = (humidity|received_data[2])>>4;
    
    temperature = (temperature|received_data[2])<<8;
    temperature = (temperature|received_data[3])<<8;
    temperature = (temperature|received_data[4]);
    temperature = temperature&0xFFFFF;

    *f_humidity = (float) ((double) humidity / 1048576.0) * 100.0;
    *f_temperature = (float) (((double)  temperature / 1048576.0) * 200.0) - 50.0;

    return ESP_OK;
}

// Private functions
uint8_t ath20_is_ready(uint8_t * state) {
    // return codes:
    // 0 - okey
    // 1 - busy
    // 2 - no callibration
    if(state[7] == 1) {
        return 1;
    } else if(state[3] == 0) {
        return 2;
    } else {
        return 0;
    }
}