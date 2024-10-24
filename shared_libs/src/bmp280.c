/*
It is libary to communicate with BMP280 sensor.
It will use force mode in sensor.

Author: https://github.com/SebastianL-NT
*/
// Includes
#include "bmp280.h"

// Defines
#define BMP280_ADDRESS 0x77 // 7 bit address
#define BMP280_CMD_ID 0xD0
#define BMP280_CMD_RESET 0xE0 // To reset: Send this + value
#define BMP280_VALUE_RESET 0xB6 // Value to send after RESET CMD
#define BMP280_CMD_STATUS 0xF3
#define BMP280_CMD_CTRL_MEAS 0xF4 // To control oversampling and power mode (table 10)
// After CMD_CTRL_MEAS we have to send value with settings: 7,6,5 bit - osrs_p, 4,3,2 bit - osrs_t, 1,0 bit - power mode
// OSRS_P/T: 000 - skipped, 001 - os x1, 010 - os x2, 011 - os x4... Table 20/21/22/23 in datasheet of bmp280
#define BMP280_CMD_CONFIG 0xF5 // Config register, after this send data: Table 23 in BMP280 datasheet
#define BMP280_CMD_DATA 0xF7 // Command/register to read raw data of pressure and temperature, Table 24 and 25 in datasheet
#define BMP280_CMD_TRIM 0x88 // Register that hold calibration data - 24 bytes up to 0x9F
#define BMP280_WAIT_TIME 20 // Wait time in [ms] before reading after starting measurments (table 13 of bmp280 datasheet)



// Private functions prototypes
void bmp280_convert(float *f_temperature, float *f_pressure, uint8_t *raw_data);

// Variables to hold callibration data.
uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
uint8_t callibration_data[24];

// Public functions
esp_err_t bmp280_init(i2c_port_t i2c_num)
{
    // Variables
    uint8_t data_to_send[1];
    uint8_t data_received[6];
    //uint8_t callibration_data[24];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Check if sensor exist - Ask for ID
    err_check(i2c_master_start(cmd));
    i2c_master_write_byte(cmd, BMP280_ADDRESS << 1, true); // Send device's address
    data_to_send[0] = BMP280_CMD_ID;
    i2c_master_write(cmd, data_to_send, 1, true); // Send init data
    i2c_master_stop(cmd);
    err_check(i2c_master_cmd_begin(i2c_num, cmd, 100));
    i2c_cmd_link_delete(cmd);

    vTaskDelay(10 / portTICK_PERIOD_MS);
    cmd = i2c_cmd_link_create();
    err_check(i2c_master_start(cmd));
    err_check(i2c_master_write_byte(cmd, (BMP280_ADDRESS << 1) + 0x01, true)); // Send device's address
    err_check(i2c_master_read(cmd, data_received, 1, I2C_MASTER_NACK));
    err_check(i2c_master_stop(cmd));
    err_check(i2c_master_cmd_begin(i2c_num, cmd, 1000));
    i2c_cmd_link_delete(cmd);

    if (data_received[0] != 0x58)
    { // Simple check if sensor exists
        return ESP_FAIL;
    }

    // Setup configuration
    vTaskDelay(10 / portTICK_PERIOD_MS);
    cmd = i2c_cmd_link_create();
    err_check(i2c_master_start(cmd));
    i2c_master_write_byte(cmd, BMP280_ADDRESS << 1, true); // Send device's address
    data_to_send[0] = BMP280_CMD_CONFIG;
    data_to_send[1] = 0x0 << 3;                                    // Standby, 3 bits
    data_to_send[1] = 0x0 << 2;                                    // IIR Filter, 3 bits
    data_to_send[1] = 0x0;                                         // Enable 3 wire-spi
    i2c_master_write(cmd, data_to_send, 2, true); // Send init data
    i2c_master_stop(cmd);
    err_check(i2c_master_cmd_begin(i2c_num, cmd, 100));
    i2c_cmd_link_delete(cmd);

    // Get callibration data
    vTaskDelay(10 / portTICK_PERIOD_MS);
    cmd = i2c_cmd_link_create();
    err_check(i2c_master_start(cmd));
    i2c_master_write_byte(cmd, BMP280_ADDRESS << 1, true); // Send device's address
    i2c_master_write_byte(cmd, BMP280_CMD_TRIM, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDRESS << 1) + 0x1, true);
    i2c_master_read(cmd, callibration_data, 23, I2C_MASTER_ACK);
    i2c_master_read(cmd, &callibration_data[23], 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    err_check(i2c_master_cmd_begin(i2c_num, cmd, 100));
    i2c_cmd_link_delete(cmd);

    // Parse data
    dig_T1 = ((callibration_data[1]<<8) | callibration_data[0]);
    dig_T2 = ((int16_t) (callibration_data[3]<<8) | callibration_data[2]);
    dig_T3 = ((int16_t)(callibration_data[5]<<8) | callibration_data[4]);
    dig_P1 = ((callibration_data[7]<<8) | callibration_data[6]);
    dig_P2 = ((int16_t)(callibration_data[9]<<8) | callibration_data[8]);
    dig_P3 = ((int16_t)(callibration_data[11]<<8) | callibration_data[10]);
    dig_P4 = ((int16_t)(callibration_data[13]<<8) | callibration_data[12]);
    dig_P5 = ((int16_t)(callibration_data[15]<<8) | callibration_data[14]);
    dig_P6 = ((int16_t)(callibration_data[17]<<8) | callibration_data[16]);
    dig_P7 = ((int16_t)(callibration_data[19]<<8) | callibration_data[18]);
    dig_P8 = ((int16_t)(callibration_data[21]<<8) | callibration_data[20]);
    dig_P9 = ((int16_t)(callibration_data[23]<<8) | callibration_data[22]);

    return ESP_OK;
}

esp_err_t bmp280_read(i2c_port_t i2c_num, float *f_temperature, float *f_humidity)
{
    // Variables
    uint8_t data_to_send[1];
    uint8_t received_data[6];
    //uint8_t converted_hex[8];
    uint8_t try_num = 0;
    uint8_t osrs_p = 0x1;   // pressure oversampling x1
    uint8_t osrs_t = 0x1;   // temperature oversampling x1
    uint8_t pwr_mode = 0x1; // Force power mode
    //uint32_t humidity = 0;
    //uint32_t temperature = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Start measurment
    err_check(i2c_master_start(cmd));
    i2c_master_write_byte(cmd, BMP280_ADDRESS << 1, true); // Send device's address
    data_to_send[0] = BMP280_CMD_CTRL_MEAS;
    data_to_send[1] = osrs_t << 3;
    data_to_send[1] = (data_to_send[1] | osrs_p) << 2;
    data_to_send[1] = data_to_send[1] | pwr_mode;
    i2c_master_write(cmd, data_to_send, 2, true); // Send init data
    i2c_master_stop(cmd);
    err_check(i2c_master_cmd_begin(i2c_num, cmd, 100));
    i2c_cmd_link_delete(cmd);

    // Wait time before reading
    vTaskDelay(BMP280_WAIT_TIME / portTICK_PERIOD_MS);

    // Read state and data
    while (try_num < 5)
    { // Check for status can be repeated maximum of 5 times
        // Check status
        cmd = i2c_cmd_link_create();
        err_check(i2c_master_start(cmd));
        i2c_master_write_byte(cmd, BMP280_ADDRESS << 1, true); // Change address to read data
        data_to_send[0] = BMP280_CMD_STATUS;
        i2c_master_write(cmd, data_to_send, 1, true); // Send init data
        i2c_master_stop(cmd);
        err_check(i2c_master_cmd_begin(i2c_num, cmd, 100));
        i2c_cmd_link_delete(cmd);

        cmd = i2c_cmd_link_create();
        err_check(i2c_master_start(cmd));
        i2c_master_write_byte(cmd, (BMP280_ADDRESS << 1) + 0x1, true); // Change address to read data
        i2c_master_read(cmd, received_data, 1, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        err_check(i2c_master_cmd_begin(i2c_num, cmd, 100));
        i2c_cmd_link_delete(cmd);

        if (received_data[0] == 0x0)
        {
            // read data
            cmd = i2c_cmd_link_create();
            err_check(i2c_master_start(cmd));
            i2c_master_write_byte(cmd, (BMP280_ADDRESS << 1), true);
            data_to_send[0] = BMP280_CMD_DATA;
            i2c_master_write(cmd, data_to_send, 1, true);
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (BMP280_ADDRESS << 1) + 0x1, true);
            i2c_master_read(cmd, received_data, 5, I2C_MASTER_ACK);
            i2c_master_read(cmd, &received_data[5], 1, I2C_MASTER_NACK);
            i2c_master_stop(cmd);
            err_check(i2c_master_cmd_begin(i2c_num, cmd, 100));
            i2c_cmd_link_delete(cmd);

            //bmp280_debug(f_temperature, f_humidity, received_data);
            bmp280_convert(f_temperature, f_humidity, received_data);
            break;
        }
        else
        {
            // Check again
            try_num++;
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
    // If try_num is equal or greater than 4 -> It is fail
    if (try_num >= 4)
    {
        return ESP_FAIL;
    }

    //*f_humidity = (float) 0;
    //*f_temperature = (float) 0;

    return ESP_OK;
}

// Private functions
void bmp280_convert(float *f_temperature, float *f_pressure, uint8_t *raw_data)
{
    long long raw_temp;
    long long raw_press;
    int32_t var1, var2, t_fine;
    int32_t p = 0;

    raw_press = ((long long) (((raw_data[0]<<16) | (raw_data[1]<<8) | raw_data[0])>>4));
    raw_temp = ((long long) (((raw_data[3]<<16) | (raw_data[4]<<8) | raw_data[5])>>4));

    // Temperature
    var1 = ((((raw_temp >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((raw_temp >> 4) - ((int32_t)dig_T1)) * ((raw_temp >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;
    *f_temperature = ((float)((t_fine * 5 + 128)>>8)) / 100.0;

    // Pressure
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dig_P6);
    var2 = var2 + ((var1 * ((int32_t)dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)dig_P1)) >> 15);
    if (var1 != 0){ // avoid exception caused by division by zero
        p = (((uint32_t)(((int32_t)1048576) - raw_press) - (var2 >> 12))) * 3125;
        if (p < 0x80000000) {
            p = (p << 1) / ((uint32_t)var1);
        } else {
            p = (p / (uint32_t)var1) * 2;
        }
        var1 = (((int32_t)dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
        var2 = (((int32_t)(p >> 2)) * ((int32_t)dig_P8)) >> 13;
        p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4)); 
    }

    *f_pressure = ((float)p) / 100.0;   
}
