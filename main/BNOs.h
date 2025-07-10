#pragma once

#include "Arduino.h"
#include "Wire.h"
// #include "SparkFun_BNO080_Arduino_Library.h"

#define BNO055_address 0x28
#define BNO080_address 0x4a

void i2c_master_init(int sda_pin, int scl_pin, uint32_t freq_hz = 400000);

void i2c_scan();

// esp_err_t bno055_write_byte(i2c_port_t i2c_num, uint8_t reg, uint8_t data);

// esp_err_t bno055_read_bytes(i2c_port_t i2c_num, uint8_t reg, uint8_t *buf, size_t len);

// void read_accel(i2c_port_t i2c_num);

// void read_orientation(i2c_port_t i2c_num);

// void read_gyroscope(i2c_port_t i2c_num);

// void BNO055_init(i2c_port_t i2c_num, int sda_io_num, int scl_io_num);

// void BNO055_read(void *args);

// void BNO080_init(i2c_port_t i2c_num, int sda_io_num, int scl_io_num);

// void setup();

// void loop();