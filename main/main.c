/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "esp_log.h"

// #define I2C_MASTER_SDA_IO       GPIO_NUM_5
// #define I2C_MASTER_SCL_IO       GPIO_NUM_6
#define I2C_MASTER_SDA_IO_0       GPIO_NUM_9
#define I2C_MASTER_SCL_IO_0       GPIO_NUM_8
#define I2C_MASTER_PORT_0         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000
#define I2C_MASTER_TIMEOUT_MS   1000

static const char *TAG = "i2c_scan";

void i2c_master_init(i2c_port_t i2c_num, int sda_io_num, int scl_io_num);

void i2c_scan(i2c_port_t i2c_num);

esp_err_t bno055_write_byte(i2c_port_t i2c_num, uint8_t reg, uint8_t data);

esp_err_t bno055_read_bytes(i2c_port_t i2c_num, uint8_t reg, uint8_t *buf, size_t len);

void read_accel(i2c_port_t i2c_num);

void read_orientation(i2c_port_t i2c_num);

void read_gyroscope(i2c_port_t i2c_num);

void BNO055_init(i2c_port_t i2c_num, int sda_io_num, int scl_io_num);

void BNO055_read(i2c_port_t i2c_num);

void app_main(void)
{
    BNO055_init(I2C_MASTER_PORT_0, I2C_MASTER_SDA_IO_0, I2C_MASTER_SCL_IO_0);
    while (1)
    {
        BNO055_read(I2C_MASTER_PORT_0);
        vTaskDelay(100);
    }
    
}

void BNO055_init(i2c_port_t i2c_num, int sda_io_num, int scl_io_num)
{
    i2c_master_init(i2c_num, sda_io_num, scl_io_num);

    uint8_t id;
    bno055_read_bytes(i2c_num, 0x00, &id, 1);  // Read CHIP_ID
    printf("Chip ID: 0x%02X\n", id);  // Should print 0xA0

    bno055_write_byte(i2c_num, 0x3D, 0x00); // Set to CONFIGMODE
    vTaskDelay(pdMS_TO_TICKS(25));

    bno055_write_byte(i2c_num, 0x3B, 0x00); // UNIT_SEL - m/s², degrees, Celsius
    bno055_write_byte(i2c_num, 0x07, 0x00); // PAGE_ID = 0

    bno055_write_byte(i2c_num, 0x3D, 0x0C); // Set to NDOF mode
    vTaskDelay(pdMS_TO_TICKS(25));

    printf("BNO055 initialized in NDOF mode.\n");
}

void BNO055_read(i2c_port_t i2c_num)
{
        read_accel(i2c_num);
        vTaskDelay(1);
        read_orientation(i2c_num);
        vTaskDelay(1);
        read_gyroscope(i2c_num);
}

void i2c_master_init(i2c_port_t i2c_num, int sda_io_num, int scl_io_num)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_io_num,
        .scl_io_num = scl_io_num,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
    };
    ESP_ERROR_CHECK(i2c_param_config(i2c_num, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_num, conf.mode, 0, 0, 0));
}

void i2c_scan(i2c_port_t i2c_num)
{
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (int addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found device at 0x%02X", addr);
        }
    }
}

esp_err_t bno055_write_byte(i2c_port_t i2c_num, uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x28 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t bno055_read_bytes(i2c_port_t i2c_num, uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x28 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x28 << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num , cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void read_accel(i2c_port_t i2c_num)
{
    uint8_t buffer[6];
    bno055_read_bytes(i2c_num, 0x08, buffer, 6); // ACC_DATA_X_LSB

    int16_t ax = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t ay = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t az = (int16_t)((buffer[5] << 8) | buffer[4]);

    // Each LSB = 1 m/s² / 100 (default)
    printf("Accel: X=%.2f Y=%.2f Z=%.2f m/s²\n", ax / 100.0, ay / 100.0, az / 100.0);
}

void read_orientation(i2c_port_t i2c_num)
{
    uint8_t buffer[6];
    bno055_read_bytes(i2c_num, 0x1a, buffer, 6); // ACC_DATA_X_LSB

    int16_t ax = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t ay = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t az = (int16_t)((buffer[5] << 8) | buffer[4]);
    // Each LSB = 1 deg / 16 (default)
    printf("Orientation: X=%.2f Y=%.2f Z=%.2f °\n", ax / 16.0, ay / 16.0, az / 16.0);

}

void read_gyroscope(i2c_port_t i2c_num)
{
    uint8_t buffer[6];
    bno055_read_bytes(i2c_num, 0x14, buffer, 6); // ACC_DATA_X_LSB

    int16_t ax = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t ay = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t az = (int16_t)((buffer[5] << 8) | buffer[4]);
    // Each LSB = 1 deg / 16 (default)
    printf("Gyroscope: X=%.2f Y=%.2f Z=%.2f °/s\n", ax / 16.0, ay / 16.0, az / 16.0);

}
