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

// void app_main(void)
// {
//     // printf("Hello world!\n");

//     // /* Print chip information */
//     // esp_chip_info_t chip_info;
//     // uint32_t flash_size;
//     // esp_chip_info(&chip_info);
//     // printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
//     //        CONFIG_IDF_TARGET,
//     //        chip_info.cores,
//     //        (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
//     //        (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
//     //        (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
//     //        (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

//     // unsigned major_rev = chip_info.revision / 100;
//     // unsigned minor_rev = chip_info.revision % 100;
//     // printf("silicon revision v%d.%d, ", major_rev, minor_rev);
//     // if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
//     //     printf("Get flash size failed");
//     //     return;
//     // }

//     // printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
//     //        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

//     // printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

//     // for (int i = 10; i >= 0; i--) {
//     //     printf("Restarting in %d seconds...\n", i);
//     //     vTaskDelay(1000 / portTICK_PERIOD_MS);
//     // }
//     // printf("Restarting now.\n");
//     // fflush(stdout);
//     // esp_restart();
// }

#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SDA_IO       GPIO_NUM_5
#define I2C_MASTER_SCL_IO       GPIO_NUM_6
#define I2C_MASTER_PORT         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000
#define I2C_MASTER_TIMEOUT_MS   1000

static const char *TAG = "i2c_scan";

void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0));
}

void i2c_scan(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (int addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found device at 0x%02X", addr);
        }
    }
}

void app_main(void)
{
    i2c_master_init();
    i2c_scan();
}