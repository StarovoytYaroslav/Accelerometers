#include <stdio.h>
#include <inttypes.h>
#include "main.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "Arduino.h"
#include "SparkFun_BNO080_Arduino_Library.h"
#include "Adafruit_BNO055.h"
#include "HardwareSerial.h"
// #include "USB.h"
// #include "BNOs.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG = "i2c_scan";

#define USER_LED GPIO_NUM_21  // Use your desired pin // XIAO

// static i2c_port_t BNO055_port;
// static i2c_port_t BNO080_port;

#define UART_PORT_NUM UART_NUM_0 // USB CDC on ESP32-S3
#define UART_BUF_SIZE (1024)

#define CMD_BUFFER_SIZE 128

char cmd_buffer[CMD_BUFFER_SIZE];
int cmd_index = 0;

void configure_gpio()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << USER_LED,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

BNO080 myIMU;

void handle_uart_command(const char *cmd)
{

    // if (strcmp(cmd, "ping") == 0)
    // {
    // uart_write_bytes(UART_PORT_NUM, "pong\n", 5);
    uart_write_bytes(UART_PORT_NUM, cmd, 5);
    // }
    // if (strcmp(cmd, "accel") == 0) {
    //     // Example: Replace with your BNO080 reading code
    //     float ax = 0.12, ay = 0.05, az = 9.81;
    //     char msg[64];
    //     snprintf(msg, sizeof(msg), "Accel: X=%.2f Y=%.2f Z=%.2f\n", ax, ay, az);
    //     uart_write_bytes(UART_PORT_NUM, msg, strlen(msg));
    // }
    // else if (strcmp(cmd, "reset") == 0) {
    //     uart_write_bytes(UART_PORT_NUM, "Resetting sensor...\n", 21);
    //     // Call your BNO080 soft reset here
    // }
    // else {
    //     uart_write_bytes(UART_PORT_NUM, "Unknown command\n", 16);
    // }
}

extern "C" void app_main(void)
{
    configure_gpio();
    // gpio_set_level(USER_LED, 1);
    // HardwareSerial Serial(0);
    initArduino();

    // Arduino-like setup()
    Serial.begin(115200);
    while (!Serial)
    {
        ; // wait for serial port to connect
    }

    while (true)
    {
        if (Serial.available())
        {
            String cmd = Serial.readStringUntil('\n');
            Serial.print("Received: ");
            Serial.println(cmd);
        }
        vTaskDelay(10);
    }
    // initArduino(); // must call before setup()

    // // setup();

    // // vTaskDelay(pdMS_TO_TICKS(100));
    // while (1)
    // {
    //     // loop();
    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }

    // i2c_master_init(I2C_MASTER_PORT_1, I2C_MASTER_SDA_IO_1, I2C_MASTER_SCL_IO_1);

    // i2c_scan(I2C_MASTER_PORT_1);

    // BNO055_init(I2C_MASTER_PORT_0, I2C_MASTER_SDA_IO_0, I2C_MASTER_SCL_IO_0);
    // xTaskCreate(BNO055_read, "BNO055_read", 2048, NULL, 1, NULL);

    // uart_init();
    // printf("Ready for UART commands...\n");

    // while (true)
    // {
    //     uint8_t byte;
    //     int len = uart_read_bytes(UART_PORT_NUM, &byte, 1, pdMS_TO_TICKS(10));

    //     if (len > 0)
    //     {
    //         if (byte == '\n' || byte == '\r')
    //         {
    //             cmd_buffer[cmd_index] = '\0'; // Null-terminate
    //             handle_uart_command(cmd_buffer);
    //             cmd_index = 0; // Reset for next command
    //         }
    //         else if (cmd_index < CMD_BUFFER_SIZE - 1)
    //         {
    //             cmd_buffer[cmd_index++] = byte;
    //         }
    //     }

    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }
}
