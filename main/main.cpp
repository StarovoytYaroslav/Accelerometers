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
#include "gpios.h"
#include "accelerometers.h"

#define UART_BUF_SIZE (1024)

#define CMD_BUFFER_SIZE 128

char cmd_buffer[CMD_BUFFER_SIZE];
int cmd_index = 0;

BNO080 myIMU;

extern "C" void app_main(void)
{
    configure_gpio();

    initArduino();

    // Arduino-like setup()
    Serial.begin(115200);
    while (!Serial)
    {
        ; // wait for serial port to connect
    }
    i2c_master_init(I2C_MASTER_SDA_IO_1, I2C_MASTER_SCL_IO_1,100000);
    reset_BNO();
    i2c_scan();
    // while (true)
    // {
    //     if (Serial.available())
    //     {
    //         String cmd = Serial.readStringUntil('\n');
    //         Serial.print("Received: ");
    //         printf("Some text");
    //         Serial.println(cmd);
    //     }
    //     vTaskDelay(10);
    // }
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
}
