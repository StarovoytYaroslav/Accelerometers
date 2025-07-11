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

BNO080 myIMU;

BNO055Wrapper bno;

extern "C" void app_main(void)
{
    configure_gpio();

    initArduino();

    // Arduino-like setup()
    Serial.begin(115200);
    while (!Serial) {; /*wait for serial port to connect*/ }
    
}
