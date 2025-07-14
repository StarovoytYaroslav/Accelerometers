#pragma once

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#ifdef XIAO
    #define USER_LED GPIO_NUM_21    // XIAO 
    #define DEFAULT_VALUE 0
    #define RST_PIN GPIO_NUM_44
#elif HELTEC_WP
    #define USER_LED GPIO_NUM_18    //Wireless Paper
    #define DEFAULT_VALUE 1
#elif HELTEC_WSL
    #define USER_LED GPIO_NUM_35    //Wireless Stick Lite
    #define DEFAULT_VALUE 1
#else
        assert("You need to set board type");
#endif

void configure_gpio();

void reset_BNO();

