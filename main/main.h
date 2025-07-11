#pragma once

#define I2C_MASTER_SDA_IO_1 GPIO_NUM_5
#define I2C_MASTER_SCL_IO_1 GPIO_NUM_6
#define I2C_MASTER_PORT_1 I2C_NUM_1
#define I2C_MASTER_SDA_IO_0 GPIO_NUM_9
#define I2C_MASTER_SCL_IO_0 GPIO_NUM_8
#define I2C_MASTER_PORT_0 I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 40000
#define I2C_MASTER_TIMEOUT_MS 1000

/*
XIAO board pinout
            USB
GPIO_1  -   |   - 5V
GPIO_2  -   |   - GND
GPIO_3  -   |   - 3V3
GPIO_4  -   |   - GPIO_9
GPIO_5  -   |   - GPIO_8
GPIO_6  -   |   - GPIO_7
GPIO_43 -   |   - GPIO_44
*/
