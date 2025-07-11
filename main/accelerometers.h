#pragma once

#include "Arduino.h"
#include "Wire.h"
#include "main.h"
#include "Adafruit_BNO055.h"
#include "SparkFun_BNO080_Arduino_Library.h"

#define BNO055_address 0x28
/*
BNO055 Connection for i2c interface
ATX - > SDA // writen on the bottom of the board
LRX - > SCL
I2C - > GND
*/
#define BNO080_address 0x4a
/*
BNO080 Connection for i2c interface
ADD - > GND
RST - > Whatever pin you like, but it needs to make pin reset, see gpios.cpp
    RESET EXAMPLE:
    Wire.begin(...) // GeneralPurposeWrapper(...)
    resetFunction(...) // reset_BNO()
    i2c_scan(...) //GeneralPurposeWrapper::scan(...)
PS1 - > GND
PS0 - > GND
*/
#define MPU6050_address 0x68 // GY-521
#define ADXL345_address 0x53
/*
ADXL345 Connection for i2c interface
CS  - > 3V3
SDO - > GND
*/

class AccelWrapper
{
protected:
    static bool isConfigured;
    TwoWire* theWire = nullptr;
public:
    AccelWrapper(TwoWire* wire = &Wire, int sda_io_num = I2C_MASTER_SDA_IO_0, int scl_io_num = I2C_MASTER_SCL_IO_0, uint32_t freq_hz = 100000);
    virtual ~AccelWrapper() = default;
    virtual void scan() = 0;
    virtual void read() = 0;
};

class GeneralPurposeWrapper : public AccelWrapper
{
public:
    GeneralPurposeWrapper(TwoWire* wire = &Wire, int sda_io_num = I2C_MASTER_SDA_IO_0, int scl_io_num = I2C_MASTER_SCL_IO_0, uint32_t freq_hz = 100000);
    ~GeneralPurposeWrapper() = default;
    void scan() override;
    void read() override;
};

class BNO055Wrapper : public AccelWrapper
{
    Adafruit_BNO055 *bno = nullptr;

public:
    BNO055Wrapper(TwoWire* wire = &Wire, int sda_io_num = I2C_MASTER_SDA_IO_0, int scl_io_num = I2C_MASTER_SCL_IO_0, uint32_t freq_hz = 100000);
    ~BNO055Wrapper();
    void read() override;
    void scan() override;
private:
    void displaySensorDetails();
    void displaySensorStatus();
    void displayCalStatus();
};

// class BNO080Wrapper : public AccelWrapper
// {
// private:
// public:
//     BNO080Wrapper(TwoWire* wire = &Wire, int sda_io_num = I2C_MASTER_SDA_IO_0, int scl_io_num = I2C_MASTER_SCL_IO_0, uint32_t freq_hz = 40000)
//     : AccelWrapper(wire, sda_io_num, scl_io_num, freq_hz) {}
//     ~BNO080Wrapper() = default;
// };

// void i2c_master_init(int sda_pin, int scl_pin, uint32_t freq_hz = 40000);
// void i2c_scan();