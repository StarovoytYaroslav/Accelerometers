#include "accelerometers.h"
#include "main.h"

void i2c_master_init(int sda_pin, int scl_pin, uint32_t freq_hz)
{
    Wire.begin(sda_pin, scl_pin, 100000);
    // Wire1.setClock(freq_hz);
    Serial.println("I2C initialized");
}

void i2c_scan()
{
    Serial.println("Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("Found device at 0x");
            Serial.println(addr, HEX);
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at 0x");
            Serial.println(addr, HEX);
        }
    }
}

// void BNO055_init(i2c_port_t i2c_num, int sda_io_num, int scl_io_num)
// {
//     BNO055_port = i2c_num;
//     i2c_master_init(BNO055_port, sda_io_num, scl_io_num);

//     uint8_t id;
//     bno055_read_bytes(BNO055_port, 0x00, &id, 1); // Read CHIP_ID
//     printf("Chip ID: 0x%02X\n", id);              // Should print 0xA0

//     bno055_write_byte(BNO055_port, 0x3D, 0x00); // Set to CONFIGMODE
//     vTaskDelay(pdMS_TO_TICKS(25));

//     bno055_write_byte(BNO055_port, 0x3B, 0x00); // UNIT_SEL - m/s², degrees, Celsius
//     bno055_write_byte(BNO055_port, 0x07, 0x00); // PAGE_ID = 0

//     bno055_write_byte(BNO055_port, 0x3D, 0x0C); // Set to NDOF mode
//     vTaskDelay(pdMS_TO_TICKS(25));

//     printf("BNO055 initialized in NDOF mode.\n");
// }

// void BNO055_read(void *args)
// {
//     while (1)
//     {
//         printf("BNO055:\n");
//         read_accel(BNO055_port);
//         vTaskDelay(1);
//         read_orientation(BNO055_port);
//         vTaskDelay(1);
//         read_gyroscope(BNO055_port);
//         vTaskDelay(100);
//     }
// }

// esp_err_t bno055_write_byte(i2c_port_t i2c_num, uint8_t reg, uint8_t data)
// {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (BNO055_address << 1) | I2C_MASTER_WRITE, true);
//     i2c_master_write_byte(cmd, reg, true);
//     i2c_master_write_byte(cmd, data, true);
//     i2c_master_stop(cmd);
//     esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

// esp_err_t bno055_read_bytes(i2c_port_t i2c_num, uint8_t reg, uint8_t *buf, size_t len)
// {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (BNO055_address << 1) | I2C_MASTER_WRITE, true);
//     i2c_master_write_byte(cmd, reg, true);
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (BNO055_address << 1) | I2C_MASTER_READ, true);
//     i2c_master_read(cmd, buf, len, I2C_MASTER_LAST_NACK);
//     i2c_master_stop(cmd);
//     esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

// void read_accel(i2c_port_t i2c_num)
// {
//     uint8_t buffer[6];
//     bno055_read_bytes(i2c_num, 0x08, buffer, 6); // ACC_DATA_X_LSB

//     int16_t ax = (int16_t)((buffer[1] << 8) | buffer[0]);
//     int16_t ay = (int16_t)((buffer[3] << 8) | buffer[2]);
//     int16_t az = (int16_t)((buffer[5] << 8) | buffer[4]);

//     // Each LSB = 1 m/s² / 100 (default)
//     printf("Accel: X=%.2f Y=%.2f Z=%.2f m/s²\n", ax / 100.0, ay / 100.0, az / 100.0);
// }

// void read_orientation(i2c_port_t i2c_num)
// {
//     uint8_t buffer[6];
//     bno055_read_bytes(i2c_num, 0x1a, buffer, 6); // ACC_DATA_X_LSB

//     int16_t ax = (int16_t)((buffer[1] << 8) | buffer[0]);
//     int16_t ay = (int16_t)((buffer[3] << 8) | buffer[2]);
//     int16_t az = (int16_t)((buffer[5] << 8) | buffer[4]);
//     // Each LSB = 1 deg / 16 (default)
//     printf("Orientation: X=%.2f Y=%.2f Z=%.2f °\n", ax / 16.0, ay / 16.0, az / 16.0);
// }

// void read_gyroscope(i2c_port_t i2c_num)
// {
//     uint8_t buffer[6];
//     bno055_read_bytes(i2c_num, 0x14, buffer, 6); // ACC_DATA_X_LSB

//     int16_t ax = (int16_t)((buffer[1] << 8) | buffer[0]);
//     int16_t ay = (int16_t)((buffer[3] << 8) | buffer[2]);
//     int16_t az = (int16_t)((buffer[5] << 8) | buffer[4]);
//     // Each LSB = 1 deg / 16 (default)
//     printf("Gyroscope: X=%.2f Y=%.2f Z=%.2f °/s\n", ax / 16.0, ay / 16.0, az / 16.0);
// }

// void BNO080_init(i2c_port_t i2c_num, int sda_io_num, int scl_io_num)
// {
// }

// void setup()
// {
//     // Serial.begin(115200);
//     Wire1.begin(I2C_MASTER_SDA_IO_1, I2C_MASTER_SCL_IO_1, 100000);

//     if (!myIMU.begin(BNO080_address, Wire1))
//     {
//         printf("BNO080 not detected.");
//         while (1)
//             vTaskDelay(pdMS_TO_TICKS(1000));
//     }

//     myIMU.enableAccelerometer(50);  // accelerometer data every 50 ms
//     myIMU.enableGyro(50);           // gyroscope data every 50 ms
//     myIMU.enableMagnetometer(50);   // magnetometer data every 50 ms
//     myIMU.enableRotationVector(50); // orientation (quaternion)
// }

// void loop()
// {
//     if (myIMU.dataAvailable())
//     {
//         // Read acceleration in m/s^2
//         float ax = myIMU.getAccelX();
//         float ay = myIMU.getAccelY();
//         float az = myIMU.getAccelZ();

//         printf("Accel: X=%.2f Y=%.2f Z=%.2f m/s^2\n", ax, ay, az);

//         // Read gyro in rad/s
//         float gx = myIMU.getGyroX();
//         float gy = myIMU.getGyroY();
//         float gz = myIMU.getGyroZ();

//         printf("Gyro: X=%.2f Y=%.2f Z=%.2f rad/s\n", gx, gy, gz);

//         // Read magnetometer in uT (microTesla)
//         float mx = myIMU.getMagX();
//         float my = myIMU.getMagY();
//         float mz = myIMU.getMagZ();

//         printf("Mag: X=%.2f Y=%.2f Z=%.2f uT\n", mx, my, mz);

//         // Read rotation quaternion
//         float qI = myIMU.getQuatI();
//         float qJ = myIMU.getQuatJ();
//         float qK = myIMU.getQuatK();
//         float qReal = myIMU.getQuatReal();

//         printf("Quat: I=%.2f J=%.2f K=%.2f R=%.2f\n", qI, qJ, qK, qReal);
//     }
// }