#include "accelerometers.h"
#include "main.h"
#include "Adafruit_BNO055.h"

bool AccelWrapper::isConfigured = false;
TwoWire *AccelWrapper::theWire = nullptr;

AccelWrapper::AccelWrapper(TwoWire *wire, int sda_io_num, int scl_io_num, uint32_t freq_hz)
{
    if (!isConfigured)
    {
        theWire = wire;
        theWire->begin(sda_io_num, scl_io_num, freq_hz);
        Serial.println("I2C initialized");
        isConfigured = true;
    }
}

void AccelWrapper::calibration()
{
    Serial.println("Function not implemented here");
}

float *AccelWrapper::read()
{
    Serial.println("Function is not implemeted for this class");
    return nullptr;
}

GeneralPurposeWrapper::GeneralPurposeWrapper(TwoWire *wire, int sda_io_num, int scl_io_num, uint32_t freq_hz)
    : AccelWrapper(wire, sda_io_num, scl_io_num, freq_hz) {}

void GeneralPurposeWrapper::scan()
{
    Serial.println("Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        theWire->beginTransmission(addr);
        uint8_t error = theWire->endTransmission();

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

BNO055Wrapper::~BNO055Wrapper()
{
    delete bno;
}

void BNO055Wrapper::scan()
{
    Serial.println("Function is not implemeted for this class");
}

BNO055Wrapper::BNO055Wrapper(TwoWire *wire, int sda_io_num, int scl_io_num, uint32_t freq_hz)
    : AccelWrapper(wire, sda_io_num, scl_io_num, freq_hz)
{
    bno = new Adafruit_BNO055(55, BNO055_address, theWire);
    Serial.println("Orientation Sensor Test");
    Serial.println("");
    if (!bno->begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }
    delay(1000);
    /* Display some basic information on this sensor */
    displaySensorDetails();
    /* Optional: Display current status */
    displaySensorStatus();
    bno->setExtCrystalUse(true);
}

void BNO055Wrapper::displaySensorDetails()
{
    sensor_t sensor;
    bno->getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       ");
    Serial.println(sensor.name);
    Serial.print("Driver Ver:   ");
    Serial.println(sensor.version);
    Serial.print("Unique ID:    ");
    Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    ");
    Serial.print(sensor.max_value);
    Serial.println(" xxx");
    Serial.print("Min Value:    ");
    Serial.print(sensor.min_value);
    Serial.println(" xxx");
    Serial.print("Resolution:   ");
    Serial.print(sensor.resolution);
    Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

void BNO055Wrapper::displaySensorStatus()
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno->getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

void BNO055Wrapper::displayCalStatus()
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno->getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

void BNO055Wrapper::calibration()
{
    /* Get a new sensor event */
    sensors_event_t event;
    bno->getEvent(&event);

    /* Display the floating point data */
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);

    /* Optional: Display calibration status */
    displayCalStatus();

    /* Optional: Display sensor status (debug only) */
    // displaySensorStatus();

    /* New line for the next sample */
    Serial.println("");

    /* Wait the specified delay before requesting nex data */
    delay(100);
}

float *BNO055Wrapper::read()
{
    sensors_event_t angVelocityData, accelerometerData, gravityData;
    memset(results, 0, sizeof(results));
    bno->getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno->getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno->getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    results[0] = accelerometerData.acceleration.x;
    results[1] = accelerometerData.acceleration.y;
    results[2] = accelerometerData.acceleration.z;
    results[3] = angVelocityData.gyro.x;
    results[4] = angVelocityData.gyro.y;
    results[5] = angVelocityData.gyro.z;
    results[6] = gravityData.acceleration.x;
    results[7] = gravityData.acceleration.y;
    results[8] = gravityData.acceleration.z;
    return results;
}

BNO080Wrapper::BNO080Wrapper(TwoWire *wire, int sda_io_num, int scl_io_num, uint32_t freq_hz)
    : AccelWrapper(wire, sda_io_num, scl_io_num, freq_hz)
{
    bno = new BNO080();
    reset_BNO();
    vTaskDelay(pdMS_TO_TICKS(100));
    // bno->begin(BNO080_address, *theWire, 255);
    bool initialized = false;
    for (int attempts = 0; attempts < 5 && !initialized; attempts++)
    {
        initialized = bno->begin(BNO080_address, *theWire, 255);
        if (!initialized)
        {
            Serial.println("BNO080 init failed, retrying...");
            vTaskDelay(pdMS_TO_TICKS(200)); // wait a bit before retry
        }
        Serial.println("Attempt:");
        Serial.println(attempts);
    }
    if (!initialized)
    {
        Serial.println("BNO080 failed to start after retries!");
        // Optionally: enter error state or use fallback
    }
    bno->enableAccelerometer(50);
    bno->enableGyro(50);
}

float* BNO080Wrapper::read()
{
    memset(results, 0, sizeof(results));
    if (bno->dataAvailable())
    {
        results[0] = bno->getAccelX();
        results[1] = bno->getAccelY();
        results[2] = bno->getAccelZ();
        results[3] = bno->getGyroX();
        results[4] = bno->getGyroY();
        results[5] = bno->getGyroZ();
    } else 
        return nullptr;
    return results;
}

void BNO080Wrapper::calibration(){}

void BNO080Wrapper::scan()
{
    Serial.println("Function is not implemeted for this class");
}

ADXL345Wrapper::ADXL345Wrapper(TwoWire* wire, int sda_io_num, int scl_io_num, uint32_t freq_hz)
    : AccelWrapper(wire, sda_io_num, scl_io_num, freq_hz)
{
    adxl = new Adafruit_ADXL345_Unified(12345);
    adxl->begin(ADXL345_address);
    adxl->setRange(ADXL345_RANGE_2_G);
    adxl->setDataRate(ADXL345_DATARATE_100_HZ);
}

float* ADXL345Wrapper::read()
{
    memset(results, 0, sizeof(results));
    sensors_event_t event; 
    adxl->getEvent(&event);
    results[0] = event.acceleration.x;
    results[1] = event.acceleration.y;
    results[2] = event.acceleration.z;
    return results;
}

void ADXL345Wrapper::calibration(){}

void ADXL345Wrapper::scan()
{
    Serial.println("Function is not implemeted for this class");
}

MPU6050Wrapper::MPU6050Wrapper(TwoWire* wire, int sda_io_num, int scl_io_num, uint32_t freq_hz)
    : AccelWrapper(wire, sda_io_num, scl_io_num, freq_hz)
{
    mpu6050 = new Adafruit_MPU6050();
    mpu6050->begin(MPU6050_address, theWire);
    mpu6050->setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu6050->setFilterBandwidth(MPU6050_BAND_21_HZ);
    vTaskDelay(pdTICKS_TO_MS(100));
}

float* MPU6050Wrapper::read()
{
    memset(results, 0, sizeof(results));
    sensors_event_t accelEvent, gyroEvent, tempEvent; 
    mpu6050->getEvent(&accelEvent, &gyroEvent, &tempEvent);
    results[0] = accelEvent.acceleration.x;
    results[1] = accelEvent.acceleration.y;
    results[2] = accelEvent.acceleration.z;
    results[3] = gyroEvent.gyro.x;
    results[4] = gyroEvent.gyro.y;
    results[5] = gyroEvent.gyro.z;
    return results;
}

void MPU6050Wrapper::calibration(){}

void MPU6050Wrapper::scan()
{
    Serial.println("Function is not implemeted for this class");
}
