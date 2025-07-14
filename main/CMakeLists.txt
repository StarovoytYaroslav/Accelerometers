idf_component_register(SRCS "accelerometers.cpp" "gpios.cpp" "main.cpp"
                    PRIV_REQUIRES spi_flash
                    INCLUDE_DIRS "."
                    REQUIRES driver arduino-esp32 BNO080 BNO055 ADXL345 MPU6050)
