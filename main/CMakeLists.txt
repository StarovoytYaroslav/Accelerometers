idf_component_register(SRCS "BNOs.cpp" "main.cpp"
                    PRIV_REQUIRES spi_flash
                    INCLUDE_DIRS "."
                    REQUIRES driver arduino-esp32 BNO080 BNO055)
