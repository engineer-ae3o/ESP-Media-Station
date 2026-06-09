idf_component_register(
                    SRCS "src/ili9341.cpp"
                    INCLUDE_DIRS "inc/"
                    REQUIRES esp_driver_spi esp_driver_gpio freertos)
