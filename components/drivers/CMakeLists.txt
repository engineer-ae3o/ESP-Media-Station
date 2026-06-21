idf_component_register(SRCS "src/ili9341.cpp"
                            "src/inmp441.cpp"
                            "src/xpt2046.cpp"
                       INCLUDE_DIRS "inc/"
                       REQUIRES app freertos esp_driver_spi
                                esp_driver_i2s esp_driver_gpio
)
