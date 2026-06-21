idf_component_register(SRCS "test_ili9341.cpp"
                            "test_inmp441.cpp"
                            "test_xpt2046.cpp"
                            "test_max98357.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES unity freertos app drivers
                                esp_driver_spi esp_driver_gpio
                                esp_driver_i2s
)
