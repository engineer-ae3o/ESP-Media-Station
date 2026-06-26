idf_component_register(SRC_DIRS "."
                       INCLUDE_DIRS "."
                       REQUIRES unity freertos app drivers
                                esp_driver_spi esp_driver_gpio
                                esp_driver_i2s
)
