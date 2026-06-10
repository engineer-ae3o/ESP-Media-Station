idf_component_register(
                    SRCS "src/rtp.cpp"
                    INCLUDE_DIRS "inc/"
                    REQUIRES drivers esp_driver_gpio esp_driver_spi freertos)
