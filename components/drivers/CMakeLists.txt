idf_component_register(
                    SRCS "src/ili9341.cpp" "src/camera.cpp"
                    INCLUDE_DIRS "inc/"
                    REQUIRES esp_driver_spi esp_driver_gpio esp_driver_cam esp_driver_i2s freertos)
