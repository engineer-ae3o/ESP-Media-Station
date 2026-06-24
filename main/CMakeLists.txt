idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES drivers app freertos esp_driver_i2c
)
