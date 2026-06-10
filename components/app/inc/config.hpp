#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include <cstdint>

namespace config {

    constexpr inline spi_host_device_t LCD_SPI_BUS          = SPI2_HOST;
    constexpr inline uint32_t          LCD_SPI_CLK_SPEED_HZ = 40'000'000U;
    constexpr inline gpio_num_t        LCD_RST_PIN          = GPIO_NUM_8;
    constexpr inline gpio_num_t        LCD_MOSI_PIN         = GPIO_NUM_11;
    constexpr inline gpio_num_t        LCD_CLK_PIN          = GPIO_NUM_12;
    constexpr inline gpio_num_t        LCD_DC_PIN           = GPIO_NUM_18;
    constexpr inline gpio_num_t        LCD_CS_PIN           = GPIO_NUM_10;

    constexpr inline spi_host_device_t TOUCH_SPI_BUS  = SPI3_HOST;
    constexpr inline gpio_num_t        TOUCH_MOSI_PIN = GPIO_NUM_NC;
    constexpr inline gpio_num_t        TOUCH_MISO_PIN = GPIO_NUM_NC;
    constexpr inline gpio_num_t        TOUCH_CLK_PIN  = GPIO_NUM_NC;
    constexpr inline gpio_num_t        TOUCH_CS_PIN   = GPIO_NUM_NC;
    constexpr inline gpio_num_t        TOUCH_IRQ_PIN  = GPIO_NUM_NC;

} // namespace config
