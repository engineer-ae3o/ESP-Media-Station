#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include <cstdint>

namespace config {

    // Pin definitions and SPI settings for the ILI9341
    constexpr inline gpio_num_t LCD_RST_PIN  = GPIO_NUM_8;
    constexpr inline gpio_num_t LCD_MOSI_PIN = GPIO_NUM_11;
    constexpr inline gpio_num_t LCD_CLK_PIN  = GPIO_NUM_12;
    constexpr inline gpio_num_t LCD_DC_PIN   = GPIO_NUM_18;
    constexpr inline gpio_num_t LCD_CS_PIN   = GPIO_NUM_10;

    constexpr inline spi_host_device_t LCD_SPI_BUS          = SPI2_HOST;
    constexpr inline uint32_t          LCD_SPI_CLK_SPEED_HZ = 40'000'000U;

    // Pin definitions and SPI settings for the touch screen controller on the ILI9341
    constexpr inline gpio_num_t TOUCH_MOSI_PIN = GPIO_NUM_NC;
    constexpr inline gpio_num_t TOUCH_MISO_PIN = GPIO_NUM_NC;
    constexpr inline gpio_num_t TOUCH_CLK_PIN  = GPIO_NUM_NC;
    constexpr inline gpio_num_t TOUCH_CS_PIN   = GPIO_NUM_NC;
    constexpr inline gpio_num_t TOUCH_IRQ_PIN  = GPIO_NUM_NC;

    constexpr inline spi_host_device_t TOUCH_SPI_BUS          = SPI3_HOST;
    constexpr inline uint32_t          TOUCH_SPI_CLK_SPEED_HZ = 2'000'000U;

    // Pin definitions for the ILI9341
    constexpr inline gpio_num_t MAX_BCLK = GPIO_NUM_NC;
    constexpr inline gpio_num_t MAX_DATA = GPIO_NUM_NC;
    constexpr inline gpio_num_t MAX_WS   = GPIO_NUM_NC;
    constexpr inline gpio_num_t MAX_SD   = GPIO_NUM_NC;
    constexpr inline gpio_num_t MAX_GAIN = GPIO_NUM_NC;

    // Pin definitions for the ILI9341
    constexpr inline gpio_num_t INMP_BCLK   = GPIO_NUM_NC;
    constexpr inline gpio_num_t INMP_DATA   = GPIO_NUM_NC;
    constexpr inline gpio_num_t INMP_WS     = GPIO_NUM_NC;
    constexpr inline gpio_num_t INMP_CHIPEN = GPIO_NUM_NC;
    constexpr inline gpio_num_t INMP_L_R    = GPIO_NUM_NC;

    // Pin definitions for the OV3660 camera on the FPC connector

} // namespace config
