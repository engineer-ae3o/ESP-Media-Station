#pragma once

#include "hal/spi_types.h"
#include "soc/gpio_num.h"

#include "ili9341.hpp"

#include <cstdint>

namespace config {

    // Pin definitions and SPI settings for the ILI9341
    constexpr inline gpio_num_t ILI_RST_PIN  = GPIO_NUM_8;
    constexpr inline gpio_num_t ILI_MOSI_PIN = GPIO_NUM_11;
    constexpr inline gpio_num_t ILI_CLK_PIN  = GPIO_NUM_12;
    constexpr inline gpio_num_t ILI_DC_PIN   = GPIO_NUM_18;
    constexpr inline gpio_num_t ILI_CS_PIN   = GPIO_NUM_10;
    constexpr inline gpio_num_t ILI_LED_PIN  = GPIO_NUM_14;

    constexpr inline spi_host_device_t ILI_SPI_BUS = SPI2_HOST;

    constexpr inline uint32_t ILI_SPI_CLK_SPEED_HZ = 40'000'000U;
    constexpr inline uint32_t ILI_MAX_TRANS_SIZE   = disp::ili9341_t::MAX_WIDTH * disp::ili9341_t::MAX_HEIGHT * 2;

    // Pin definitions and SPI settings for the touch screen controller on the ILI9341
    constexpr inline gpio_num_t XPT_MOSI_PIN = GPIO_NUM_15;
    constexpr inline gpio_num_t XPT_MISO_PIN = GPIO_NUM_7;
    constexpr inline gpio_num_t XPT_CLK_PIN  = GPIO_NUM_17;
    constexpr inline gpio_num_t XPT_CS_PIN   = GPIO_NUM_16;
    constexpr inline gpio_num_t XPT_IRQ_PIN  = GPIO_NUM_6;

    constexpr inline spi_host_device_t XPT_SPI_BUS = SPI3_HOST;

    constexpr inline uint32_t XPT_SPI_CLK_SPEED_HZ = 2'000'000U;
    constexpr inline uint32_t XPT_MAX_TRANS_SIZE   = 3U;

    // Pin definitions for the MAX98357
    constexpr inline gpio_num_t MAX_BCLK = GPIO_NUM_47;
    constexpr inline gpio_num_t MAX_DATA = GPIO_NUM_21;
    constexpr inline gpio_num_t MAX_WS   = GPIO_NUM_45;
    constexpr inline gpio_num_t MAX_SD   = GPIO_NUM_19;
    constexpr inline gpio_num_t MAX_GAIN = GPIO_NUM_20;

    // Pin definitions for the INMP441
    constexpr inline gpio_num_t INMP_BCLK   = GPIO_NUM_42;
    constexpr inline gpio_num_t INMP_DATA   = GPIO_NUM_41;
    constexpr inline gpio_num_t INMP_WS     = GPIO_NUM_2;
    constexpr inline gpio_num_t INMP_CHIPEN = GPIO_NUM_NC;
    constexpr inline gpio_num_t INMP_L_R    = GPIO_NUM_1;

    // Pin definitions for all buttons
    constexpr inline gpio_num_t BUTTON_1 = GPIO_NUM_37;
    constexpr inline gpio_num_t BUTTON_2 = GPIO_NUM_38;
    constexpr inline gpio_num_t BUTTON_3 = GPIO_NUM_39;
    constexpr inline gpio_num_t BUTTON_4 = GPIO_NUM_40;

} // namespace config
