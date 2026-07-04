#pragma once

#include "hal/spi_types.h"
#include "soc/gpio_num.h"

#include "ili9341.hpp"
#include "xpt2046.hpp"

#include <cstdint>

namespace config {

    // Shared SPI2 bus settings
    constexpr inline spi_host_device_t SHARED_SPI_BUS = SPI2_HOST;

    constexpr inline gpio_num_t SHARED_MOSI_PIN = GPIO_NUM_11;
    constexpr inline gpio_num_t SHARED_CLK_PIN  = GPIO_NUM_12;
    constexpr inline gpio_num_t SHARED_MISO_PIN = GPIO_NUM_13;

    // Pin definitions and SPI settings for the ILI9341
    constexpr inline gpio_num_t ILI_MOSI_PIN = SHARED_MOSI_PIN;
    constexpr inline gpio_num_t ILI_CLK_PIN  = SHARED_CLK_PIN;
    constexpr inline gpio_num_t ILI_MISO_PIN = SHARED_MISO_PIN;
    constexpr inline gpio_num_t ILI_DC_PIN   = GPIO_NUM_46;
    constexpr inline gpio_num_t ILI_LED_PIN  = GPIO_NUM_14;
    constexpr inline gpio_num_t ILI_CS_PIN   = GPIO_NUM_10;
    constexpr inline gpio_num_t ILI_RST_PIN  = GPIO_NUM_9;

    constexpr inline spi_host_device_t ILI_SPI_BUS          = SHARED_SPI_BUS;
    constexpr inline uint32_t          ILI_SPI_CLK_SPEED_HZ = 40'000'000U;

    // Pin definitions and SPI settings for the touch screen controller on the ILI9341
    constexpr inline gpio_num_t XPT_MOSI_PIN = SHARED_MOSI_PIN;
    constexpr inline gpio_num_t XPT_MISO_PIN = SHARED_MISO_PIN;
    constexpr inline gpio_num_t XPT_CLK_PIN  = SHARED_CLK_PIN;
    constexpr inline gpio_num_t XPT_CS_PIN   = GPIO_NUM_2;
    constexpr inline gpio_num_t XPT_IRQ_PIN  = GPIO_NUM_1;

    constexpr inline spi_host_device_t XPT_SPI_BUS          = SHARED_SPI_BUS;
    constexpr inline uint32_t          XPT_SPI_CLK_SPEED_HZ = 2'000'000U;

    // Pin definitions for the MAX98357
    constexpr inline gpio_num_t MAX_DOUT_PIN = GPIO_NUM_18;
    constexpr inline gpio_num_t MAX_GAIN_PIN = GPIO_NUM_17;
    constexpr inline gpio_num_t MAX_SD_PIN   = GPIO_NUM_16;
    constexpr inline gpio_num_t MAX_BCLK_PIN = GPIO_NUM_8;
    constexpr inline gpio_num_t MAX_WS_PIN   = GPIO_NUM_3;

    // Pin definitions for the INMP441
    constexpr inline gpio_num_t INMP_L_R_PIN    = GPIO_NUM_15;
    constexpr inline gpio_num_t INMP_WS_PIN     = GPIO_NUM_7;
    constexpr inline gpio_num_t INMP_DIN_PIN    = GPIO_NUM_5;
    constexpr inline gpio_num_t INMP_BCLK_PIN   = GPIO_NUM_6;
    constexpr inline gpio_num_t INMP_CHIPEN_PIN = GPIO_NUM_4;

    // Pin definitions for the buttons
    constexpr inline gpio_num_t RECORD_BUTTON = GPIO_NUM_NC;
    constexpr inline gpio_num_t BUTTON_1      = GPIO_NUM_NC;
    constexpr inline gpio_num_t BUTTON_2      = GPIO_NUM_NC;
    constexpr inline gpio_num_t BUTTON_3      = GPIO_NUM_NC;

} // namespace config
