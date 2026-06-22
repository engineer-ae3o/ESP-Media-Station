#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_err.h"
#include "hal/ledc_types.h"

#include <span>
#include <cstdint>
#include <utility>

namespace disp {

    struct config_t {
        // SPI configuration
        spi_host_device_t spi_host{};
        uint32_t          spi_clock_speed_hz{};

        // GPIO pins
        gpio_num_t led_pin{GPIO_NUM_NC};
        gpio_num_t rst_pin{GPIO_NUM_NC};
        gpio_num_t cs_pin{GPIO_NUM_NC};
        gpio_num_t dc_pin{GPIO_NUM_NC};

        // Display parameter
        uint8_t rotation{}; // 0-3 for different orientations

        // Timer and channel for pwm control of the LED
        ledc_timer_t   led_ledc_timer{};
        ledc_channel_t led_ledc_channel{};
    };

    struct coord_t {
        uint16_t x1{}, y1{}, x2{}, y2{};
    };

    class ili9341_t {
    public:
        constexpr static auto MAX_WIDTH  = 240U;
        constexpr static auto MAX_HEIGHT = 320U;

        constexpr static auto TIMEOUT_MS       = 50U;
        constexpr static auto TRANS_QUEUE_SIZE = 5U;

        constexpr static auto LED_LEDC_TIMER_RES     = LEDC_TIMER_8_BIT;
        constexpr static auto LED_LEDC_TIMER_FREQ_HZ = 20'000U;

        constexpr static auto LEDC_RES_MAX_VAL = 1 << std::to_underlying(LED_LEDC_TIMER_RES);

        constexpr static auto* TAG{"ILI9341"};

        ili9341_t() = default;
        ~ili9341_t() noexcept;

        ili9341_t(const ili9341_t&)            = delete;
        ili9341_t& operator=(const ili9341_t&) = delete;
        ili9341_t(ili9341_t&&)                 = delete;
        ili9341_t& operator=(ili9341_t&&)      = delete;

        /**
         * @brief Initialize the ili9341 driver.
         *
         * @param[in] config Reference to struct containing driver configuration.
         * 
         * @return ESP_OK on success, error code otherwise.
         */
        [[nodiscard]] esp_err_t init(const config_t& config);

        /**
         * @brief Deinitialize the ili9341 driver and free resources.
         *
         * @return ESP_OK on success, error code otherwise.
         */
        [[nodiscard]] esp_err_t deinit();

        /**
         * @brief Flush the given pixels to the display controller.
         *
         * @param[in] coord Struct containing coordinates to write to.
         * @param[in] data RGB565 pixel buffer.
         *
         * @return ESP_OK if data transmitted successfully, error code otherwise.
         * 
         * @note The ILI9341 expects the data as big endian, so when rendering, make
         *       sure the pixels are rendered as big endian before calling flush(...)
         *       as the driver sends the data verbatim. Also, ensure the data can be
         *       accessed by the DMA controller to prevent copying of the data before
         *       the actual pixel transmission takes place. Do so by marking the buffer
         *       with `DMA_ATTR` if statically allocated.
         */
        [[nodiscard]] esp_err_t flush(const coord_t& coord, std::span<const uint16_t> data);

        /**
         * @brief Sets the screen to given RGB16 color.
         *
         * @param color         Color in RGB16 format to set the screen to.
         * @param little_endian Determines whether the pixel data stored in
         *                      color is little endian or big endian.
         * 
         * @return ESP_OK if data transmitted successfully, error code otherwise.
         */
        [[nodiscard]] esp_err_t set_screen(uint16_t color, bool little_endian = true);

        /**
         * @brief Help with the initialization of the timer to be used for the Ledc channel.
         *
         * @param timer Timer to use for the Ledc channel.
         * @param init  Whether or not to initiaize or deinitialize the Ledc timer.
         * 
         * @return ESP_OK on success, error code otherwise.
         */
        [[nodiscard]] static esp_err_t init_ledc_timer(ledc_timer_t timer, bool init = true);

        /**
         * @brief Sets the screen to given brightness level.
         *
         * @param level Level to set the display's brightness.
         *
         * @return ESP_OK if data transmitted successfully, error code otherwise.
         */
        [[nodiscard]] esp_err_t set_brightness(uint8_t level = 255) const;

    private:
        bool                m_is_initialized{};
        config_t            m_config{};
        spi_device_handle_t m_device_handle{};

        // Helpers
        esp_err_t init_sequence();
        void      cleanup_resources();
        esp_err_t send_cmd(uint8_t cmd);
        esp_err_t set_window(const coord_t& coord);
        esp_err_t send_data(std::span<const uint8_t> data);
    };

} // namespace disp
