#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

#include <span>
#include <cstdint>

namespace disp {

    constexpr auto MAX_WIDTH{240U};
    constexpr auto MAX_HEIGHT{320U};

    constexpr auto TIMEOUT_MS{50U};
    constexpr auto TRANS_QUEUE_SIZE{5U};

    constexpr auto* TAG{"ILI9341"};

    struct config_t {
        // SPI configuration
        spi_host_device_t spi_host{};
        uint32_t          spi_clock_speed_hz{};

        // GPIO pins
        gpio_num_t cs{GPIO_NUM_NC};
        gpio_num_t dc{GPIO_NUM_NC};
        gpio_num_t rst{GPIO_NUM_NC};

        // Display parameters
        size_t  width{};
        size_t  height{};
        uint8_t rotation{}; // 0-3 for different orientations
    };

    class ili9341_t {
    public:
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
         * @brief Deinitialize ili9341 driver and free resources.
         *
         * @return ESP_OK on success, error code otherwise.
         */
        [[nodiscard]] esp_err_t deinit();

        /**
         * @brief Flush the given pixels to the display controller.
         *
         * @param x1, y1   Top left corner of update region.
         * @param x2, y2   Bottom right corner of update region.
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
        [[nodiscard]] esp_err_t flush(size_t x1, size_t y1, size_t x2, size_t y2, std::span<const uint16_t> data);

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

    private:
        bool                m_is_initialized{};
        config_t            m_config{};
        spi_device_handle_t m_device_handle{};

        // Helpers
        esp_err_t init_sequence();
        void      cleanup_resources();
        esp_err_t send_cmd(uint8_t cmd);
        esp_err_t send_data(std::span<const uint8_t> data);
        esp_err_t set_window(size_t x1, size_t y1, size_t x2, size_t y2);
    };

} // namespace disp
