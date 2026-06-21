#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

#include <span>
#include <cstdint>

namespace disp {

    struct config_t {
        // SPI configuration
        spi_host_device_t spi_host{};
        uint32_t          spi_clock_speed_hz{};

        // GPIO pins
        gpio_num_t cs_pin{GPIO_NUM_NC};
        gpio_num_t dc_pin{GPIO_NUM_NC};
        gpio_num_t rst_pin{GPIO_NUM_NC};

        // Display parameter
        uint8_t rotation{}; // 0-3 for different orientations
    };

    struct coord_t {
        uint16_t x1{}, y1{}, x2{}, y2{};
    };

    class ili9341_t {
    public:
        constexpr static auto MAX_WIDTH{240U};
        constexpr static auto MAX_HEIGHT{320U};

        constexpr static auto TIMEOUT_MS{50U};
        constexpr static auto TRANS_QUEUE_SIZE{5U};

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
        [[nodiscard]] esp_err_t init_spi_bus(const bus_config_t& config);

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
