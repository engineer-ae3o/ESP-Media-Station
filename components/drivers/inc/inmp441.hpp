#pragma once

#include "driver/i2s_etm.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"

#include <expected>
#include <memory>
#include <span>

namespace mic {

    struct config_t {
        gpio_num_t mclk{GPIO_NUM_NC};
        gpio_num_t bclk{GPIO_NUM_NC};
        gpio_num_t data{GPIO_NUM_NC};
        gpio_num_t ws{GPIO_NUM_NC};

        size_t sample_buf_size_bytes{};
    };

    class inmp441_t {
    public:
        inmp441_t() = default;
        ~inmp441_t() noexcept;

        inmp441_t(const inmp441_t&)            = delete;
        inmp441_t& operator=(const inmp441_t&) = delete;
        inmp441_t(inmp441_t&&)                 = delete;
        inmp441_t& operator=(inmp441_t&&)      = delete;

        /**
         * @brief Initialize the INMP441 driver.
         *
         * @param[in] config Reference to struct containing driver configuration.
         * 
         * @return ESP_OK on success, error code otherwise.
         */
        [[nodiscard]] esp_err_t init(const config_t& config);

        /**
         * @brief Deinitialize INMP441 driver and free resources.
         *
         * @return ESP_OK on success, error code otherwise.
         */
        [[nodiscard]] esp_err_t deinit();

        /**
         * @brief Samples data from the INMP441, fills the buffer and stops. Not
         *        compatible with the streaming mode.
         *
         * @param[in] data Buffer to store the data.
         *
         * @return ESP_OK if data received successfully, error code otherwise.
         */
        [[nodiscard]] esp_err_t get_oneshot_sample(std::span<uint32_t> data);

        /**
         * @brief Starts filling any of the available buffers with data. Uses
         *        double buffering. When one buffer is filled, swaps to the other.
         * 
         * @return ESP_OK if data started streaming successfully successfully, error code otherwise.
         */
        [[nodiscard]] esp_err_t start_stream();

        /**
         * @brief Stops filling the buffers with data.
         * 
         * @return ESP_OK if data data stopped streaming successfully, error code otherwise.
         */
        [[nodiscard]] esp_err_t stop_stream();

        /**
         * @brief Gets any available buffer that has been filled with data. Blocks until a buffer is free.
         * 
         * @param timeout_ms Timeout in miliseconds for waiting for the buffer.
         * 
         * @return The filled data buffer if available, error code otherwise.
         */
        [[nodiscard]] std::expected<std::span<const uint32_t>, esp_err_t> get_free_buffer(uint32_t timeout_ms = portMAX_DELAY) const;

    private:
        bool     m_is_initialized{};
        config_t m_config{};

        bool m_is_streaming{};
        bool m_is_buf1_active{};

        // Buffers for storing samples
        std::unique_ptr<uint32_t> m_buf1;
        std::unique_ptr<uint32_t> m_buf2;

        // Helpers
        void cleanup_resources();
    };

} // namespace mic
