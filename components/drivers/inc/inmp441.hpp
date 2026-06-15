#pragma once

#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <expected>
#include <span>

namespace mic {

    struct config_t {
        gpio_num_t chip_en{GPIO_NUM_NC};
        gpio_num_t bclk{GPIO_NUM_NC};
        gpio_num_t data{GPIO_NUM_NC};
        gpio_num_t l_r{GPIO_NUM_NC};
        gpio_num_t ws{GPIO_NUM_NC};

        bool use_right_chan{};
        bool enable_during_init{};

        size_t sample_buf_size_bytes{};
    };

    constexpr inline auto TAG{"INMP441"};

    constexpr inline auto SAMPLE_RATE_HZ{48'000UZ};

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
         * @brief Enables the INMP441 through the CHIPEN gpio pin.
         * 
         * @param enable Whether or not to enable INMP441.
         *
         * @return ESP_OK on success, error code otherwise.
         */
        [[nodiscard]] esp_err_t enable(bool enable = true);

        /**
         * @brief Starts filling any of the available buffers with data. Uses
         *        double buffering. When one buffer is filled, swaps to the other.
         * 
         * @return ESP_OK if data started streaming successfully, error code otherwise.
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
        [[nodiscard]] std::expected<std::span<const int32_t>, esp_err_t> get_free_buffer(uint32_t timeout_ms = portMAX_DELAY) const;

    private:
        bool m_is_initialized{};
        bool m_is_enabled{};
        bool m_is_streaming{};

        config_t          m_config{};
        i2s_chan_handle_t m_handle{};

        // Buffers for storing samples
        int32_t* m_buf1{};
        int32_t* m_buf2{};
        bool     m_is_buf1_filled{};
        bool     m_is_buf2_filled{};

        // Helpers
        void cleanup_resources();
    };

} // namespace mic
