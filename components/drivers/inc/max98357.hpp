#pragma once

#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <span>
#include <cstdint>

namespace amp {

    struct config_t {
        gpio_num_t bclk{GPIO_NUM_NC};
        gpio_num_t data{GPIO_NUM_NC};
        gpio_num_t ws{GPIO_NUM_NC};
    };

    class max98357_t {
    public:
        max98357_t() = default;
        ~max98357_t() noexcept;

        max98357_t(const max98357_t&)            = delete;
        max98357_t& operator=(const max98357_t&) = delete;
        max98357_t(max98357_t&&)                 = delete;
        max98357_t& operator=(max98357_t&&)      = delete;

        /**
         * @brief Initialize the MAX98357 driver.
         *
         * @param[in] config Reference to struct containing driver configuration.
         * 
         * @return ESP_OK on success, error code otherwise.
         */
        [[nodiscard]] esp_err_t init(const config_t& config);

        /**
         * @brief Deinitialize the MAX98357 driver and free resources.
         *
         * @return ESP_OK on success, error code otherwise.
         */
        [[nodiscard]] esp_err_t deinit();

        /**
         * @brief Sends the buffer containing the audio samples to the MAX98357.
         * 
         * @return ESP_OK if data sent successfully, error code otherwise.
         */
        [[nodiscard]] esp_err_t send_audio_buf(std::span<int32_t> data);

    private:
        bool              m_is_initialized{};
        config_t          m_config{};
        i2s_chan_handle_t m_handle{};

        // Helpers
        void cleanup_resources();
    };

} // namespace amp
