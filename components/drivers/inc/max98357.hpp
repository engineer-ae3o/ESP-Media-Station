#pragma once

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "max98357.hpp"
#include "utils.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <span>
#include <utility>
#include <cstdint>

namespace amp {

    enum class gain_t : uint8_t {
        dB_3,  // 3dB
        dB_6,  // 6dB
        dB_9,  // 9dB
        dB_12, // 12dB
        dB_15, // 15dB
    };

    enum class mode_t : uint8_t {
        LEFT_CHANNEL,
        RIGHT_CHANNEL,
        STEREO,
    };

    struct config_t {
        bool use_right_chan{};

        gpio_num_t bclk{GPIO_NUM_NC};
        gpio_num_t data{GPIO_NUM_NC};
        gpio_num_t gain{GPIO_NUM_NC};
        gpio_num_t ws{GPIO_NUM_NC};
        gpio_num_t sd{GPIO_NUM_NC};
    };

    constexpr inline auto TAG{"MAX98357"};

    constexpr inline auto SAMPLE_RATE_HZ{96'000UZ};

    template<gain_t gain, bool use_gain_pin = true>
    class max98357a_t {
    public:
        max98357a_t() = default;

        ~max98357a_t() noexcept {
            if (m_is_initialized) {
                cleanup_resources();
            }
        }

        max98357a_t(const max98357a_t&)            = delete;
        max98357a_t& operator=(const max98357a_t&) = delete;
        max98357a_t(max98357a_t&&)                 = delete;
        max98357a_t& operator=(max98357a_t&&)      = delete;

        /**
         * @brief Initialize the MAX98357 driver.
         *
         * @param[in] config Reference to struct containing driver configuration.
         * 
         * @return ESP_OK on success, error code otherwise.
         */
        [[nodiscard]] esp_err_t init(const config_t& config) {
            if (m_is_initialized) {
                return ESP_ERR_INVALID_STATE;
            }

            m_config = config;

            // Initalize the I2S channel
            const i2s_chan_config_t chan_config = {
                .id                   = I2S_NUM_AUTO,
                .role                 = I2S_ROLE_MASTER,
                .dma_desc_num         = 12,
                .dma_frame_num        = 960,
                .auto_clear_after_cb  = false,
                .auto_clear_before_cb = false,
                .allow_pd             = true,
                .intr_priority        = 4,
            };
            TRY(i2s_new_channel(&chan_config, &m_handle, nullptr));

            // Configure the I2S channel for standard mode
            const i2s_std_config_t i2s_std_config = {
                .clk_cfg =
                    {
                        .sample_rate_hz  = SAMPLE_RATE_HZ,
                        .clk_src         = I2S_CLK_SRC_DEFAULT,
                        .ext_clk_freq_hz = 0,
                        .mclk_multiple   = I2S_MCLK_MULTIPLE_1152,
                        .bclk_div        = 0,
                    },
                .slot_cfg =
                    {
                        // Use the MAX98357 in 32 bit mono audio mode
                        .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
                        .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
                        .slot_mode      = I2S_SLOT_MODE_MONO,
                        .slot_mask      = m_config.use_right_chan ? I2S_STD_SLOT_RIGHT : I2S_STD_SLOT_LEFT,
                        .ws_width       = 0,
                        .ws_pol         = false,
                        .bit_shift      = false,
                        .left_align     = true,
                        .big_endian     = false,
                        .bit_order_lsb  = false,
                    },
                .gpio_cfg =
                    {
                        .mclk = I2S_GPIO_UNUSED,
                        .bclk = m_config.bclk,
                        .ws   = m_config.ws,
                        .dout = m_config.data,
                        .din  = I2S_GPIO_UNUSED,
                        .invert_flags =
                            {
                                .mclk_inv = false,
                                .bclk_inv = false,
                                .ws_inv   = false,
                            },
                    },
            };
            TRY(i2s_channel_init_std_mode(m_handle, &i2s_std_config));

            // The gain pin is optional. Leaving it unconnected means a gain of 9dB
            if constexpr (use_gain_pin) {
                // Configure the gain gpio pin of the MAX98357 amplifier
                const gpio_config_t gain_pin_config = {
                    .pin_bit_mask = static_cast<uint64_t>(1ULL << std::to_underlying(m_config.gain)),
                    .mode         = GPIO_MODE_OUTPUT,
                    .pull_up_en   = GPIO_PULLUP_DISABLE,
                    .pull_down_en = GPIO_PULLDOWN_DISABLE,
                    .intr_type    = GPIO_INTR_DISABLE,
                };
                TRY(gpio_config(&gain_pin_config));

                // HIGH on the gain pin with a 100k-ohm resistor gives a gain of 3dB
                // HIGH on the gain pin gives a gain of 6dB
                // Leaving the gain pin unconnected gives a gain of 9dB
                // LOW on the gain pin gives a gain of 12dB
                // LOW on the gain pin with a 100k-ohm resistor gives a gain of 15dB
                // To get a gain of either 3dB or 15dB, the resistor will have tobe put in place
                if constexpr (gain == gain_t::dB_3 || gain == gain_t::dB_6) {
                    gpio_set_level(m_config.gain, 1);
                } else if constexpr (gain == gain_t::dB_9) {
                    gpio_reset_pin(m_config.gain);
                } else if constexpr (gain == gain_t::dB_12 || gain == gain_t::dB_15) {
                    gpio_set_level(m_config.gain, 0);
                } else {
                    return ESP_ERR_INVALID_ARG;
                }
            }

            // Configure the SD pin
            const gpio_config_t sd_pin_config = {
                .pin_bit_mask = static_cast<uint64_t>(1ULL << std::to_underlying(m_config.sd)),
                .mode         = GPIO_MODE_OUTPUT,
                .pull_up_en   = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type    = GPIO_INTR_DISABLE,
            };
            TRY(gpio_config(&sd_pin_config));

            m_is_initialized = true;
            return ESP_OK;
        }

        /**
         * @brief Deinitialize the MAX98357 driver and free resources.
         *
         * @return ESP_OK on success, error code otherwise.
         */
        [[nodiscard]] esp_err_t deinit() {

            if (!m_is_initialized) {
                return ESP_ERR_INVALID_STATE;
            }

            cleanup_resources();
            m_is_initialized = false;

            return ESP_OK;
        }

        /**
         * @brief Sends the buffer containing the audio samples to the MAX98357.
         * 
         * @param[in] data   Audio samples to transmit.
         * @param timeout_ms Timeout in milisecond to block while waiting for the data to be transmitted.
         * 
         * @return ESP_OK if data sent successfully, error code otherwise.
         */
        [[nodiscard]] esp_err_t send_audio_buf(std::span<int32_t> data, uint32_t timeout_ms = portMAX_DELAY) {
            if (!m_is_initialized) {
                return ESP_ERR_INVALID_STATE;
            }

            // Power on the MAX98357
            gpio_set_level(m_config.sd, 1);
            TRY(i2s_channel_enable(m_handle));

            size_t num_of_bytes_sent{};
            auto   ret = i2s_channel_write(m_handle, data.data(), (data.size() * sizeof(int32_t)), &num_of_bytes_sent, timeout_ms);
            if (ret != ESP_OK || num_of_bytes_sent != (data.size() * sizeof(int32_t))) {
                ESP_LOGE("TAG", "Failed to fully transmit audio buffer: %s", esp_err_to_name(ret));
            }

            // Power off the MAX98357
            gpio_set_level(m_config.sd, 0);
            TRY(i2s_channel_disable(m_handle));

            return ret;
        }

    private:
        bool              m_is_initialized{};
        config_t          m_config{};
        i2s_chan_handle_t m_handle{};

        // Helpers
        void cleanup_resources() {
            if (m_handle) {
                i2s_del_channel(m_handle);
                m_handle = nullptr;
            }

            gpio_reset_pin(m_config.sd);
            gpio_reset_pin(m_config.gain);

            m_config = {};
        }
    };

} // namespace amp
