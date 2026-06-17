#pragma once

#include "driver/i2s_std.h"
#include "driver/gpio.h"

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
        LEFT_CHANNEL  = I2S_STD_SLOT_LEFT,
        RIGHT_CHANNEL = I2S_STD_SLOT_RIGHT,
        STEREO        = I2S_STD_SLOT_BOTH,
    };

    struct config_t {
        gpio_num_t bclk{GPIO_NUM_NC};
        gpio_num_t data{GPIO_NUM_NC};
        gpio_num_t gain{GPIO_NUM_NC};
        gpio_num_t ws{GPIO_NUM_NC};
        gpio_num_t sd{GPIO_NUM_NC};
    };

    constexpr inline auto TAG{"MAX98357"};

    constexpr inline auto SAMPLE_RATE_HZ{48'000UZ};

    template<gain_t gain, mode_t mode, bool use_gain_pin = true>
    class max98357a_t {
    public:
        max98357a_t() = default;

        ~max98357a_t() noexcept {
            if (m_is_initialized) {
                (void)cleanup_resources();
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
                .dma_desc_num         = 8,
                .dma_frame_num        = 1024,
                .auto_clear_after_cb  = false,
                .auto_clear_before_cb = false,
                .allow_pd             = true,
                .intr_priority        = 5, // Moderately high
            };
            TRY(i2s_new_channel(&chan_config, &m_handle, nullptr));

            const i2s_std_config_t i2s_std_config = {
                .clk_cfg =
                    {
                        .sample_rate_hz  = SAMPLE_RATE_HZ,
                        .clk_src         = I2S_CLK_SRC_PLL_240M,
                        .ext_clk_freq_hz = 0,
                        .mclk_multiple   = I2S_MCLK_MULTIPLE_1152, // A higher MCLK multiple reduces clock jitter on BCLK and WS
                        .bclk_div        = 8,                      // Default setting. Not used in master mode
                    },
                .slot_cfg =
                    {
                        // Use the MAX98357 in 32 bit audio mode
                        .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
                        .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
                        .slot_mode      = mode == mode_t::STEREO ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO,
                        .slot_mask      = static_cast<i2s_std_slot_mask_t>(mode),
                        // Since we use a slot size of 32 bits, the WS will be high for 32 BCLK clock cycles
                        .ws_width      = std::to_underlying(I2S_SLOT_BIT_WIDTH_32BIT),
                        .ws_pol        = false, // WS should be low first, that is, data starts on the rising edge
                        .bit_shift     = true,  // Bit shift for Philips mode
                        .left_align    = false, // Left alignment is irrelevant here since slot size == data size
                        .big_endian    = true,  // The MAX98357 expects MSB first then the LSB as the last bit
                        .bit_order_lsb = false, // LSB is not first
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
            TRY_WITH_FUNC(i2s_channel_init_std_mode(m_handle, &i2s_std_config), (void)cleanup_resources());

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
                TRY_WITH_FUNC(gpio_config(&gain_pin_config), (void)cleanup_resources());

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
                    static_assert(false, "Invalid gain on the MAX98357");
                }
            }

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

            TRY(cleanup_resources());
            m_is_initialized = false;

            return ESP_OK;
        }

        /**
         * @brief Power on the MAX98357.
         *
         * @return ESP_OK on success, error code otherwise.
         */
        [[nodiscard]] esp_err_t power_on(bool power_on = true) {
            if (power_on) {
                if constexpr (mode == mode_t::LEFT_CHANNEL) {
                    // For the MAX98357, if we want to use the left channel, we don't have to connect any
                    // external resistors. Instead, we just drive the pin HIGH dircetly to enable it.
                    const gpio_config_t sd_pin_config = {
                        .pin_bit_mask = static_cast<uint64_t>(1ULL << std::to_underlying(m_config.sd)),
                        .mode         = GPIO_MODE_OUTPUT,
                        .pull_up_en   = GPIO_PULLUP_DISABLE,
                        .pull_down_en = GPIO_PULLDOWN_DISABLE,
                        .intr_type    = GPIO_INTR_DISABLE,
                    };
                    // Enable the I2S channel before powering on the MAX98357
                    TRY(i2s_channel_enable(m_handle));
                    TRY(gpio_config(&sd_pin_config));
                    gpio_set_level(m_config.sd, 1);

                } else if constexpr (mode == mode_t::RIGHT_CHANNEL || mode == mode_t::STEREO) {
                    // Whereas if we wanted to use the right channel or stereo modes, we would have to connect
                    // resistors of 100k-ohm and 200k-ohm respectively. This lets the resistor bias the pins to
                    // 1/2 * Vdd and 1/3 * Vdd respectively, which is what the MAX98357 needs to use right or mixed
                    // stereo mode. We also have to make the pins input to prevent the mESP32 from driving the pin.
                    const gpio_config_t sd_pin_config = {
                        .pin_bit_mask = static_cast<uint64_t>(1ULL << std::to_underlying(m_config.sd)),
                        .mode         = GPIO_MODE_INPUT,
                        .pull_up_en   = GPIO_PULLUP_DISABLE,
                        .pull_down_en = GPIO_PULLDOWN_DISABLE,
                        .intr_type    = GPIO_INTR_DISABLE,
                    };
                    // Enable the I2S channel before powering on the MAX98357
                    TRY(i2s_channel_enable(m_handle));
                    TRY(gpio_config(&sd_pin_config));

                } else {
                    static_assert(false, "Invalid audio mode");
                }

            } else {
                // To power down the MAX98357, we can just drive the SD pin low.
                const gpio_config_t sd_pin_config = {
                    .pin_bit_mask = static_cast<uint64_t>(1ULL << std::to_underlying(m_config.sd)),
                    .mode         = GPIO_MODE_OUTPUT,
                    .pull_up_en   = GPIO_PULLUP_DISABLE,
                    .pull_down_en = GPIO_PULLDOWN_DISABLE,
                    .intr_type    = GPIO_INTR_DISABLE,
                };
                // Power off the MAX98357 before disabling the I2S channel
                TRY(gpio_config(&sd_pin_config));
                gpio_set_level(m_config.sd, 0);
                TRY(i2s_channel_disable(m_handle));
            }
            m_is_on = power_on;

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
        [[nodiscard]] esp_err_t send_audio_buf(std::span<const uint32_t> data, uint32_t timeout_ms = portMAX_DELAY) {
            if (!m_is_initialized || !m_is_on) {
                return ESP_ERR_INVALID_STATE;
            }

            const auto bytes_to_send = data.size() * sizeof(uint32_t);
            size_t     num_of_bytes_sent{};

            TRY(i2s_channel_write(m_handle, data.data(), bytes_to_send, &num_of_bytes_sent, timeout_ms));
            if (num_of_bytes_sent != bytes_to_send) {
                ESP_LOGE(TAG, "Error transmitting audio buffer. Only %zu bytes of %zu bytes sent", num_of_bytes_sent, bytes_to_send);
                return ESP_ERR_TIMEOUT;
            }

            return ESP_OK;
        }

    private:
        bool m_is_initialized{};
        bool m_is_on{};

        config_t          m_config{};
        i2s_chan_handle_t m_handle{};

        // Helpers
        [[nodiscard]] esp_err_t cleanup_resources() {
            if constexpr (use_gain_pin) {
                TRY(gpio_reset_pin(m_config.gain));
            }

            if (m_is_on) {
                TRY(power_on(false));
            }
            TRY(gpio_reset_pin(m_config.sd));

            if (m_handle) {
                TRY(i2s_del_channel(m_handle));
                m_handle = nullptr;
            }

            m_config = {};
            return ESP_OK;
        }
    };

} // namespace amp
