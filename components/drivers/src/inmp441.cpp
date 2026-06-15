#pragma once

#include "inmp441.hpp"
#include "utils.hpp"

#include "esp_heap_caps.h"
#include "esp_log.h"

#include <utility>

namespace mic {

    inmp441_t::~inmp441_t() noexcept {
        if (m_is_initialized) {
            cleanup_resources();
        }
    }

    esp_err_t inmp441_t::init(const config_t& config) {
        if (m_is_initialized) {
            return ESP_ERR_INVALID_STATE;
        }

        m_config = config;

        // The INMP441 requires around 90ms after startup to stabilize
        vTaskDelay(pdMS_TO_TICKS(90));

        // Configure the CHIPEN and L/R pins
        const gpio_config_t io_conf = {
            .pin_bit_mask = static_cast<uint64_t>(1ULL << std::to_underlying(m_config.chip_en) | 1ULL << std::to_underlying(m_config.l_r)),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };

        auto ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure CHIPEN and L/R pins", esp_err_to_name(ret));
            return ret;
        }

        if (m_config.enable_during_init) {
            // Enable the INMP441 with the CHIPEN pin
            gpio_set_level(m_config.chip_en, true);
        }

        // Set whether to use the right channel
        // LOW is to make the INMP441 output on the left channel, HIGH is for the right channel
        gpio_set_level(m_config.l_r, m_config.use_right_chan);

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
        TRY(i2s_new_channel(&chan_config, nullptr, &m_handle));

        // Configure the I2S channel for standard mode
        const i2s_std_config_t i2s_config = {
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
                    //
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
                    .dout = I2S_GPIO_UNUSED,
                    .din  = m_config.data,
                    .invert_flags =
                        {
                            .mclk_inv = false,
                            .bclk_inv = false,
                            .ws_inv   = false,
                        },
                },
        };
        TRY(i2s_channel_init_std_mode(m_handle, &i2s_config));
        TRY(i2s_channel_enable(m_handle));

        // Allocate the buffers
        m_buf1 = static_cast<int32_t*>(
            heap_caps_malloc(m_config.sample_buf_size_bytes, (MALLOC_CAP_32BIT | MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM)));
        m_buf2 = static_cast<int32_t*>(
            heap_caps_malloc(m_config.sample_buf_size_bytes, (MALLOC_CAP_32BIT | MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM)));

        if (m_buf1 == nullptr || m_buf2 == nullptr) {
            cleanup_resources();
            return ESP_ERR_NO_MEM;
        }

        return ESP_OK;
    }

    esp_err_t inmp441_t::deinit() {
        if (!m_is_initialized) {
            return ESP_ERR_INVALID_STATE;
        }

        cleanup_resources();
        m_is_initialized = false;

        return ESP_OK;
    }

    esp_err_t inmp441_t::enable(bool enable) {
        if (!m_is_initialized) {
            return ESP_ERR_INVALID_STATE;
        }

        // HIGH on the CHIPEN pin means to enable the INMP441
        gpio_set_level(m_config.chip_en, enable);
        m_is_enabled = enable;
        if (m_is_enabled) {
            // If we are enabling, a delay of ~45ms is mandatory to allow it stabilize
            vTaskDelay(pdMS_TO_TICKS(45));
        }

        return ESP_OK;
    }

    esp_err_t inmp441_t::start_stream() {
        if (!m_is_initialized || !m_is_enabled || m_is_streaming) {
            return ESP_ERR_INVALID_STATE;
        }

        // TODO: Handle streaming mode starting logic

        m_is_streaming = true;

        return ESP_OK;
    }

    esp_err_t inmp441_t::stop_stream() {
        if (!m_is_initialized || !m_is_enabled || !m_is_streaming) {
            return ESP_ERR_INVALID_STATE;
        }

        // TODO: Handle streaming mode stopping logic

        m_is_streaming = false;

        return ESP_OK;
    }

    std::expected<std::span<const int32_t>, esp_err_t> inmp441_t::get_free_buffer(uint32_t timeout_ms) const {
        if (!m_is_initialized) {
            return std::unexpected(ESP_ERR_INVALID_STATE);
        }

        // TODO: Handle buffer returning logic

        return {};
    }

    // Helpers
    void inmp441_t::cleanup_resources() {
        // Disable the INMP441 before cleaning any resources
        gpio_set_level(m_config.chip_en, false);
        gpio_reset_pin(m_config.chip_en);
        gpio_reset_pin(m_config.l_r);

        if (m_handle) {
            i2s_channel_disable(m_handle);
            i2s_del_channel(m_handle);
            m_handle = nullptr;
        }

        if (m_buf1) {
            heap_caps_free(m_buf1);
            m_buf1 = nullptr;
        }
        if (m_buf2) {
            heap_caps_free(m_buf2);
            m_buf2 = nullptr;
        }
    }

} // namespace mic
