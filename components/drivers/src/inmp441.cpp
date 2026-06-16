#include "inmp441.hpp"
#include "utils.hpp"

#include "esp_heap_caps.h"
#include "esp_log.h"

#include <utility>

namespace mic {

    inmp441_t::~inmp441_t() noexcept {
        if (m_is_initialized) {
            (void)cleanup_resources();
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
        TRY(gpio_config(&io_conf));

        if (m_config.enable_during_init) {
            // Enable the INMP441 with the CHIPEN pin
            gpio_set_level(m_config.chip_en, true);
        }

        // Set whether to use the right channel of the INMP441
        // HIGH on the l_r pin makes the INMP441 output on the right channel
        gpio_set_level(m_config.l_r, m_config.use_right_chan);

        // Initalize the I2S channel
        const i2s_chan_config_t chan_config = {
            .id                   = I2S_NUM_AUTO,
            .role                 = I2S_ROLE_MASTER,
            .dma_desc_num         = 12,
            .dma_frame_num        = 960,
            .auto_clear_after_cb  = false, // Only used for TX
            .auto_clear_before_cb = false, // Only used for TX
            .allow_pd             = true,
            .intr_priority        = 4,
        };
        TRY_WITH_FUNC(i2s_new_channel(&chan_config, nullptr, &m_handle), (void)cleanup_resources());

        // Configure the I2S channel for standard mode
        const i2s_std_config_t i2s_std_config = {
            .clk_cfg =
                {
                    .sample_rate_hz  = SAMPLE_RATE_HZ,
                    .clk_src         = I2S_CLK_SRC_PLL_240M, // 240MHz divides cleanly to 48kHz
                    .ext_clk_freq_hz = 0,
                    .mclk_multiple   = I2S_MCLK_MULTIPLE_1152, // A higher MCLK multiple reduces clock jitter on BCLK and WS
                    .bclk_div        = 8,                      // Default setting. Not used in master mode
                },
            .slot_cfg =
                {
                    // Use the INMP441 in 32 bit audio mode. In reality, the data we receive is
                    // 24 bits, but this is to avoid alignment and similar requirements of ESP32-S3
                    // when using 24 bit audio. To get the proper audio, right shift all the elements
                    // in the buffer by 8 places if required to have all 24 bits in the lower bit positions
                    // We can get away with this because the INMP441 sends its 24 bit data in a 32 bit slot
                    // (64 bit frame since a frame includes both left and right samples), MSB first.
                    .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
                    .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
                    .slot_mode      = I2S_SLOT_MODE_MONO,
                    .slot_mask      = m_config.use_right_chan ? I2S_STD_SLOT_RIGHT : I2S_STD_SLOT_LEFT,
                    // Since we use a slot size of 32 bits, the WS will be high for 32 BCLK clock cycles
                    .ws_width      = static_cast<uint32_t>(I2S_SLOT_BIT_WIDTH_32BIT),
                    .ws_pol        = false, // WS should be low first, that is, data starts on the rising edge
                    .bit_shift     = false, // No need for bit shifts
                    .left_align    = false,
                    .big_endian    = true,  // The INMP441 expects MSB first till the LSB as the last bit
                    .bit_order_lsb = false, // LSB is not first
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
        TRY_WITH_FUNC(i2s_channel_init_std_mode(m_handle, &i2s_std_config), (void)cleanup_resources());
        TRY_WITH_FUNC(i2s_channel_enable(m_handle), (void)cleanup_resources());

        // Allocate the buffers
        m_buf1 = static_cast<int32_t*>(
            heap_caps_malloc(m_config.sample_buf_size_bytes, (MALLOC_CAP_32BIT | MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM)));
        m_buf2 = static_cast<int32_t*>(
            heap_caps_malloc(m_config.sample_buf_size_bytes, (MALLOC_CAP_32BIT | MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM)));

        if (m_buf1 == nullptr || m_buf2 == nullptr) {
            (void)cleanup_resources();
            return ESP_ERR_NO_MEM;
        }

        m_is_initialized = true;
        m_is_enabled     = m_config.enable_during_init;

        return ESP_OK;
    }

    esp_err_t inmp441_t::deinit() {
        if (!m_is_initialized) {
            return ESP_ERR_INVALID_STATE;
        }

        TRY(cleanup_resources());
        m_is_initialized = false;

        return ESP_OK;
    }

    esp_err_t inmp441_t::enable(bool enable) {
        if (!m_is_initialized || m_is_enabled == enable) {
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
    esp_err_t inmp441_t::cleanup_resources() {
        // Disable the INMP441 before cleaning any resources
        gpio_set_level(m_config.chip_en, 0);
        m_is_enabled = false;

        gpio_reset_pin(m_config.chip_en);
        gpio_reset_pin(m_config.l_r);
        m_config = {};

        if (m_handle) {
            TRY(i2s_channel_disable(m_handle));
            TRY(i2s_del_channel(m_handle));
            m_handle       = nullptr;
            m_is_streaming = false;
        }

        if (m_buf1) {
            heap_caps_free(m_buf1);
            m_buf1 = nullptr;
        }

        if (m_buf2) {
            heap_caps_free(m_buf2);
            m_buf2 = nullptr;
        }

        return ESP_OK;
    }

} // namespace mic
