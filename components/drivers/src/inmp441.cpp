#include "inmp441.hpp"
#include "utils.hpp"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"

#include <atomic>
#include <utility>

namespace audio::mic {

    // Public API
    inmp441_t::~inmp441_t() noexcept {
        if (m_is_initialized) {
            cleanup();
        }
    }

    esp_err_t inmp441_t::init(const config_t& config) {
        if (m_is_initialized) {
            return ESP_ERR_INVALID_STATE;
        }

        m_config             = config;
        m_shutdown_requested = false;

        // Create the background streaming task
        // It is created first because it handles all the cleanup
        auto ret = xTaskCreate(stream_task, "Stream task", STREAM_TASK_STACK_SIZE, this, STREAM_TASK_PRIORITY, &m_streaming_task_handle);
        if (ret != pdPASS || m_streaming_task_handle == nullptr) {
            return ESP_ERR_NO_MEM;
        }

        // Initalize the I2S channel
        const i2s_chan_config_t chan_config = {
            .id                   = I2S_NUM_AUTO,
            .role                 = I2S_ROLE_MASTER,
            .dma_desc_num         = DMA_DESCR_NUM,
            .dma_frame_num        = DMA_FRAME_NUM,
            .auto_clear_after_cb  = false, // Only used for TX
            .auto_clear_before_cb = false, // Only used for TX
            .allow_pd             = false,
            .intr_priority        = 4, // Average value
        };
        TRY_WITH_FUNC(i2s_new_channel(&chan_config, nullptr, &m_handle), cleanup());

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
                    .ws_width      = std::to_underlying(I2S_SLOT_BIT_WIDTH_32BIT),
                    .ws_pol        = false, // WS should be low first, that is, data starts on the rising edge
                    .bit_shift     = true,  // Bit shift for Philips mode
                    .left_align    = false, // Left alignment is irrelevant here since slot size == data size
                    .big_endian    = false, // We don't want big endian
                    .bit_order_lsb = false, // LSB is not first
                },
            .gpio_cfg =
                {
                    .mclk = I2S_GPIO_UNUSED,
                    .bclk = m_config.bclk_pin,
                    .ws   = m_config.ws_pin,
                    .dout = I2S_GPIO_UNUSED,
                    .din  = m_config.din_pin,
                    .invert_flags =
                        {
                            .mclk_inv = false,
                            .bclk_inv = false,
                            .ws_inv   = false,
                        },
                },
        };
        TRY_WITH_FUNC(i2s_channel_init_std_mode(m_handle, &i2s_std_config), cleanup());

        // Configure the CHIPEN and L/R pins
        const gpio_config_t io_conf = {
            .pin_bit_mask =
                static_cast<uint64_t>(1ULL << std::to_underlying(m_config.chip_en_pin) | 1ULL << std::to_underlying(m_config.l_r_pin)),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        TRY_WITH_FUNC(gpio_config(&io_conf), cleanup());

        // Set whether to use the right channel of the INMP441
        // HIGH on the l_r pin makes the INMP441 output on the right channel
        gpio_set_level(m_config.l_r_pin, m_config.use_right_chan);

        // Allocate the buffers
        m_buf1 = static_cast<int32_t*>(heap_caps_malloc(RECV_BUF_SIZE_BYTES, (MALLOC_CAP_32BIT | MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM)));
        m_buf2 = static_cast<int32_t*>(heap_caps_malloc(RECV_BUF_SIZE_BYTES, (MALLOC_CAP_32BIT | MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM)));

        if (m_buf1 == nullptr || m_buf2 == nullptr) {
            cleanup();
            return ESP_ERR_NO_MEM;
        }

        // Enable the INMP441 with the CHIPEN pin
        gpio_set_level(m_config.chip_en_pin, 1);
        TRY_WITH_FUNC(i2s_channel_enable(m_handle), cleanup());

        // The INMP441 requires around 90ms after startup to fully stabilize
        vTaskDelay(pdMS_TO_TICKS(90));

        m_is_enabled     = true;
        m_is_initialized = true;

        return ESP_OK;
    }

    esp_err_t inmp441_t::deinit() {
        if (!m_is_initialized) {
            return ESP_ERR_INVALID_STATE;
        }

        cleanup();

        return ESP_OK;
    }

    esp_err_t inmp441_t::enable(bool on) {
        if (!m_is_initialized || m_is_enabled == on || m_is_streaming) {
            return ESP_ERR_INVALID_STATE;
        }

        if (on) {
            // If we are enabling, a delay of ~45ms is mandatory to allow the INMP441 fully stabilize
            gpio_set_level(m_config.chip_en_pin, 1);
            vTaskDelay(pdMS_TO_TICKS(45));
            // Enable the I2S channel after the INMP441 is fully powered
            TRY(i2s_channel_enable(m_handle));
        } else {
            // Disable the I2S channel before we disable the INMP441
            TRY(i2s_channel_disable(m_handle));
            gpio_set_level(m_config.chip_en_pin, 0);
        }
        m_is_enabled = on;

        return ESP_OK;
    }

    esp_err_t inmp441_t::start_stream() {
        if (!m_is_initialized || !m_is_enabled || m_is_streaming) {
            return ESP_ERR_INVALID_STATE;
        }

        // Set as true and wake the streaming task
        m_is_streaming = true;
        xTaskNotifyGive(m_streaming_task_handle);

        return ESP_OK;
    }

    esp_err_t inmp441_t::stop_stream() {
        if (!m_is_initialized || !m_is_enabled || !m_is_streaming) {
            return ESP_ERR_INVALID_STATE;
        }

        m_is_streaming = false;

        return ESP_OK;
    }

    std::expected<std::span<int32_t, inmp441_t::RECV_BUF_SIZE_ELEMENTS>, esp_err_t> inmp441_t::get_filled_buffer() {
        if (!m_is_initialized) {
            return std::unexpected(ESP_ERR_INVALID_STATE);
        }

        // Check if the first buffer is filled and is not being used by the stream task
        if (m_is_buf1_filled) {
            return std::span<int32_t, inmp441_t::RECV_BUF_SIZE_ELEMENTS>{m_buf1, inmp441_t::RECV_BUF_SIZE_ELEMENTS};
        }

        // Similarly, check if the second buffer is filled and is not also being used by the stream task
        if (m_is_buf2_filled) {
            return std::span<int32_t, inmp441_t::RECV_BUF_SIZE_ELEMENTS>{m_buf2, inmp441_t::RECV_BUF_SIZE_ELEMENTS};
        }

        return std::unexpected(ESP_ERR_NOT_FOUND);
    }

    esp_err_t inmp441_t::return_buffer(const int32_t* buf) {
        if (!m_is_initialized) {
            return ESP_ERR_INVALID_STATE;
        }

        if (buf == m_buf1 && m_is_buf1_filled) {
            m_is_buf1_filled = false;
            return ESP_OK;
        }

        if (buf == m_buf2 && m_is_buf2_filled) {
            m_is_buf2_filled = false;
            return ESP_OK;
        }

        return ESP_ERR_INVALID_ARG;
    }

    // Helpers
    void inmp441_t::cleanup() {
        m_deinit_task_handle = xTaskGetCurrentTaskHandle();
        m_shutdown_requested = true;

        // Wake the streaming task since it will be sleeping
        xTaskNotifyGive(m_streaming_task_handle);

        // Block till the streaming task is done with cleanup
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        m_deinit_task_handle = nullptr;
    }

    esp_err_t inmp441_t::cleanup_resources() {

        esp_err_t ret{ESP_OK};
        esp_err_t err_ret{ESP_OK};

        if (m_is_enabled) {
            // Disable the I2S channel before we disable the INMP441
            err_ret = i2s_channel_disable(m_handle);
            if (err_ret != ESP_OK) {
                ESP_LOGE(TAG, "Error disabling the I2S channel: %s", esp_err_to_name(err_ret));
                ret = err_ret;
            }

            err_ret = gpio_set_level(m_config.chip_en_pin, 0);
            if (err_ret != ESP_OK) {
                ESP_LOGE(TAG, "Error setting the CHIPEN pin low to disable the INMP441: %s", esp_err_to_name(err_ret));
                ret = err_ret;
            } else {
                m_is_enabled = false;
            }
        }

        err_ret = gpio_reset_pin(m_config.chip_en_pin);
        if (err_ret != ESP_OK) {
            ESP_LOGE(TAG, "Error deinitializing the CHIPEN pin: %s", esp_err_to_name(err_ret));
            ret = err_ret;
        }

        err_ret = gpio_reset_pin(m_config.l_r_pin);
        if (err_ret != ESP_OK) {
            ESP_LOGE(TAG, "Error deinitializing the L/R pin: %s", esp_err_to_name(err_ret));
            ret = err_ret;
        }

        if (m_handle) {
            err_ret = i2s_del_channel(m_handle);
            if (err_ret != ESP_OK) {
                ESP_LOGE(TAG, "Error deleting the I2S channel: %s", esp_err_to_name(err_ret));
                ret = err_ret;
            }
            m_handle = nullptr;
        }

        if (m_buf1) {
            heap_caps_free(m_buf1);
            m_buf1           = nullptr;
            m_is_buf1_filled = false;
        }

        if (m_buf2) {
            heap_caps_free(m_buf2);
            m_buf2           = nullptr;
            m_is_buf2_filled = false;
        }

        m_config                = {};
        m_streaming_task_handle = nullptr;

        m_is_streaming   = false;
        m_is_initialized = false;

        return ret;
    }

    void inmp441_t::stream_task(void* arg) {

        auto& driver = *static_cast<inmp441_t*>(arg);
        auto  state  = state_t::SLEEPING;

        while (!driver.m_shutdown_requested) {
            STATE_LUT[std::to_underlying(state)](driver, state);
        }

        auto ret = driver.cleanup_resources();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to cleanup all resources: %s", esp_err_to_name(ret));
        }

        // Send a notification to the task that requested the deinitialization
        // so it can safely return since all cleanup has been done properly
        TaskHandle_t deinit_task_handle = driver.m_deinit_task_handle;
        if (deinit_task_handle != nullptr) {
            xTaskNotifyGive(deinit_task_handle);
        }

        vTaskDelete(nullptr);
    }

    // State helpers
    void inmp441_t::state_check_buf1(inmp441_t& driver, state_t& state) {
        if (!driver.m_is_streaming) {
            state = state_t::SLEEPING;
            return;
        }

        // Check if the first buffer holds unread data. If it does, switch to and check buffer 2
        if (driver.m_is_buf1_filled) {
            state = state_t::CHECKING_BUF2;

            // Delay the streaming thread whenever the buffer being used is swapped
            // to prevent the streaming task from over utilizing the CPU
            vTaskDelay(pdMS_TO_TICKS(STREAM_TASK_DELAY_MS));
            return;
        }

        // If buffer 1 holds data that has been already been read, start writing new data to it
        state = state_t::WRITING_BUF1;
    }

    void inmp441_t::state_check_buf2(inmp441_t& driver, state_t& state) {
        if (!driver.m_is_streaming) {
            state = state_t::SLEEPING;
            return;
        }

        // Check if the second buffer holds unread data. If it does, switch to and check buffer 1
        if (driver.m_is_buf2_filled) {
            state = state_t::CHECKING_BUF1;

            // Delay the streaming thread whenever the buffer being used is swapped
            // to prevent the streaming task from over utilizing the CPU
            vTaskDelay(pdMS_TO_TICKS(STREAM_TASK_DELAY_MS));
            return;
        }

        // If buffer 2 holds data that has been already been read, start writing new data to it
        state = state_t::WRITING_BUF2;
    }

    void inmp441_t::state_writing_buf1(inmp441_t& driver, state_t& state) {
        if (!driver.m_is_streaming) {
            state = state_t::SLEEPING;
            return;
        }

        [&] {
            esp_err_t ret{};
            for (uint8_t i{}; i < MAX_RETRIES_ON_ERROR; i++) {
                size_t num_of_bytes_read{};
                ret = i2s_channel_read(driver.m_handle, driver.m_buf1, RECV_BUF_SIZE_BYTES, &num_of_bytes_read, STREAM_TASK_TIMEOUT_MS);
                if (ret == ESP_OK && num_of_bytes_read == RECV_BUF_SIZE_BYTES) {
                    driver.m_is_buf1_filled = true;
                    return;
                }
                ESP_LOGE(TAG, "Failed to read data from the I2S channel into buffer 1 on %u iteration: %s", i, esp_err_to_name(ret));
            }

            if (driver.m_config.error_cb != nullptr) {
                driver.m_config.error_cb(ret);
            }
        }();

        // Switch state to checking buffer 2 regardless of whether we failed to write data to buffer 1 or we succeded
        state = state_t::CHECKING_BUF2;
    }

    void inmp441_t::state_writing_buf2(inmp441_t& driver, state_t& state) {
        if (!driver.m_is_streaming) {
            state = state_t::SLEEPING;
            return;
        }

        [&] {
            esp_err_t ret{};
            for (uint8_t i{}; i < MAX_RETRIES_ON_ERROR; i++) {
                size_t num_of_bytes_read{};
                ret = i2s_channel_read(driver.m_handle, driver.m_buf2, RECV_BUF_SIZE_BYTES, &num_of_bytes_read, STREAM_TASK_TIMEOUT_MS);
                if (ret == ESP_OK && num_of_bytes_read == RECV_BUF_SIZE_BYTES) {
                    driver.m_is_buf2_filled = true;
                    return;
                }
                ESP_LOGE(TAG, "Failed to read data from the I2S channel into buffer 2 on %u iteration: %s", i, esp_err_to_name(ret));
            }

            if (driver.m_config.error_cb != nullptr) {
                driver.m_config.error_cb(ret);
            }
        }();

        // Switch state to checking buffer 1 regardless of whether we failed to write data to buffer 1 or we succeded
        state = state_t::CHECKING_BUF1;
    }

    void inmp441_t::state_sleeping(inmp441_t& driver, state_t& state) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (driver.m_is_streaming) {
            state = state_t::CHECKING_BUF1;
        }
    }

} // namespace audio::mic
