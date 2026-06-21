#pragma once

#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <span>
#include <array>
#include <atomic>
#include <utility>
#include <expected>

namespace mic {

    struct config_t {
        bool use_right_chan{};
        void (*error_cb)(esp_err_t error){};

        gpio_num_t chip_en{GPIO_NUM_NC};
        gpio_num_t bclk{GPIO_NUM_NC};
        gpio_num_t data{GPIO_NUM_NC};
        gpio_num_t l_r{GPIO_NUM_NC};
        gpio_num_t ws{GPIO_NUM_NC};
    };

    class inmp441_t {
    public:
        // Tag for identification in ESP_LOGx macros
        constexpr static auto* TAG{"INMP441"};

        // Sampling rate of the I2S channel
        constexpr static size_t SAMPLE_RATE_HZ = 48'000;

        // Hardware limit on the DMA buffer size
        constexpr static size_t MAX_DMA_BUF_SIZE = 4092;

        // Number of DMA descriptors being used
        constexpr static size_t DMA_DESCR_NUM = 2;

        // Frame size is (32 bits * 1 slot / 8) bytes
        // We use 1 slot since the INMP441 is a mono mic
        constexpr static size_t FRAME_SIZE = 1 * std::to_underlying(I2S_SLOT_BIT_WIDTH_32BIT) / 8;

        // Number of frames inside the DMA buffer
        constexpr static size_t DMA_FRAME_NUM = MAX_DMA_BUF_SIZE / FRAME_SIZE;

        // Size of our receiving buffer: Gotten from the number of DMA
        // frames, the DMA descriptor number and size of a frame
        constexpr static size_t RECV_BUF_SIZE_BYTES    = DMA_FRAME_NUM * DMA_DESCR_NUM * FRAME_SIZE;
        constexpr static size_t RECV_BUF_SIZE_ELEMENTS = RECV_BUF_SIZE_BYTES / sizeof(int32_t);

        // Used by the streaming task
        constexpr static size_t STREAM_TASK_TIMEOUT_MS = 50;
        constexpr static size_t STREAM_TASK_DELAY_MS   = 5;
        constexpr static size_t MAX_RETRIES_ON_ERROR   = 5;
        constexpr static size_t STREAM_TASK_STACK_SIZE = 2048;
        constexpr static size_t STREAM_TASK_PRIORITY   = 1;

        inmp441_t() = default;
        ~inmp441_t() noexcept;

        inmp441_t(const inmp441_t&)                   = delete;
        const inmp441_t& operator=(const inmp441_t&)  = delete;
        inmp441_t(const inmp441_t&&)                  = delete;
        const inmp441_t& operator=(const inmp441_t&&) = delete;

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
         * @param on Whether or not to enable the INMP441.
         *
         * @return ESP_OK on success, error code otherwise.
         */
        [[nodiscard]] esp_err_t enable(bool on = true);

        /**
         * @brief Starts filling any of the available buffers with data. Uses
         *        double buffering. When one buffer is filled, the task swaps to the other.
         * 
         * @return ESP_OK if data started streaming successfully, error code otherwise.
         */
        [[nodiscard]] esp_err_t start_stream();

        /**
         * @brief Stops filling the buffers with data.
         * 
         * @return ESP_OK if data stopped streaming successfully, error code otherwise.
         */
        [[nodiscard]] esp_err_t stop_stream();

        /**
         * @brief Gets any available buffer that has been filled with data. Returns
         *        immediately whether or not a buffer is filled with fresh data.
         * 
         * @return The filled data buffer if available, error code otherwise.
         * 
         * @note If the buffers aren't read on time, the streaming task blocks and polls till a buffer is read.
         */
        [[nodiscard]] std::expected<std::span<const int32_t, RECV_BUF_SIZE_ELEMENTS>, esp_err_t> get_filled_buffer();

        /**
         * @brief Returns a previously taken buffer.
         *
         * @param[in] buf The pointer to the buffer to be returned.
         * 
         * @return ESP_OK if the buffer is valid and was returned successfully, error code otherwise.
         */
        [[nodiscard]] esp_err_t return_buffer(const int32_t* buf);

    private:
        std::atomic<bool> m_is_initialized;
        std::atomic<bool> m_is_enabled;
        std::atomic<bool> m_is_streaming;

        config_t          m_config{};
        i2s_chan_handle_t m_handle{};

        // Buffers for storing samples
        int32_t* m_buf1{};
        int32_t* m_buf2{};

        std::atomic<bool> m_is_buf1_filled;
        std::atomic<bool> m_is_buf2_filled;

        // Task which handles the streaming
        TaskHandle_t      m_streaming_task_handle{};
        std::atomic<bool> m_shutdown_requested;

        // To be used as a synchronisation primitive when deinitializing the driver
        std::atomic<TaskHandle_t> m_deinit_task_handle;

        // Helpers
        void                    cleanup();
        [[nodiscard]] esp_err_t cleanup_resources();
        static void             stream_task(void* arg);

        enum class state_t : uint8_t {
            CHECKING_BUF1,
            CHECKING_BUF2,
            WRITING_BUF1,
            WRITING_BUF2,
            SLEEPING,
            COUNT,
        };

        // State helpers
        static void state_check_buf1(inmp441_t& driver, state_t& state);
        static void state_check_buf2(inmp441_t& driver, state_t& state);
        static void state_writing_buf1(inmp441_t& driver, state_t& state);
        static void state_writing_buf2(inmp441_t& driver, state_t& state);
        static void state_sleeping(inmp441_t& driver, state_t& state);

        constexpr static std::array<void (*)(inmp441_t&, state_t&), std::to_underlying(state_t::COUNT)> STATE_LUT = {{
            [std::to_underlying(state_t::CHECKING_BUF1)] = state_check_buf1,
            [std::to_underlying(state_t::CHECKING_BUF2)] = state_check_buf2,
            [std::to_underlying(state_t::WRITING_BUF1)]  = state_writing_buf1,
            [std::to_underlying(state_t::WRITING_BUF2)]  = state_writing_buf2,
            [std::to_underlying(state_t::SLEEPING)]      = state_sleeping,
        }};
    };

} // namespace mic
