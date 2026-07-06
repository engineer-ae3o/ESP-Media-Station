#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"

#include "hal/gpio_types.h"
#include "utils.hpp"
#include "coord_compute.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <utility>
#include <optional>

namespace touch {

    struct config_t {
        // SPI configuration
        spi_host_device_t spi_host{};

        // Blame the awkward types on ESP-IDF's SPI driver
        uint32_t clock_freq_hz{};
        int      queue_length{};

        // GPIO pins
        gpio_num_t cs_pin{GPIO_NUM_NC};
        gpio_num_t irq_pin{GPIO_NUM_NC};

        // Pixel length of the display
        uint16_t screen_pixel_len_x{};
        uint16_t screen_pixel_len_y{};

        // Conversion task details
        size_t task_priority{8};
        size_t task_stack_size{2048};
    };

    template<bool init_gpio_isr_service = true, int flags = ESP_INTR_FLAG_LEVEL1>
    class xpt2046_t {
    public:
        constexpr static uint8_t MAX_DATA_WRITE_BYTES = 3;

        xpt2046_t() = default;

        ~xpt2046_t() noexcept {
            if (m_is_initialized) {
                cleanup();
            }
        }

        xpt2046_t(const xpt2046_t&)            = delete;
        xpt2046_t& operator=(const xpt2046_t&) = delete;
        xpt2046_t(xpt2046_t&&)                 = delete;
        xpt2046_t& operator=(xpt2046_t&&)      = delete;

        /**
         * @brief Initialize the XPT2046 driver.
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

            // Create the task first since it handles cleanup for everything else
            auto ret =
                xTaskCreate(conv_task, "Conversion Task", m_config.task_stack_size, this, m_config.task_priority, &m_conv_task_handle);
            if (ret != pdPASS || m_conv_task_handle == nullptr) {
                return ESP_ERR_NO_MEM;
            }

            // Configure the XPT2046 as an SPI device
            const spi_device_interface_config_t device_config = {
                .command_bits     = 0,
                .address_bits     = 0,
                .dummy_bits       = 0,
                .mode             = 0, // The XPT2046 accepts a CPOL-CPHA of 0-0
                .clock_source     = SPI_CLK_SRC_APB,
                .duty_cycle_pos   = 128, // A duty cycle on the positive clock of 50%/50%
                .cs_ena_pretrans  = 0,
                .cs_ena_posttrans = 0,
                .clock_speed_hz   = static_cast<int>(m_config.clock_freq_hz),
                .input_delay_ns   = 0,
                .sample_point     = SPI_SAMPLING_POINT_PHASE_0,
                .spics_io_num     = m_config.cs_pin,
                .flags            = 0,
                .queue_size       = m_config.queue_length,
                .pre_cb           = nullptr,
                .post_cb          = nullptr,
            };
            TRY(spi_bus_add_device(m_config.spi_host, &device_config, &m_device_handle));

            // Send the control byte to the XPT2046
            // Start bit is 1, A2, A1 and A0 bits are all 0 (used for channel select), mode bit is 0 for 12 bit ADC resolution,
            // SER/DFR bit is 0 for differential mode, PD1 and PD0 bits are both 0 for auto power down between conversions
            alignas(4) constexpr uint8_t control_byte = (1U << START_BIT_POS);

            spi_transaction_t trans = {
                .flags            = SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL,
                .cmd              = 0,
                .addr             = 0,
                .length           = sizeof(control_byte) * 8, // Length in bits
                .rxlength         = 0,
                .override_freq_hz = 0,
                .user             = nullptr,
                .tx_buffer        = &control_byte,
                .rx_buffer        = nullptr,
            };
            TRY_WITH_FUNC(spi_device_transmit(m_device_handle, &trans), cleanup());

            // Configure the IRQ pin
            const gpio_config_t irq_config = {
                .pin_bit_mask = static_cast<uint64_t>(1ULL << std::to_underlying(m_config.irq_pin)),
                .mode         = GPIO_MODE_INPUT,
                .pull_up_en   = GPIO_PULLUP_ENABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type    = GPIO_INTR_NEGEDGE,
            };
            TRY_WITH_FUNC(gpio_config(&irq_config), cleanup());

            if constexpr (init_gpio_isr_service) {
                TRY_WITH_FUNC(gpio_install_isr_service(flags), cleanup());
            }

            // Add the ISR for the IRQ pin
            TRY_WITH_FUNC(gpio_isr_handler_add(m_config.irq_pin, irq_handler, this), cleanup());

            // Create the event queue used to pass the coordinates of presses
            m_event_queue = xQueueCreate(m_config.queue_length, sizeof(coord_t));
            if (!m_event_queue) {
                cleanup();
                return ESP_ERR_NO_MEM;
            }

            m_is_initialized = true;
            ESP_LOGI(TAG, "Initialization complete");

            return ESP_OK;
        }

        /**
         * @brief Deinitialize the XPT2046 driver and free resources.
         *
         * @return ESP_OK on success, error code otherwise.
         */
        [[nodiscard]] esp_err_t deinit() {
            if (!m_is_initialized) {
                return ESP_ERR_INVALID_STATE;
            }

            cleanup();
            return ESP_OK;
        }

        /**
         * @brief Gets the queue into which events are pushed into upon a touch detection.
         *
         * @return The event queue. Pretty straightforward.
         */
        [[nodiscard]] QueueHandle_t get_event_queue() const {
            if (!m_is_initialized) {
                return nullptr;
            }
            return m_event_queue;
        }

        /**
         * @brief Runs the calibration routine and logs the values for all 4 corners to the monitor.
         *
         * @note Not be used during normal operation.
         */
        void run_calibration() {

            // No point in letting interrupts be active since the measurement is synchronous
            gpio_intr_disable(m_config.irq_pin);

            constexpr std::array<const char*, NUM_CORNERS_TO_GET_CALIB> corners = {
                "top left corner",
                "top right corner",
                "bottom left corner",
                "bottom right corner",
            };

            std::array<uint16_t, NUM_OF_TIMES_TO_SAMPLE_DURING_CALIB> x_samples{};
            std::array<uint16_t, NUM_OF_TIMES_TO_SAMPLE_DURING_CALIB> y_samples{};

            for (const auto& corner : corners) {

                ESP_LOGI(TAG, "Touch the %s of the screen", corner);
                vTaskDelay(pdMS_TO_TICKS(5000));

                for (uint8_t i{}; i < NUM_OF_TIMES_TO_SAMPLE_DURING_CALIB; i++) {
                    // Read ADC samples for X and Y channels
                    const auto x_sample = read_chan<channel_t::X_CHAN>();
                    const auto y_sample = read_chan<channel_t::Y_CHAN>();

                    // We can skip this sample if an error occurred since it'll get filtered out
                    if (!x_sample.has_value() || !y_sample.has_value()) {
                        continue;
                    }

                    x_samples[i] = x_sample.value();
                    y_samples[i] = y_sample.value();
                }

                // Get the trimmed mean for the set of samples
                const uint16_t x_sample = compute_trimmed_mean<NUM_OF_TIMES_TO_SAMPLE_DURING_CALIB, TRIM_COUNT_DURING_CALIB>(x_samples);
                const uint16_t y_sample = compute_trimmed_mean<NUM_OF_TIMES_TO_SAMPLE_DURING_CALIB, TRIM_COUNT_DURING_CALIB>(y_samples);

                ESP_LOGI(TAG, "XPT2046 ADC calibration data for %s is (%u, %u)", corner, x_sample, y_sample);

                vTaskDelay(pdMS_TO_TICKS(5000));
            }

            gpio_intr_enable(m_config.irq_pin);
        }

    private:
        bool     m_is_initialized{};
        config_t m_config{};

        spi_device_handle_t m_device_handle{};

        TaskHandle_t  m_conv_task_handle{};
        QueueHandle_t m_event_queue{};

        std::atomic<bool>         m_shutdown_requested;
        std::atomic<TaskHandle_t> m_deinit_task_handle;

        // Bit positions in the byte to be sent to the XPT2046 controller
        constexpr static uint8_t START_BIT_POS   = 7; // Enables the XPT2046
        constexpr static uint8_t A2_BIT_POS      = 6; // Controls channel selection
        constexpr static uint8_t A1_BIT_POS      = 5; // Controls channel selection
        constexpr static uint8_t A0_BIT_POS      = 4; // Controls channel selection
        constexpr static uint8_t MODE_BIT_POS    = 3; // Determines whether the ADC uses 12 bit or 8 bit resolution
        constexpr static uint8_t SER_DFR_BIT_POS = 2; // Selects between Single ended or Differential reference mode
        constexpr static uint8_t PD1_BIT_POS     = 1; // Power down and internal reference selection
        constexpr static uint8_t PD0_BIT_POS     = 0; // Power down and internal reference selection

        constexpr static auto* TAG = "XPT2046";

        constexpr static uint8_t DEBOUNCE_MS            = 10;
        constexpr static uint8_t NUM_OF_TIMES_TO_SAMPLE = 15;
        constexpr static uint8_t TRIM_COUNT             = 5;

        constexpr static uint8_t NUM_OF_TIMES_TO_SAMPLE_DURING_CALIB = 100;
        constexpr static uint8_t TRIM_COUNT_DURING_CALIB             = 30;
        constexpr static uint8_t NUM_CORNERS_TO_GET_CALIB            = 4;

        enum class channel_t : uint8_t {
            X_CHAN = 0b101U,
            Y_CHAN = 0b001U,
        };

        // Helpers
        void cleanup() {
            m_deinit_task_handle.store(xTaskGetCurrentTaskHandle(), std::memory_order_release);
            m_shutdown_requested.store(true, std::memory_order_release);

            // Wake the conversion task since it could be sleeping
            xTaskNotifyGive(m_conv_task_handle);

            // Block till the conversion task is done with cleanup
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            m_deinit_task_handle.store(nullptr, std::memory_order_relaxed);
        }

        void cleanup_resources() {
            if (m_device_handle) {
                spi_bus_remove_device(m_device_handle);
                m_device_handle = nullptr;
            }

            gpio_intr_disable(m_config.irq_pin);
            gpio_isr_handler_remove(m_config.irq_pin);
            gpio_reset_pin(m_config.irq_pin);
            gpio_reset_pin(m_config.cs_pin);

            if constexpr (init_gpio_isr_service) {
                // Uninstall the global gpio ISR service
                gpio_uninstall_isr_service();
            }

            if (m_event_queue) {
                vQueueDelete(m_event_queue);
                m_event_queue = nullptr;
            }

            m_config           = {};
            m_conv_task_handle = nullptr;

            m_is_initialized = false;
        }

        template<channel_t channel>
        [[nodiscard]] std::optional<uint16_t> read_chan() {

            // Construct the control byte
            constexpr uint8_t control_byte = (1U << START_BIT_POS) | (std::to_underlying(channel) << A0_BIT_POS) | (0U << MODE_BIT_POS) |
                                             (0U << SER_DFR_BIT_POS) | (0U << PD1_BIT_POS) | (0U << PD0_BIT_POS);

            // For conversion, the XPT2046 expects ops of 24 bit cycles
            // Only the control byte matters for the tx buffer
            alignas(4) const std::array<uint8_t, MAX_DATA_WRITE_BYTES> tx_buf = {control_byte, 0x00U, 0x00U};
            alignas(4) std::array<uint8_t, MAX_DATA_WRITE_BYTES>       rx_buf{};

            spi_transaction_t trans = {
                .flags            = SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL,
                .cmd              = 0,
                .addr             = 0,
                .length           = MAX_DATA_WRITE_BYTES * 8,
                .rxlength         = MAX_DATA_WRITE_BYTES * 8,
                .override_freq_hz = 0,
                .user             = nullptr,
                .tx_buffer        = tx_buf.data(),
                .rx_buffer        = rx_buf.data(),
            };

            auto ret = spi_device_transmit(m_device_handle, &trans);
            if (ret != ESP_OK) {
                return std::nullopt;
            }

            // Get the lower 12 bits of the received data and put in lower bit positions
            return static_cast<uint16_t>((((rx_buf[1] << 8) | rx_buf[2]) >> 3) & 0x0FFF);
        }

        static void irq_handler(void* arg) {
            auto& driver = *static_cast<xpt2046_t<init_gpio_isr_service, flags>*>(arg);

            // Mask interrupts on the irq pin to prevent false positives during conversion
            gpio_intr_disable(driver.m_config.irq_pin);

            // Wake the conversion task
            auto higher_priority_task_woken{pdFALSE};

            if (driver.m_conv_task_handle != nullptr) {
                vTaskNotifyGiveFromISR(driver.m_conv_task_handle, &higher_priority_task_woken);
            }

            if (higher_priority_task_woken == pdTRUE) {
                portYIELD_FROM_ISR();
            } else {
                // Reenable the interrupt since the task failed to be woken and won't enable it
                gpio_intr_enable(driver.m_config.irq_pin);
            }
        }

        static void conv_task(void* arg) {
            auto& driver = *static_cast<xpt2046_t<init_gpio_isr_service, flags>*>(arg);

            while (!driver.m_shutdown_requested.load(std::memory_order_relaxed)) {
                // Sleep till a touch is detected
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

                // Check to see if the reason we were woken was because shutdown was requested
                if (driver.m_shutdown_requested.load(std::memory_order_acquire)) {
                    break;
                }

                ESP_LOGI(TAG, "Conversion task running");

                std::array<uint16_t, NUM_OF_TIMES_TO_SAMPLE> x_samples{};
                std::array<uint16_t, NUM_OF_TIMES_TO_SAMPLE> y_samples{};

                for (uint8_t i{}; i < NUM_OF_TIMES_TO_SAMPLE; i++) {
                    // Read ADC samples for X and Y channels
                    // The filter should remove these 0 values if there's an error
                    x_samples[i] = driver.template read_chan<channel_t::X_CHAN>().value_or(4095);
                    y_samples[i] = driver.template read_chan<channel_t::Y_CHAN>().value_or(4095);
                }

                const auto coord = compute_coord<NUM_OF_TIMES_TO_SAMPLE, TRIM_COUNT>(
                    x_samples, y_samples, driver.m_config.screen_pixel_len_x, driver.m_config.screen_pixel_len_y);

                ESP_LOGI(TAG, "Coordinates of press: (%u, %u)", coord.x, coord.y);

                // Push coordinate of press to the event queue
                auto ret = xQueueSend(driver.m_event_queue, &coord, 0);
                if (ret != pdPASS) {
                    ESP_LOGE(TAG, "Failed to push coordinates to event queue");
                }

                // Enable the interrupt only after we are done with conversion
                gpio_intr_enable(driver.m_config.irq_pin);
            }

            // Free used resources
            driver.cleanup_resources();

            // Send a notification to the task that requested the deinitialization
            // so it can safely return since all cleanup has been done properly
            auto deinit_task_handle = driver.m_deinit_task_handle.load(std::memory_order_acquire);
            if (deinit_task_handle != nullptr) {
                xTaskNotifyGive(deinit_task_handle);
            }

            vTaskDelete(nullptr);
        }
    };

} // namespace touch
