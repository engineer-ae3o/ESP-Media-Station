#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_log.h"

#include "utils.hpp"

#include <array>
#include <cstdint>
#include <utility>
#include <optional>
#include <expected>

namespace touch {

    struct config_t {
        // SPI configuration
        spi_host_device_t spi_host{};

        uint32_t clock_freq_hz{};
        int      queue_length{};

        // GPIO pins
        gpio_num_t cs_pin{GPIO_NUM_NC};
        gpio_num_t irq_pin{GPIO_NUM_NC};
    };

    struct coord_t {
        size_t x{}, y{};
    };

    template<bool init_gpio_isr_service = true, int flags = (ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_LEVEL4)>
    class xpt2046_t {
    public:
        constexpr static auto MAX_X_COORD = 240U;
        constexpr static auto MAX_Y_COORD = 320U;

        xpt2046_t() = default;

        ~xpt2046_t() noexcept {
            if (m_is_initialized) {
                cleanup_resources();
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
            TRY_WITH_FUNC(spi_bus_add_device(m_config.spi_host, &device_config, &m_device_handle), cleanup_resources());

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
            TRY_WITH_FUNC(spi_device_transmit(m_device_handle, &trans), cleanup_resources());

            // Configure the IRQ pin
            const gpio_config_t irq_config = {
                .pin_bit_mask = static_cast<uint64_t>(1ULL << std::to_underlying(m_config.irq_pin)),
                .mode         = GPIO_MODE_INPUT,
                .pull_up_en   = GPIO_PULLUP_ENABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type    = GPIO_INTR_NEGEDGE,
            };
            TRY_WITH_FUNC(gpio_config(&irq_config), cleanup_resources());

            if constexpr (init_gpio_isr_service) {
                TRY_WITH_FUNC(gpio_install_isr_service(flags), cleanup_resources());
            }

            // Add the ISR for the IRQ pin
            TRY_WITH_FUNC(gpio_isr_handler_add(m_config.irq_pin, irq_handler, this), cleanup_resources());

            // Create the timer for conversion and event queue used to pass the coordinates of presses
            m_conv_timer  = xTimerCreate("Conversion Timer", pdMS_TO_TICKS(TIMEOUT_MS), pdFALSE, this, conv_timer);
            m_event_queue = xQueueCreate(m_config.queue_length, sizeof(coord_t));

            if (!m_event_queue || !m_conv_timer) {
                cleanup_resources();
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

            cleanup_resources();

            return ESP_OK;
        }

        /**
         * @brief Gets the queue into which events are pushed into upon a touch detection.
         *
         * @return The event queue. Pretty straightforward.
         */
        [[nodiscard]] std::expected<QueueHandle_t, esp_err_t> get_event_queue() const {
            if (!m_is_initialized) {
                return std::unexpected(ESP_ERR_INVALID_STATE);
            }
            return m_event_queue;
        }

    private:
        bool     m_is_initialized{};
        config_t m_config{};

        spi_device_handle_t m_device_handle{};

        TimerHandle_t m_conv_timer{};
        QueueHandle_t m_event_queue{};

        // Bit positions in the byte to be sent to the XPT2046 controller
        constexpr static uint8_t START_BIT_POS   = 7;
        constexpr static uint8_t A2_BIT_POS      = 6;
        constexpr static uint8_t A1_BIT_POS      = 5;
        constexpr static uint8_t A0_BIT_POS      = 4;
        constexpr static uint8_t MODE_BIT_POS    = 3;
        constexpr static uint8_t SER_DFR_BIT_POS = 2;
        constexpr static uint8_t PD1_BIT_POS     = 1;
        constexpr static uint8_t PD0_BIT_POS     = 0;

        // Channel select
        constexpr static uint8_t X_CHAN_BIT_MASK = 0b101U << A0_BIT_POS;
        constexpr static uint8_t Y_CHAN_BIT_MASK = 0b001U << A0_BIT_POS;

        constexpr static auto* TAG        = "XPT2046";
        constexpr static auto  TIMEOUT_MS = 50U;

        enum class channel_t : uint8_t {
            X_CHAN = X_CHAN_BIT_MASK,
            Y_CHAN = Y_CHAN_BIT_MASK,
        };

        // Helpers
        void cleanup_resources() {
            if (m_device_handle) {
                spi_bus_remove_device(m_device_handle);
                m_device_handle = nullptr;
            }

            gpio_isr_handler_remove(m_config.irq_pin);
            gpio_reset_pin(m_config.irq_pin);
            gpio_reset_pin(m_config.cs_pin);

            if constexpr (init_gpio_isr_service) {
                // Uninstall the global gpio ISR service
                gpio_uninstall_isr_service();
            }

            if (m_conv_timer) {
                xTimerStop(m_conv_timer, portMAX_DELAY);
                xTimerDelete(m_conv_timer, portMAX_DELAY);
                m_conv_timer = nullptr;
            }

            if (m_event_queue) {
                vQueueDelete(m_event_queue);
                m_event_queue = nullptr;
            }

            m_config         = {};
            m_is_initialized = false;
        }

        [[nodiscard]] std::optional<uint16_t> read_chan(channel_t channel) {

            // Construct the control byte
            const uint8_t control_byte = 0x00U;

            // FOr conversion, the XPT2046 expects ops of 24 bit cycles
            // Only the control byte matters for the tx buffer
            alignas(4) const uint8_t tx_buf[3] = {control_byte, 0x00U, 0x00U};
            alignas(4) uint8_t       rx_buf[3]{};

            spi_transaction_t trans = {
                .flags            = SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL,
                .cmd              = 0,
                .addr             = 0,
                .length           = sizeof(tx_buf) * 8,
                .rxlength         = sizeof(rx_buf) * 8,
                .override_freq_hz = 0,
                .user             = nullptr,
                .tx_buffer        = tx_buf,
                .rx_buffer        = rx_buf,
            };

            auto ret = spi_device_transmit(m_device_handle, &trans);
            if ((ret != ESP_OK) || (rx_buf[0] == 0 && rx_buf[1] == 0 && rx_buf[2] == 0)) {
                return std::nullopt;
            }

            return static_cast<uint16_t>(0);
        }

        static void IRAM_ATTR irq_handler(void* arg) {
            auto* driver = static_cast<xpt2046_t*>(arg);

            // Mask interrupts on the irq pin to prevent false positives during conversion
            gpio_intr_disable(driver->m_config.irq_pin);

            // Enable the conversion timer
            auto higher_priority_task_woken{pdFALSE};
            xTimerStartFromISR(driver->m_conv_timer, &higher_priority_task_woken);

            if (higher_priority_task_woken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }

        static void conv_timer(TimerHandle_t handle) {
            auto* driver = static_cast<xpt2046_t*>(pvTimerGetTimerID(handle));

            // Read ADC samples for X and Y channels
            [[maybe_unused]] auto x = driver->read_chan(channel_t::X_CHAN);
            [[maybe_unused]] auto y = driver->read_chan(channel_t::Y_CHAN);

            // TODO: Handle conversion from raw ADC samples to useable coordinates
            coord_t coord{};

            // Enable the interrupt only after we are done with conversion
            gpio_intr_enable(driver->m_config.irq_pin);

            // Push coordinate of press to the event queue
            auto ret = xQueueSend(driver->m_event_queue, &coord, 0);
            if (ret == pdPASS) {
                portYIELD();
            }
        }
    };

} // namespace touch
