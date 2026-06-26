#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_log.h"

#include "utils.hpp"

#include <span>
#include <cstdint>
#include <utility>
#include <expected>

namespace touch {

    struct config_t {
        // SPI configuration
        spi_host_device_t spi_host{};

        uint32_t spi_clock_speed_hz{};
        size_t   queue_size{};

        // GPIO pins
        gpio_num_t cs_pin{GPIO_NUM_NC};
        gpio_num_t irq_pin{GPIO_NUM_NC};
    };

    template<bool init_gpio_isr_service = true, int flags = (ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_LEVEL4)>
    class xpt2046_t {
    public:
        constexpr static auto TIMEOUT_MS = 50U;

        constexpr static auto* TAG = "XPT2046";

        xpt2046_t() = default;

        ~xpt2046_t() noexcept {
            auto ret = cleanup_resources();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to cleanup used resources (destructor): %s", esp_err_to_name(ret));
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

            // Configure the IRQ pin
            const gpio_config_t irq_config = {
                .pin_bit_mask = static_cast<uint64_t>(1ULL << std::to_underlying(m_config.irq_pin)),
                .mode         = GPIO_MODE_INPUT,
                .pull_up_en   = GPIO_PULLUP_ENABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type    = GPIO_INTR_NEGEDGE,
            };
            TRY(gpio_config(&irq_config));

            if constexpr (init_gpio_isr_service) {
                // Install the global gpio ISR service
                TRY_WITH_FUNC(gpio_install_isr_service(flags), cleanup_resources());
            }

            // Add the ISR for the IRQ pin
            TRY_WITH_FUNC(gpio_isr_handler_add(m_config.irq_pin, irq_pin_handler, this), cleanup_resources());

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
                .clock_speed_hz   = static_cast<int>(m_config.spi_clock_speed_hz),
                .input_delay_ns   = 0,
                .sample_point     = SPI_SAMPLING_POINT_PHASE_0,
                .spics_io_num     = m_config.cs_pin,
                .flags            = 0,
                .queue_size       = m_config.queue_size,
                .pre_cb           = nullptr,
                .post_cb          = nullptr,
            };
            TRY_WITH_FUNC(spi_bus_add_device(m_config.spi_host, &device_config, &m_device_handle), cleanup_resources());

            m_event_queue = xQueueCreate(m_config.queue_size, 1);
            if (!m_event_queue) {
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
        QueueHandle_t       m_event_queue{};

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

            if (m_event_queue) {
                vQueueDelete(m_event_queue);
                m_event_queue = nullptr;
            }

            m_config         = {};
            m_is_initialized = false;
        }

        [[nodiscard]] esp_err_t send_cmd(uint8_t cmd) {
            return ESP_OK;
        }

        [[nodiscard]] esp_err_t send_data(std::span<const uint8_t> data) {
            return ESP_OK;
        }

        IRAM_ATTR static void irq_pin_handler(void* arg) {
            auto* driver = static_cast<xpt2046_t*>(arg);
        }
    };

} // namespace touch
