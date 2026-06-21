#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "utils.hpp"

#include <span>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace touch {

    struct config_t {
        // SPI configuration
        spi_host_device_t spi_host{};
        uint32_t          spi_clock_speed_hz{};

        // GPIO pins
        gpio_num_t cs_pin{GPIO_NUM_NC};
        gpio_num_t irq_pin{GPIO_NUM_NC};
    };

    template<bool init_gpio_isr_service = true>
    class xpt2046_t {
    public:
        constexpr static auto TIMEOUT_MS{50U};
        constexpr static auto TRANS_QUEUE_SIZE{5U};

        constexpr static auto* TAG{"XPT2046"};

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

            auto ret = gpio_config(&irq_config);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to configure the irq pin: %s", esp_err_to_name(ret));
                return ret;
            }

            // Setup ISR handler
            ret = gpio_install_isr_service(0);
            if (ret != ESP_OK || ret != ESP_ERR_INVALID_STATE) {
                ESP_LOGE(TAG, "Failed to register the global gpio ISR service: %s", esp_err_to_name(ret));
                return ret;
            }

            // Configure SPI device
            const spi_device_interface_config_t dev_cfg = {
                .command_bits     = 0,
                .address_bits     = 0,
                .dummy_bits       = 0,
                .mode             = 0, // The XPT2046 accepts a CPOL-CPHA of 0-0
                .clock_source     = SPI_CLK_SRC_DEFAULT,
                .duty_cycle_pos   = 128, // Default param
                .cs_ena_pretrans  = 0,
                .cs_ena_posttrans = 0,
                .clock_speed_hz   = static_cast<int>(m_config.spi_clock_speed_hz),
                .input_delay_ns   = 0,
                .sample_point     = SPI_SAMPLING_POINT_PHASE_0,
                .spics_io_num     = m_config.cs_pin,
                .flags            = 0,
                .queue_size       = TRANS_QUEUE_SIZE,
                .pre_cb           = nullptr,
                .post_cb          = nullptr,
            };

            ret = spi_bus_add_device(m_config.spi_host, &dev_cfg, &m_device_handle);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "SPI device configure failed: %s", esp_err_to_name(ret));
                cleanup_resources();
                return ret;
            }

            // Send initialization sequence to the XPT2046
            ret = init_sequence();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to transmit the initialization sequence: %s", esp_err_to_name(ret));
                cleanup_resources();
                return ret;
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

            auto ret = cleanup_resources();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to cleanup used resources (deinit()): %s", esp_err_to_name(ret));
                return ret;
            }

            m_is_initialized = false;
            return ESP_OK;
        }

    private:
        bool                m_is_initialized{};
        config_t            m_config{};
        spi_device_handle_t m_device_handle{};

        // Helpers
        [[nodiscard]] esp_err_t init_sequence() {
            return ESP_OK;
        }

        [[nodiscard]] esp_err_t cleanup_resources() {
            return ESP_OK;
        }

        [[nodiscard]] esp_err_t send_cmd(uint8_t cmd) {
            return ESP_OK;
        }

        [[nodiscard]] esp_err_t send_data(std::span<const uint8_t> data) {
            return ESP_OK;
        }
    };

} // namespace touch
