#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"

#include <source_location>

#define TRY(func)                                                                                                                          \
    do {                                                                                                                                   \
        if (auto ret_ = (func); ret_ != ESP_OK) {                                                                                          \
            ESP_LOGE("ERROR", "%s:(%s):Line %d failed: %s", __FILE__, __PRETTY_FUNCTION__, __LINE__, esp_err_to_name(ret_));               \
            return ret_;                                                                                                                   \
        }                                                                                                                                  \
    } while (0)

#define TRY_WITH_FUNC(func, err_cb)                                                                                                        \
    do {                                                                                                                                   \
        if (auto ret_ = (func); ret_ != ESP_OK) {                                                                                          \
            ESP_LOGE("ERROR", "%s:(%s):Line %d failed: %s", __FILE__, __PRETTY_FUNCTION__, __LINE__, esp_err_to_name(ret_));               \
            (err_cb);                                                                                                                      \
            return ret_;                                                                                                                   \
        }                                                                                                                                  \
    } while (0)

#define TRY_WITH_FUNC_VOID(func, err_cb)                                                                                                   \
    do {                                                                                                                                   \
        if (auto ret_ = (func); ret_ != ESP_OK) {                                                                                          \
            ESP_LOGE("ERROR", "%s:(%s):Line %d failed: %s", __FILE__, __PRETTY_FUNCTION__, __LINE__, esp_err_to_name(ret_));               \
            (err_cb);                                                                                                                      \
        }                                                                                                                                  \
    } while (0)

#define TRY_THEN_LOG(func, msg)                                                                                                            \
    do {                                                                                                                                   \
        if (auto ret_ = (func); ret_ != ESP_OK) {                                                                                          \
            ESP_LOGE(TAG, "%s: %s", msg, esp_err_to_name(ret_));                                                                           \
        }                                                                                                                                  \
    } while (0)

namespace utils {

    struct spi_bus_config_t {
        spi_host_device_t bus{};

        uint32_t flags{};
        int      max_trans_size{};

        gpio_num_t mosi_pin{GPIO_NUM_NC};
        gpio_num_t miso_pin{GPIO_NUM_NC};
        gpio_num_t sclk_pin{GPIO_NUM_NC};
    };

    [[nodiscard]] inline esp_err_t init_spi_bus(const spi_bus_config_t& config) {

        const ::spi_bus_config_t bus_config = {
            .mosi_io_num           = config.mosi_pin,
            .miso_io_num           = config.miso_pin,
            .sclk_io_num           = config.sclk_pin,
            .quadwp_io_num         = GPIO_NUM_NC,
            .quadhd_io_num         = GPIO_NUM_NC,
            .data4_io_num          = GPIO_NUM_NC,
            .data5_io_num          = GPIO_NUM_NC,
            .data6_io_num          = GPIO_NUM_NC,
            .data7_io_num          = GPIO_NUM_NC,
            .data_io_default_level = false,
            .max_transfer_sz       = config.max_trans_size,
            .flags                 = config.flags,
            .isr_cpu_id            = ESP_INTR_CPU_AFFINITY_AUTO,
            .intr_flags            = 0,
        };
        TRY(spi_bus_initialize(config.bus, &bus_config, SPI_DMA_CH_AUTO));

        return ESP_OK;
    }

    [[noreturn]] inline void fatal(std::source_location location = std::source_location::current()) {
        ESP_LOGE("FATAL", "Unrecoverable error from %s (%s): %u", location.function_name(), location.file_name(), location.line());

        // Crash and halt in debug builds, but reboot in release builds
#ifdef CONFIG_COMPILER_OPTIMIZATION_LEVEL_DEBUG
        esp_system_abort("Fatal error. Cannot recover");
#else
        ESP_LOGE("FATAL", "Rebooting system");
        esp_restart();
#endif
    }

} // namespace utils
