#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_log.h"

#define TRY(func)                                                                                                                          \
    do {                                                                                                                                   \
        if (auto ret_ = (func); ret_ != ESP_OK) {                                                                                          \
            ESP_LOGE("ERR", "%s:%s:Line %d failed: %s", __FILE__, __PRETTY_FUNCTION__, __LINE__, esp_err_to_name(ret_));                   \
            return ret_;                                                                                                                   \
        }                                                                                                                                  \
    } while (0)

#define TRY_WITH_FUNC(func, err_cb)                                                                                                        \
    do {                                                                                                                                   \
        if (auto ret_ = (func); ret_ != ESP_OK) {                                                                                          \
            ESP_LOGE("ERR", "%s:(%s):Line %d failed: %s", __FILE__, __PRETTY_FUNCTION__, __LINE__, esp_err_to_name(ret_));                 \
            (err_cb);                                                                                                                      \
            return ret_;                                                                                                                   \
        }                                                                                                                                  \
    } while (0)

namespace utils {

    struct spi_bus_config_t {
        gpio_num_t mosi_pin{GPIO_NUM_NC};
        gpio_num_t miso_pin{GPIO_NUM_NC};
        gpio_num_t sclk_pin{GPIO_NUM_NC};
    };

    [[nodiscard]] inline esp_err_t init_spi_bus(spi_host_device_t bus, int max_trans_size, const spi_bus_config_t& config) {

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
            .max_transfer_sz       = max_trans_size,
            .flags                 = (SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_IOMUX_PINS),
            .isr_cpu_id            = ESP_INTR_CPU_AFFINITY_AUTO,
            .intr_flags            = 0,
        };
        TRY(spi_bus_initialize(bus, &bus_config, SPI_DMA_CH_AUTO));

        return ESP_OK;
    };

} // namespace utils
