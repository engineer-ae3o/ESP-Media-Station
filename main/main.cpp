#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_err.h"

#include "ili9341.hpp"
#include "config.hpp"

namespace {

    void disp_task(void* arg) {
        (void)arg;

        constexpr spi_bus_config_t bus_config = {
            .mosi_io_num           = config::LCD_MOSI_PIN,
            .miso_io_num           = GPIO_NUM_NC,
            .sclk_io_num           = config::LCD_CLK_PIN,
            .quadwp_io_num         = GPIO_NUM_NC,
            .quadhd_io_num         = GPIO_NUM_NC,
            .data4_io_num          = GPIO_NUM_NC,
            .data5_io_num          = GPIO_NUM_NC,
            .data6_io_num          = GPIO_NUM_NC,
            .data7_io_num          = GPIO_NUM_NC,
            .data_io_default_level = false,
            .max_transfer_sz       = (disp::MAX_WIDTH * disp::MAX_HEIGHT * 2),
            .flags                 = SPICOMMON_BUSFLAG_IOMUX_PINS,
            .isr_cpu_id            = ESP_INTR_CPU_AFFINITY_AUTO,
            .intr_flags            = 0,
        };
        ESP_ERROR_CHECK(spi_bus_initialize(config::LCD_SPI_BUS, &bus_config, SPI_DMA_CH_AUTO));

        constexpr disp::config_t config = {
            .spi_host           = config::LCD_SPI_BUS,
            .spi_clock_speed_hz = config::LCD_SPI_CLK_SPEED_HZ,
            .cs                 = config::LCD_CS_PIN,
            .dc                 = config::LCD_DC_PIN,
            .rst                = config::LCD_RST_PIN,
            .width              = disp::MAX_WIDTH,
            .height             = disp::MAX_HEIGHT,
            .rotation           = 0,
        };

        disp::ili9341_t display;
        ESP_ERROR_CHECK(display.init(config));

        uint16_t color{0xF100};

        while (true) {
            ESP_ERROR_CHECK(display.set_screen(color));
            ESP_LOGI("MAIN", "Color: 0x%X", color);

            color += 100;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

} // namespace

extern "C" {
    void app_main() {
        auto ret = xTaskCreate(disp_task, "Display Task", 4096, {}, 4, {});
        if (ret != pdPASS) {
            ESP_LOGE("MAIN", "Failed to create Display Task");
            assert(0);
        }
    }
}
