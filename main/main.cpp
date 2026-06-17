#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_err.h"

#include "max98357.hpp"
#include "inmp441.hpp"
#include "ili9341.hpp"
#include "config.hpp"

namespace {

    [[noreturn]] void disp_task(void* arg) {
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
            .rotation           = 0,
        };

        disp::ili9341_t display;
        ESP_ERROR_CHECK(display.init(config));

        uint16_t color{0xF100}; // Start at RED

        while (true) {
            ESP_ERROR_CHECK(display.set_screen(color));
            ESP_LOGI("MAIN", "Color: 0x%X", color);

            color += 100;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    [[noreturn]] void audio_task(void* arg) {
        (void)arg;

        // Initialize the MAX98357A audio amplifier
        constexpr amp::config_t max_config = {
            .bclk = config::MAX_BCLK,
            .data = config::MAX_DATA,
            .gain = config::MAX_GAIN,
            .ws   = config::MAX_WS,
            .sd   = config::MAX_SD,
        };

        amp::max98357a_t<amp::gain_t::dB_12, amp::mode_t::LEFT_CHANNEL> max98357;
        ESP_ERROR_CHECK(max98357.init(max_config));
        ESP_ERROR_CHECK(max98357.power_on());

        // Initialize the INMP441 microphone
        constexpr mic::config_t inmp_config = {
            .use_right_chan     = false,
            .enable_during_init = true,
            .chip_en            = config::INMP_CHIPEN,
            .bclk               = config::INMP_BCLK,
            .data               = config::INMP_DATA,
            .l_r                = config::INMP_L_R,
            .ws                 = config::INMP_WS,
        };

        mic::inmp441_t inmp441;
        ESP_ERROR_CHECK(inmp441.init(inmp_config));

        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
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
        ret = xTaskCreate(audio_task, "Audio Task", 4096, {}, 5, {});
        if (ret != pdPASS) {
            ESP_LOGE("MAIN", "Failed to create Audio Task");
            assert(0);
        }
    }
}
