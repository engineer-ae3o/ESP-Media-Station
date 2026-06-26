#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "driver/i2c_master.h"

#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_err.h"

#include "max98357.hpp"
#include "inmp441.hpp"
#include "ili9341.hpp"
#include "config.hpp"

#include <span>

namespace {

    [[noreturn]] void disp_task(void* arg) {
        (void)arg;

        [[maybe_unused]] constexpr auto* TAG = "DISP_CAM";

        constexpr utils::spi_bus_config_t bus_config = {
            .bus            = config::LCD_SPI_BUS,
            .max_trans_size = disp::ili9341_t::MAX_WIDTH * disp::ili9341_t::MAX_HEIGHT * 2,
            .mosi_pin       = config::LCD_MOSI_PIN,
            .miso_pin       = GPIO_NUM_NC,
            .sclk_pin       = config::LCD_CLK_PIN,
        };
        ESP_ERROR_CHECK(utils::init_spi_bus(bus_config));

        constexpr disp::config_t config = {
            .spi_host           = config::LCD_SPI_BUS,
            .spi_clock_speed_hz = config::LCD_SPI_CLK_SPEED_HZ,
            .led_pin            = config::LCD_LED_PIN,
            .rst_pin            = config::LCD_RST_PIN,
            .cs_pin             = config::LCD_CS_PIN,
            .dc_pin             = config::LCD_DC_PIN,
            .rotation           = 0,
            .led_ledc_timer     = LEDC_TIMER_0,
            .led_ledc_channel   = LEDC_CHANNEL_0,
        };

        disp::ili9341_t display;
        ESP_ERROR_CHECK(disp::ili9341_t::init_ledc_timer(LEDC_TIMER_0));
        ESP_ERROR_CHECK(display.init(config));
        ESP_ERROR_CHECK(display.set_brightness(255));

        while (true) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    /**
    [[noreturn]] void audio_task(void* arg) {
        (void)arg;

        // Initialize the MAX98357A audio amplifier
        constexpr audio::amp::config_t max_config = {
            .bclk_pin = config::MAX_BCLK,
            .dout_pin = config::MAX_DATA,
            .gain_pin = config::MAX_GAIN,
            .ws_pin   = config::MAX_WS,
            .sd_pin   = config::MAX_SD,
        };

        audio::amp::max98357a_t<audio::amp::gain_t::dB_12, audio::amp::mode_t::LEFT_CHANNEL> amp;
        ESP_ERROR_CHECK(amp.init(max_config));
        ESP_ERROR_CHECK(amp.power_on());

        // Initialize the INMP441 microphone
        constexpr audio::mic::config_t inmp_config = {
            .use_right_chan = false,
            .error_cb       = nullptr,
            .chip_en_pin    = config::INMP_CHIPEN,
            .bclk_pin       = config::INMP_BCLK,
            .din_pin        = config::INMP_DATA,
            .l_r_pin        = config::INMP_L_R,
            .ws_pin         = config::INMP_WS,
        };

        audio::mic::inmp441_t mic;
        ESP_ERROR_CHECK(mic.init(inmp_config));

        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    */

} // namespace

extern "C" {
    void app_main() {
        auto ret = xTaskCreate(disp_task, "Display Task", 4096, {}, 4, {});
        if (ret != pdPASS) {
            ESP_LOGE("MAIN", "Failed to create Display Task");
            assert(0);
        }

        // LittleFS initialization
        constexpr esp_vfs_littlefs_conf_t littlefs_config = {
            .base_path              = "/lfs",
            .partition_label        = "storage",
            .partition              = nullptr,
            .sdcard                 = nullptr,
            .blockdev               = nullptr,
            .format_if_mount_failed = 1,
            .read_only              = 0,
            .dont_mount             = 0,
            .grow_on_mount          = 1,
        };
        ESP_ERROR_CHECK(esp_vfs_littlefs_register(&littlefs_config));
    }
}
