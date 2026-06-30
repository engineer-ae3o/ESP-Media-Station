#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"

#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_err.h"

#include "config.hpp"
#include "inmp441.hpp"
#include "ili9341.hpp"
#include "xpt2046.hpp"
#include "max98357.hpp"

namespace {

    [[noreturn]] void disp_task(void* arg) {
        (void)arg;

        [[maybe_unused]] constexpr auto* TAG = "ILI_XPT";

        // Initialize the SPI bus on which the ILI9341 resides
        constexpr utils::spi_bus_config_t ili_bus_config = {
            .bus            = config::ILI_SPI_BUS,
            .max_trans_size = config::ILI_MAX_TRANS_SIZE,
            .mosi_pin       = config::ILI_MOSI_PIN,
            .miso_pin       = GPIO_NUM_NC,
            .sclk_pin       = config::ILI_CLK_PIN,
        };
        ESP_ERROR_CHECK(utils::init_spi_bus(ili_bus_config));

        // Initialize the ILI9341
        constexpr disp::config_t ili_config = {
            .spi_host           = config::ILI_SPI_BUS,
            .spi_clock_speed_hz = config::ILI_SPI_CLK_SPEED_HZ,
            .led_pin            = config::ILI_LED_PIN,
            .rst_pin            = config::ILI_RST_PIN,
            .cs_pin             = config::ILI_CS_PIN,
            .dc_pin             = config::ILI_DC_PIN,
            .rotation           = 0,
            .led_ledc_timer     = LEDC_TIMER_0,
            .led_ledc_channel   = LEDC_CHANNEL_0,
        };

        disp::ili9341_t ili9341;
        ESP_ERROR_CHECK(disp::ili9341_t::init_ledc_timer(LEDC_TIMER_0));
        ESP_ERROR_CHECK(ili9341.init(ili_config));
        ESP_ERROR_CHECK(ili9341.set_brightness());

        // Initialize the SPI bus on which the XPT2046 resides
        constexpr utils::spi_bus_config_t xpt_bus_config = {
            .bus            = config::XPT_SPI_BUS,
            .max_trans_size = config::XPT_MAX_TRANS_SIZE,
            .mosi_pin       = config::XPT_MOSI_PIN,
            .miso_pin       = config::XPT_MISO_PIN,
            .sclk_pin       = config::XPT_CLK_PIN,
        };
        ESP_ERROR_CHECK(utils::init_spi_bus(xpt_bus_config));

        // Initialize the XPT2046
        constexpr touch::config_t xpt_config = {
            .spi_host      = config::XPT_SPI_BUS,
            .clock_freq_hz = config::XPT_SPI_CLK_SPEED_HZ,
            .queue_length  = 5,
            .cs_pin        = config::XPT_CS_PIN,
            .irq_pin       = config::XPT_IRQ_PIN,
        };

        touch::xpt2046_t xpt2046;
        ESP_ERROR_CHECK(xpt2046.init(xpt_config));
        auto* event_queue = xpt2046.get_event_queue();
        assert(event_queue != nullptr);

        while (true) {
            vTaskDelay(portMAX_DELAY);
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
