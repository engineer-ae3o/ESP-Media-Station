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

        constexpr utils::spi_bus_config_t ili9341_bus_config = {
            .mosi_pin = config::LCD_MOSI_PIN,
            .miso_pin = GPIO_NUM_NC,
            .sclk_pin = config::LCD_CLK_PIN,
        };
        ESP_ERROR_CHECK(
            utils::init_spi_bus(config::LCD_SPI_BUS, disp::ili9341_t::MAX_WIDTH * disp::ili9341_t::MAX_HEIGHT, ili9341_bus_config));

        constexpr disp::config_t config = {
            .spi_host           = config::LCD_SPI_BUS,
            .spi_clock_speed_hz = config::LCD_SPI_CLK_SPEED_HZ,
            .led_pin            = config::LCD_LED_PIN,
            .rst_pin            = config::LCD_RST_PIN,
            .cs_pin             = config::LCD_CS_PIN,
            .dc_pin             = config::LCD_DC_PIN,
            .rotation           = 0,
            .led_ledc_timer     = LEDC_TIMER_1,
            .led_ledc_channel   = LEDC_CHANNEL_0,
        };

        disp::ili9341_t display;
        ESP_ERROR_CHECK(disp::ili9341_t::init_ledc_timer(LEDC_TIMER_1));
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
