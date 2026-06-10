#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led_strip.h"

#include "esp_log.h"
#include "esp_err.h"

namespace {

    constexpr auto LED_PIN = GPIO_NUM_48;
    constexpr auto TAG     = "MAIN";

    led_strip_handle_t configure_led() {

        constexpr led_strip_config_t strip_config = {
            .strip_gpio_num         = LED_PIN,
            .max_leds               = 1,
            .led_model              = LED_MODEL_WS2812,
            .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
            .flags                  = {.invert_out = 0},
        };

        constexpr led_strip_rmt_config_t rmt_config = {
            .clk_src           = RMT_CLK_SRC_DEFAULT,
            .resolution_hz     = 10'000'000,
            .mem_block_symbols = 128,
            .flags             = {.with_dma = 1},
        };

        // LED Strip object handle
        led_strip_handle_t led_strip{};
        ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

        ESP_LOGI(TAG, "Created LED strip object with SPI backend");
        return led_strip;
    }

} // namespace

extern "C" {
    void app_main() {
        led_strip_handle_t led_strip = configure_led();
        ESP_LOGI(TAG, "Start blinking LED strip");

        while (true) {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 255, 0, 0));
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            ESP_LOGI(TAG, "RED");
            vTaskDelay(pdMS_TO_TICKS(1'000));

            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 255, 0));
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            ESP_LOGI(TAG, "GREEN");
            vTaskDelay(pdMS_TO_TICKS(1'000));

            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 0, 255));
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            ESP_LOGI(TAG, "BLUE");
            vTaskDelay(pdMS_TO_TICKS(1'000));
        }
    }
}
