#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led_strip.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"

namespace {

    [[noreturn]] void led_task(void* arg) {
        (void)arg;

        led_strip_handle_t strip{};

        constexpr led_strip_config_t strip_config = {
            .strip_gpio_num         = 48,
            .max_leds               = 1,
            .led_model              = LED_MODEL_WS2812,
            .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB,
            .flags                  = {.invert_out = 0},
        };
        constexpr led_strip_rmt_config_t rmt_config = {
            .clk_src           = RMT_CLK_SRC_DEFAULT,
            .resolution_hz     = 1'000'000,
            .mem_block_symbols = 128,
            .flags             = {.with_dma = 1},
        };

        ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &strip));

        while (true) {
            ESP_ERROR_CHECK(led_strip_set_pixel(strip, 0, 255, 0, 0));
            ESP_ERROR_CHECK(led_strip_refresh(strip));
            ESP_LOGI("Led Task", "RED");
            vTaskDelay(pdMS_TO_TICKS(1000));

            ESP_ERROR_CHECK(led_strip_set_pixel(strip, 0, 0, 255, 0));
            ESP_ERROR_CHECK(led_strip_refresh(strip));
            ESP_LOGI("Led Task", "GREEN");
            vTaskDelay(pdMS_TO_TICKS(1000));

            ESP_ERROR_CHECK(led_strip_set_pixel(strip, 0, 0, 0, 255));
            ESP_ERROR_CHECK(led_strip_refresh(strip));
            ESP_LOGI("Led Task", "BLUE");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

} // namespace

extern "C" {
    void app_main() {
        auto ret = xTaskCreate(led_task, "Led Task", 2048, {}, 2, {});
        if (ret != pdPASS) {
            ESP_LOGE("ERROR", "Failed to create Led Task");
            esp_restart();
        }
    }
}
