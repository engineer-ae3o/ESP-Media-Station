#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "utils.hpp"
#include "config.hpp"
#include "audio.hpp"
#include "inmp441.hpp"
#include "max98357.hpp"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"

namespace audio::pipeline {

    namespace {

        amp::max98357a_t g_max98357{};
        mic::inmp441_t   g_inmp441{};

    } // namespace

    [[nodiscard]] esp_err_t init() {

        using namespace config;

        constexpr amp::config_t amp_config = {
            .bclk_pin = MAX_BCLK_PIN,
            .dout_pin = MAX_DOUT_PIN,
            .gain_pin = MAX_GAIN_PIN,
            .ws_pin   = MAX_WS_PIN,
            .sd_pin   = MAX_SD_PIN,
        };
        TRY(g_max98357.init(amp_config));
        TRY(g_max98357.power_on());

        constexpr mic::config_t inmp_config = {
            .use_right_chan = false,
            .error_cb =
                [](esp_err_t err) {
                    static uint32_t err_counter = 0;
                    ESP_LOGE("ERROR", "Error %u occurred while writing to the INMP441's buffers: %s", err_counter, esp_err_to_name(err));
                    err_counter++;
                    if (err_counter >= MAX_ERR_COUNT) {
                        ESP_LOGE("ERROR", "Too many errors occured while writing to INMP441's buffer. Restarting system");
                        esp_restart();
                    }
                },
            .chip_en_pin = INMP_CHIPEN_PIN,
            .bclk_pin    = INMP_BCLK_PIN,
            .din_pin     = INMP_DIN_PIN,
            .l_r_pin     = INMP_L_R_PIN,
            .ws_pin      = INMP_WS_PIN,
        };

        TRY(g_inmp441.init(inmp_config));
        TRY(g_inmp441.enable());

        return ESP_OK;
    }

    [[nodiscard]] esp_err_t deinit() {
        TRY(g_inmp441.deinit());
        TRY(g_max98357.deinit());
        return ESP_OK;
    }

} // namespace audio::pipeline
