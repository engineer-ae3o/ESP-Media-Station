#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "utils.hpp"
#include "audio.hpp"
#include "config.hpp"
#include "inmp441.hpp"
#include "max98357.hpp"

#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"

#include "esp_audio_enc.h"
#include "esp_audio_dec.h"
#include "esp_audio_enc_default.h"
#include "esp_audio_dec_default.h"

#include <array>
#include <atomic>
#include <cstdio>
#include <cstdint>
#include <utility>
#include <source_location>

namespace audio::pipeline {

    namespace {

        void try_esp_audio_func(esp_audio_err_t func, std::source_location location = std::source_location::current()) {
            if (func != ESP_AUDIO_ERR_OK) {
                ESP_LOGE("ERROR", "%s:(%s):Line %d failed: %d", __FILE__, __PRETTY_FUNCTION__, __LINE__, func);
                utils::fatal(location);
            }
        }

        amp::max98357a_t g_max98357{};
        mic::inmp441_t   g_inmp441{};

        TaskHandle_t  g_audio_task_handle{};
        QueueHandle_t g_record_btn_queue{};
        TimerHandle_t g_record_btn_debounce_timer{};

        bool              g_is_initialized{};
        std::atomic<bool> g_shutdown_requested{};

        constexpr const char* TAG = "Audio";

        constexpr std::array<const char*, std::to_underlying(file_t::COUNT)> FILE_LUT = {{
            [std::to_underlying(file_t::RECORD_FILE)] = "/lfs/audio/record_audio.opus",
        }};

        enum class record_t : uint8_t {
            START,
            STOP,
        };

        void on_first_boot() {
        }

        void cleanup() {
            if (g_record_btn_queue) {
                vQueueDelete(g_record_btn_queue);
                g_record_btn_queue = nullptr;
            }

            if (g_record_btn_debounce_timer) {
                xTimerStop(g_record_btn_debounce_timer, portMAX_DELAY);
                xTimerDelete(g_record_btn_debounce_timer, portMAX_DELAY);
                g_record_btn_debounce_timer = nullptr;
            }

            gpio_intr_disable(config::RECORD_BUTTON);
            gpio_isr_handler_remove(config::RECORD_BUTTON);
            gpio_reset_pin(config::RECORD_BUTTON);

            (void)g_inmp441.deinit();
            (void)g_max98357.deinit();

            g_audio_task_handle = nullptr;
            g_is_initialized    = false;
        }

        void isr_handler(void* arg) {
            gpio_intr_disable(config::RECORD_BUTTON);
            BaseType_t higher_priority_task_woken = pdFALSE;
            xTimerStartFromISR(g_record_btn_debounce_timer, &higher_priority_task_woken);
            if (higher_priority_task_woken == pdTRUE) {
                portYIELD_FROM_ISR();
            } else {
                gpio_intr_enable(config::RECORD_BUTTON);
            }
        }

        void debounce_timer_cb(TimerHandle_t handle) {
            if (gpio_get_level(config::RECORD_BUTTON)) {
                gpio_intr_enable(config::RECORD_BUTTON);
                return;
            }

            static bool    is_recording = false;
            const record_t event        = is_recording ? record_t::STOP : record_t::START;
            is_recording                = !is_recording;

            auto ret = xQueueSend(g_record_btn_queue, &event, 0);
            if (ret != pdPASS) {
                ESP_LOGE(TAG, "Failed to send event to record button event queue");
            }

            gpio_intr_enable(config::RECORD_BUTTON);
            portYIELD();
        }

        void opus_codec_init(esp_audio_enc_handle_t& encoder, esp_audio_dec_handle_t& decoder) {
            // Register built in encoders and decoders
            try_esp_audio_func(esp_audio_enc_register_default());
            try_esp_audio_func(esp_audio_dec_register_default());

            constexpr uint32_t OPUS_BIT_RATE   = 40'000;
            constexpr uint32_t OPUS_COMPLEXITY = 6;

            // Configure the opus encoder
            esp_opus_enc_config_t opus_enc_config = {
                .sample_rate      = mic::inmp441_t::SAMPLE_RATE_HZ,
                .channel          = ESP_AUDIO_MONO,
                .bits_per_sample  = ESP_AUDIO_BIT16,
                .bitrate          = OPUS_BIT_RATE,
                .frame_duration   = ESP_OPUS_ENC_FRAME_DURATION_20_MS,
                .application_mode = ESP_OPUS_ENC_APPLICATION_VOIP,
                .complexity       = OPUS_COMPLEXITY,
                .enable_fec       = true,
                .enable_dtx       = false,
                .enable_vbr       = false,
            };

            esp_audio_enc_config_t enc_config = {
                .type   = ESP_AUDIO_TYPE_OPUS,
                .cfg    = &opus_enc_config,
                .cfg_sz = sizeof(esp_opus_enc_config_t),
            };
            try_esp_audio_func(esp_audio_enc_open(&enc_config, &encoder));

            // Configure the opus decoder
            esp_opus_dec_cfg_t opus_dec_config = {
                .sample_rate    = mic::inmp441_t::SAMPLE_RATE_HZ,
                .channel        = ESP_AUDIO_MONO,
                .frame_duration = ESP_OPUS_DEC_FRAME_DURATION_20_MS,
                .self_delimited = false,
            };

            esp_audio_dec_cfg_t dec_config = {
                .type   = ESP_AUDIO_TYPE_OPUS,
                .cfg    = &opus_dec_config,
                .cfg_sz = sizeof(esp_opus_dec_cfg_t),
            };
            try_esp_audio_func(esp_audio_dec_open(&dec_config, &decoder));
        }

        void opus_codec_deinit(esp_audio_enc_handle_t& encoder, esp_audio_dec_handle_t& decoder) {
            esp_audio_enc_close(encoder);
            esp_audio_dec_close(decoder);

            encoder = nullptr;
            decoder = nullptr;

            esp_audio_enc_unregister_default();
            esp_audio_dec_unregister_default();
        }

        void record_task(void* arg) {

            FILE* record_file = nullptr;

            esp_audio_enc_handle_t encoder{};
            esp_audio_dec_handle_t decoder{};
            opus_codec_init(encoder, decoder);

            while (g_shutdown_requested.load(std::memory_order_acquire)) {
                record_t event{};
                xQueueReceive(g_record_btn_queue, &event, portMAX_DELAY);

                if (event == record_t::START) {
                    // Always zero out the file when opening
                    record_file = fopen(FILE_LUT[std::to_underlying(file_t::RECORD_FILE)], "w");
                    if (record_file == nullptr) {
                        ESP_LOGE(TAG, "Failed to open file to store audio. Running out of flash, perhaps");
                        continue;
                    }

                } else if (event == record_t::STOP) {
                    if (record_file) {
                        TRY_THEN_LOG(g_inmp441.enable(false), "Failed to disable the INMP441");
                        fclose(record_file);
                        record_file = nullptr;
                    }
                } else {
                }
            }

            opus_codec_deinit(encoder, decoder);
            vTaskDelete(nullptr);
        }

        void audio_task(void* arg) {

            while (true) {
            }

            cleanup();
            vTaskDelete(nullptr);
        }

    } // namespace

    [[nodiscard]] esp_err_t init() {
        if (g_is_initialized) {
            return ESP_ERR_INVALID_STATE;
        }

        using namespace config;

        auto ret = xTaskCreate(audio_task, "Audio Task", AUDIO_TASK_STACK_SIZE, nullptr, AUDIO_TASK_PRIORITY, &g_audio_task_handle);
        if (ret != pdPASS) {
            return ESP_ERR_NO_MEM;
        }

        ret = xTaskCreate(record_task, "Record Task", RECORD_TASK_STACK_SIZE, nullptr, RECORD_TASK_PRIORITY, nullptr);
        if (ret != pdPASS) {
            g_shutdown_requested.store(true, std::memory_order_release);
            return ESP_ERR_NO_MEM;
        }

        constexpr amp::config_t amp_config = {
            .bclk_pin = MAX_BCLK_PIN,
            .dout_pin = MAX_DOUT_PIN,
            .gain_pin = MAX_GAIN_PIN,
            .ws_pin   = MAX_WS_PIN,
            .sd_pin   = MAX_SD_PIN,
        };
        TRY_WITH_FUNC(g_max98357.init(amp_config), g_shutdown_requested.store(true, std::memory_order_release));
        TRY_WITH_FUNC(g_max98357.power_on(), g_shutdown_requested.store(true, std::memory_order_release));

        constexpr mic::config_t inmp_config = {
            .use_right_chan = false,
            .error_cb =
                [](esp_err_t err) {
                    static uint32_t err_counter = 0;
                    ESP_LOGE("ERROR", "Error %u occurred while writing to the INMP441's buffers: %s", err_counter, esp_err_to_name(err));
                    err_counter++;
                    if (err_counter >= MAX_ERR_COUNT) {
                        ESP_LOGE("ERROR", "Too many errors occured while writing to INMP441's buffer");
                        utils::fatal();
                    }
                },
            .chip_en_pin = INMP_CHIPEN_PIN,
            .bclk_pin    = INMP_BCLK_PIN,
            .din_pin     = INMP_DIN_PIN,
            .l_r_pin     = INMP_L_R_PIN,
            .ws_pin      = INMP_WS_PIN,
        };
        TRY_WITH_FUNC(g_inmp441.init(inmp_config), g_shutdown_requested.store(true, std::memory_order_release));
        TRY_WITH_FUNC(g_inmp441.enable(), g_shutdown_requested.store(true, std::memory_order_release));

        constexpr gpio_config_t config = {
            .pin_bit_mask = (1ULL << RECORD_BUTTON),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_NEGEDGE,
        };
        TRY_WITH_FUNC(gpio_config(&config), g_shutdown_requested.store(true, std::memory_order_release));
        TRY_WITH_FUNC(gpio_isr_handler_add(RECORD_BUTTON, isr_handler, nullptr),
                      g_shutdown_requested.store(true, std::memory_order_release));

        g_record_btn_queue = xQueueCreate(QUEUE_LEN, sizeof(record_t));
        g_record_btn_debounce_timer =
            xTimerCreate("Record Button Debounce Timer", pdMS_TO_TICKS(DEBOUNCE_TIME_MS), pdFALSE, nullptr, debounce_timer_cb);

        if (g_record_btn_queue == nullptr || g_record_btn_debounce_timer == nullptr) {
            cleanup();
            return ESP_ERR_NO_MEM;
        }

        g_is_initialized = true;

        return ESP_OK;
    }

    [[nodiscard]] esp_err_t deinit() {
        if (!g_is_initialized) {
            return ESP_ERR_INVALID_STATE;
        }

        // No point in waiting for the audio task to finish its cleanup
        g_shutdown_requested.store(true, std::memory_order_release);

        return ESP_OK;
    }

} // namespace audio::pipeline
