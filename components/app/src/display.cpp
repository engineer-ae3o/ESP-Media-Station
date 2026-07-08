#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "utils.hpp"
#include "config.hpp"
#include "display.hpp"
#include "ili9341.hpp"
#include "xpt2046.hpp"

#include "esp_log.h"
#include "esp_cache.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include "lvgl.h"

#include <atomic>

namespace display {

    namespace {

        ili9341_t        g_ili9341{};
        touch::xpt2046_t g_xpt2046{};

        uint8_t* g_frame_buffer_1{};
        uint8_t* g_frame_buffer_2{};

        bool              g_is_initialized{};
        std::atomic<bool> g_is_rendering{};

        lv_display_t* g_display = nullptr;

        TaskHandle_t     g_render_task_handle   = nullptr;
        constexpr size_t RENDER_TASK_STACK_SIZE = 8192; // The call stack for lv_timer_handler(...) runs deep
        constexpr size_t RENDER_PERIOD_MS       = 10;
        constexpr size_t RENDER_TASK_PRIORITY   = 3; // Fairly low

        esp_timer_handle_t g_lvgl_tick_timer = nullptr;

        constexpr const char* TAG               = "DISPLAY";
        constexpr size_t      BUFFER_SIZE_BYTES = ili9341_t::MAX_HEIGHT * ili9341_t::MAX_WIDTH * sizeof(uint16_t);

        void cleanup() {

            if (g_render_task_handle) {
                // Safe to delete without synchronisation.
                vTaskDelete(g_render_task_handle);
                g_render_task_handle = nullptr;
            }

            if (g_lvgl_tick_timer) {
                esp_timer_stop(g_lvgl_tick_timer);
                esp_timer_delete(g_lvgl_tick_timer);
                g_lvgl_tick_timer = nullptr;
            }

            if (g_display) {
                lv_display_delete(g_display);
                g_display = nullptr;
            }

            lv_deinit();

            if (g_frame_buffer_1) {
                heap_caps_free(g_frame_buffer_1);
                g_frame_buffer_1 = nullptr;
            }

            if (g_frame_buffer_2) {
                heap_caps_free(g_frame_buffer_2);
                g_frame_buffer_2 = nullptr;
            }

            (void)g_ili9341.deinit();
            (void)g_xpt2046.deinit();

            spi_bus_free(config::SHARED_SPI_BUS);
            g_is_initialized = false;
        }

    } // namespace

    esp_err_t init() {
        if (g_is_initialized) {
            return ESP_ERR_INVALID_STATE;
        }

        using namespace config;

        constexpr utils::spi_bus_config_t bus_config = {
            .bus            = SHARED_SPI_BUS,
            .flags          = (SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_IOMUX_PINS),
            .max_trans_size = 32 * 1024, // Hardware limit
            .mosi_pin       = SHARED_MOSI_PIN,
            .miso_pin       = SHARED_MISO_PIN,
            .sclk_pin       = SHARED_CLK_PIN,
        };
        TRY(utils::init_spi_bus(bus_config));

        constexpr config_t ili_config = {
            .spi_host           = ILI_SPI_BUS,
            .spi_clock_speed_hz = ILI_SPI_CLK_SPEED_HZ,
            .led_pin            = ILI_LED_PIN,
            .rst_pin            = ILI_RST_PIN,
            .cs_pin             = ILI_CS_PIN,
            .dc_pin             = ILI_DC_PIN,
            .rotation           = 0,
            .led_ledc_timer     = LEDC_TIMER_0,
            .led_ledc_channel   = LEDC_CHANNEL_0,
        };
        TRY_WITH_FUNC(g_ili9341.init(ili_config), cleanup());

        constexpr touch::config_t xpt_config = {
            .spi_host           = XPT_SPI_BUS,
            .clock_freq_hz      = XPT_SPI_CLK_SPEED_HZ,
            .queue_length       = 10,
            .cs_pin             = XPT_CS_PIN,
            .irq_pin            = XPT_IRQ_PIN,
            .screen_pixel_len_x = ili9341_t::MAX_WIDTH,
            .screen_pixel_len_y = ili9341_t::MAX_HEIGHT,
            .task_priority      = 16,
            .task_stack_size    = 3072,
        };
        TRY_WITH_FUNC(g_xpt2046.init(xpt_config), cleanup());

        g_frame_buffer_1 = static_cast<uint8_t*>(
            heap_caps_malloc(BUFFER_SIZE_BYTES, (MALLOC_CAP_8BIT | MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED)));
        g_frame_buffer_2 = static_cast<uint8_t*>(
            heap_caps_malloc(BUFFER_SIZE_BYTES, (MALLOC_CAP_8BIT | MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED)));

        if (g_frame_buffer_1 == nullptr || g_frame_buffer_2 == nullptr) {
            cleanup();
            return ESP_ERR_NO_MEM;
        }

        lv_init();

        g_display = lv_display_create(ili9341_t::MAX_WIDTH, ili9341_t::MAX_HEIGHT);
        if (g_display == nullptr) {
            cleanup();
            return ESP_ERR_NO_MEM;
        }

        lv_display_set_buffers(g_display, g_frame_buffer_1, g_frame_buffer_2, BUFFER_SIZE_BYTES, LV_DISPLAY_RENDER_MODE_FULL);
        lv_display_set_color_format(g_display, LV_COLOR_FORMAT_RGB565);
        lv_display_set_flush_cb(g_display, [](lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
            // Get the pixel height and width from the x and y coordinates
            const auto   width       = static_cast<uint16_t>(area->x2 - area->x1 + 1);
            const auto   height      = static_cast<uint16_t>(area->y2 - area->y1 + 1);
            const size_t pixel_count = width * height;

            const auto* px_data = reinterpret_cast<const uint16_t*>(px_map);

            // Flush the cache so the new data in the buffer are visible to the DMA controller
            auto ret =
                esp_cache_msync(px_map, (pixel_count * sizeof(uint16_t)), ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_TYPE_DATA);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to perform cache writeback op: %s", esp_err_to_name(ret));
                // If a failure happened, no point in transferring a stale buffer
                lv_display_flush_ready(g_display);
                return;
            }

            ret = g_ili9341.flush({static_cast<uint16_t>(area->x1),
                                   static_cast<uint16_t>(area->y1),
                                   static_cast<uint16_t>(area->x2),
                                   static_cast<uint16_t>(area->y2)},
                                  {px_data, pixel_count});
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to transmit framebuffer: %s", esp_err_to_name(ret));
            }

            lv_display_flush_ready(g_display);
        });

        // LVGL tick timer. Required by LVGL to increment it's internal timers and state
        constexpr esp_timer_create_args_t tick_timer_args = {
            .callback =
                [](void* arg) {
                    lv_tick_inc(5); // 5ms
                },
            .arg                   = nullptr,
            .dispatch_method       = ESP_TIMER_ISR,
            .name                  = "lvgl_tick",
            .skip_unhandled_events = false,
        };
        TRY_WITH_FUNC(esp_timer_create(&tick_timer_args, &g_lvgl_tick_timer), cleanup());
        TRY_WITH_FUNC(esp_timer_start_periodic(g_lvgl_tick_timer, 5 * 1'000), cleanup()); // 5ms

        // Render task. Calls lv_timer_handler(...) to refresh the screen
        auto ret = xTaskCreate(
            [](void* arg) {
                while (true) {
                    // No need for locks here. LVGL does so internally already
                    // Just check and wait till the rendering flag is true
                    g_is_rendering.wait(false, std::memory_order_acquire);
                    lv_timer_handler();
                    vTaskDelay(pdMS_TO_TICKS(RENDER_PERIOD_MS));
                }
            },
            "Render Task",
            RENDER_TASK_STACK_SIZE,
            nullptr,
            RENDER_TASK_PRIORITY,
            &g_render_task_handle);
        if (ret != pdPASS) {
            cleanup();
            return ESP_ERR_NO_MEM;
        }

        g_is_initialized = true;

        return ESP_OK;
    }

    esp_err_t deinit() {
        if (!g_is_initialized) {
            return ESP_ERR_INVALID_STATE;
        }

        cleanup();

        return ESP_OK;
    }

    esp_err_t pause_rendering(bool pause) {
        g_is_rendering.store(!pause, std::memory_order_release);
        g_is_rendering.notify_one();
        return ESP_OK;
    }

} // namespace display
