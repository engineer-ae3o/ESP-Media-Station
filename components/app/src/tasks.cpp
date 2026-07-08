#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tasks.hpp"
#include "audio.hpp"
#include "config.hpp"
#include "display.hpp"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_littlefs.h"

namespace tasks {

    constexpr const char* TAG = "TASKS";

    namespace {

        [[nodiscard]] esp_err_t filesystem_init() {
            constexpr esp_vfs_littlefs_conf_t config = {
                .base_path              = config::FILESYSTEM_BASE_PATH,
                .partition_label        = config::FILESYSTEM_PARTITION_LABEL,
                .partition              = nullptr,
                .sdcard                 = nullptr,
                .blockdev               = nullptr,
                .format_if_mount_failed = 1,
                .read_only              = 0,
                .dont_mount             = 0,
                .grow_on_mount          = 1,
            };
            TRY(esp_vfs_littlefs_register(&config));
            return ESP_OK;
        }

        [[nodiscard]] esp_err_t filesystem_deinit() {
            TRY(esp_vfs_littlefs_unregister(config::FILESYSTEM_PARTITION_LABEL));
            return ESP_OK;
        }

        // These are all critical to the runtime of the system.
        // If any fail to be initialized, restart the system as we can't proceed
        void init_all() {
            auto ret = filesystem_init();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to mount filesystem");
                esp_restart();
            }

            ret = audio::pipeline::init();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to startup the audio pipeline");
                esp_restart();
            }
        }

    } // namespace

    void run() {
    }

} // namespace tasks
