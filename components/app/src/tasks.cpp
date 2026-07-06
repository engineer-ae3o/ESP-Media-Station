#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tasks.hpp"
#include "audio.hpp"
#include "display.hpp"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_littlefs.h"

namespace tasks {

    esp_err_t filesystem_init() {
        constexpr esp_vfs_littlefs_conf_t config = {
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
        TRY(esp_vfs_littlefs_register(&config));
        return ESP_OK;
    }

    void run() {
    }

} // namespace tasks
