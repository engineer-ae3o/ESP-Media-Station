#pragma once

#include "esp_err.h"
#include "esp_log.h"

namespace utils {} // namespace utils

#define TRY(func)                                                                                                                          \
    do {                                                                                                                                   \
        if (auto ret_ = (func); ret_ != ESP_OK) {                                                                                          \
            ESP_LOGE("ERR", "%s:%s:Line %d failed: %s", __FILE__, #func, __LINE__, esp_err_to_name(ret_));                                 \
            return ret_;                                                                                                                   \
        }                                                                                                                                  \
    } while (0)

#define TRY_WITH_FUNC(func, err_cb)                                                                                                        \
    do {                                                                                                                                   \
        if (auto ret_ = (func); ret_ != ESP_OK) {                                                                                          \
            ESP_LOGE("ERR", "%s:%s:Line %d failed: %s", __FILE__, #func, __LINE__, esp_err_to_name(ret_));                                 \
            (err_cb);                                                                                                                      \
            return ret_;                                                                                                                   \
        }                                                                                                                                  \
    } while (0)
