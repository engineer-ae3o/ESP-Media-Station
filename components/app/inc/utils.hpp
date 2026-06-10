#pragma once

namespace utils {

#define TRY(func)                                                                                                                          \
    do {                                                                                                                                   \
        if (auto ret_ = (func); ret_ != ESP_OK) {                                                                                          \
            ESP_LOGE(TAG, "%s failed: %s", #func, esp_err_to_name(ret_));                                                                  \
            return ret_;                                                                                                                   \
        }                                                                                                                                  \
    } while (0)

} // namespace utils
