#pragma once

#include "esp_err.h"

namespace audio::pipeline {

    enum class file_t : uint8_t {
        RECORD_FILE,
        COUNT,
    };

    [[nodiscard]] esp_err_t init();

    [[nodiscard]] esp_err_t deinit();

} // namespace audio::pipeline
