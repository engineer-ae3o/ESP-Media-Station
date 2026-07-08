#pragma once

#include "esp_err.h"

namespace display {

    [[nodiscard]] esp_err_t init();

    [[nodiscard]] esp_err_t deinit();

    [[nodiscard]] esp_err_t pause_rendering(bool pause = true);

} // namespace display
