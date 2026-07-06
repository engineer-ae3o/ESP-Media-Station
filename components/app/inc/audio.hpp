#pragma once

#include "esp_err.h"

namespace audio::pipeline {

    esp_err_t init();

    esp_err_t deinit();

} // namespace audio::pipeline
