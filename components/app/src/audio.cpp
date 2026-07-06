#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "utils.hpp"
#include "audio.hpp"
#include "inmp441.hpp"
#include "max98357.hpp"

#include "esp_err.h"
#include "esp_log.h"

namespace audio::pipeline {

    esp_err_t init() {
        return ESP_OK;
    }

    esp_err_t deinit() {
        return ESP_OK;
    }

} // namespace audio::pipeline
