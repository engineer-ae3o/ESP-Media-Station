#include "max98357.hpp"

namespace amp {

    esp_err_t max98357_t::init(const config_t& config) {
        return ESP_OK;
    }

    esp_err_t max98357_t::deinit() {
        return ESP_OK;
    }

    esp_err_t max98357_t::send_audio_buf(std::span<int32_t> data) {
        return ESP_OK;
    }

    void max98357_t::cleanup_resources() {
    }

} // namespace amp
