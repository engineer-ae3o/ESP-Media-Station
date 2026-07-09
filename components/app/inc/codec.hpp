#pragma once

#include "inmp441.hpp"

#include "esp_err.h"
#include "esp_audio_enc.h"

#include <span>
#include <cstdint>

namespace audio::codec_opus {

    constexpr uint32_t BIT_RATE          = 40'000;
    constexpr uint32_t FRAME_DURATION_MS = 20;

    constexpr uint32_t SAMPLES_PER_FRAME = (mic::inmp441_t::SAMPLE_RATE_HZ * FRAME_DURATION_MS) / 1'000;
    constexpr uint32_t FRAME_SIZE_BYTES  = SAMPLES_PER_FRAME * (ESP_AUDIO_BIT16 / 8) * 1; // 1 channel

    void init();

    void deinit();

    [[nodiscard]] esp_err_t encode(std::span<uint8_t> pcm_in, std::span<uint8_t> opus_out);

    [[nodiscard]] esp_err_t decode(std::span<uint8_t> opus_in, std::span<uint8_t> pcm_out);

} // namespace audio::codec_opus
