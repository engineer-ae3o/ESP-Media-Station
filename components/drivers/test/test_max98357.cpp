#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "unity.h"

#include "esp_heap_caps.h"

#include "config.hpp"
#include "max98357.hpp"

#include <cmath>
#include <array>
#include <cstdint>
#include <numbers>

namespace {

    consteval auto get_test_config() {
        return audio::amp::config_t{
            .bclk_pin = config::MAX_BCLK_PIN,
            .dout_pin = config::MAX_DOUT_PIN,
            .gain_pin = config::MAX_GAIN_PIN,
            .ws_pin   = config::MAX_WS_PIN,
            .sd_pin   = config::MAX_SD_PIN,
        };
    }

    int32_t* make_sine_buf(size_t elements) {

        constexpr float freq_hz     = 440;
        constexpr float sample_rate = audio::amp::max98357a_t<>::SAMPLE_RATE_HZ; // 48,000Hz
        constexpr float pi          = std::numbers::pi_v<float>;

        auto* buf = static_cast<int32_t*>(
            heap_caps_malloc(elements * sizeof(int32_t), MALLOC_CAP_CACHE_ALIGNED | MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM));
        TEST_ASSERT_NOT_NULL_MESSAGE(buf, "Failed to allocate enough memory for buffer to store the audio sine wave");

        for (size_t i = 0; i < elements; i++) {
            const auto sample = std::sin(2.0F * pi * freq_hz * static_cast<float>(i) / sample_rate);
            buf[i]            = static_cast<int32_t>(sample * static_cast<float>(INT32_MAX) * 0.5F); // -6dB headroom
        }

        return buf;
    }

    using stereo_amp_t  = audio::amp::max98357a_t<audio::amp::gain_t::dB_12, audio::amp::mode_t::STEREO, true>;
    using left_amp_t    = audio::amp::max98357a_t<audio::amp::gain_t::dB_12, audio::amp::mode_t::LEFT_CHANNEL, true>;
    using right_amp_t   = audio::amp::max98357a_t<audio::amp::gain_t::dB_12, audio::amp::mode_t::RIGHT_CHANNEL, true>;
    using no_gain_pin_t = audio::amp::max98357a_t<audio::amp::gain_t::dB_9, audio::amp::mode_t::STEREO, false>;

} // namespace

TEST_CASE("Initialization and deinitialization", "[max98357a][i2s]") {
    stereo_amp_t   amp{};
    constexpr auto cfg = get_test_config();

    // Test valid init
    TEST_ESP_OK(amp.init(cfg));

    // Test double init (should fail with invalid state)
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, amp.init(cfg));

    // Test valid deinit
    TEST_ESP_OK(amp.deinit());

    // Test double deinit (should fail with invalid state)
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, amp.deinit());
}

TEST_CASE("Deinit while powered on cleans up without error", "[max98357a][i2s]") {
    stereo_amp_t   amp{};
    constexpr auto cfg = get_test_config();

    TEST_ESP_OK(amp.init(cfg));
    TEST_ESP_OK(amp.power_on(true));

    // deinit's cleanup_resources() powers down internally before deleting
    // the I2S channel; this shouldn't require an explicit power_on(false) first.
    TEST_ESP_OK(amp.deinit());
}

TEST_CASE("Power on/off redundant calls are rejected", "[max98357a][i2s]") {
    stereo_amp_t   amp{};
    constexpr auto cfg = get_test_config();

    TEST_ESP_OK(amp.init(cfg));

    TEST_ESP_OK(amp.power_on(true));
    // Already on: should fail with invalid state, not silently succeed
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, amp.power_on(true));

    TEST_ESP_OK(amp.power_on(false));
    // Already off: should fail with invalid state
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, amp.power_on(false));

    TEST_ESP_OK(amp.deinit());
}

TEST_CASE("Amp with no gain pin configured still inits and cleans up", "[max98357a][i2s]") {
    no_gain_pin_t  amp{};
    constexpr auto cfg = get_test_config();

    TEST_ESP_OK(amp.init(cfg));
    TEST_ESP_OK(amp.power_on(true));
    TEST_ESP_OK(amp.power_on(false));
    TEST_ESP_OK(amp.deinit());
}

TEST_CASE("send_audio_buf rejects calls before init or before power on", "[max98357a][i2s]") {
    stereo_amp_t   amp{};
    constexpr auto cfg = get_test_config();

    constexpr size_t len  = audio::amp::max98357a_t<>::SAMPLE_RATE_HZ * 0.01; // 10ms of data @ 48kHz
    auto*            sine = make_sine_buf(len);

    // Not initialized at all
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, amp.send_audio_buf({sine, len}));

    TEST_ESP_OK(amp.init(cfg));

    // Initialized but not powered on
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, amp.send_audio_buf({sine, len}));

    TEST_ESP_OK(amp.power_on(true));
    TEST_ESP_OK(amp.send_audio_buf({sine, len}));

    TEST_ESP_OK(amp.power_on(false));
    TEST_ESP_OK(amp.deinit());

    heap_caps_free(sine);
}

TEST_CASE("Stereo mode transmits a full-length buffer", "[max98357a][i2s][audible]") {
    stereo_amp_t   amp{};
    constexpr auto cfg = get_test_config();

    TEST_ESP_OK(amp.init(cfg));
    TEST_ESP_OK(amp.power_on(true));

    // 5 seconds of tone. Confirms send_audio_buf's byte count check against a
    // buffer that spans many DMA descriptor refills, not just one.
    constexpr size_t len = audio::amp::max98357a_t<>::SAMPLE_RATE_HZ * 5;
    auto*            buf = make_sine_buf(len);

    TEST_ESP_OK(amp.send_audio_buf({buf, len}));

    TEST_ESP_OK(amp.power_on(false));
    TEST_ESP_OK(amp.deinit());

    heap_caps_free(buf);
}

TEST_CASE("Left and right channel modes initializes and transmits", "[max98357a][i2s]") {
    // Left channel
    left_amp_t     left_amp{};
    constexpr auto left_cfg = get_test_config();

    TEST_ESP_OK(left_amp.init(left_cfg));
    TEST_ESP_OK(left_amp.power_on(true));

    constexpr size_t left_len = audio::amp::max98357a_t<>::SAMPLE_RATE_HZ * 0.1; // 100ms
    auto*            left_buf = make_sine_buf(left_len);

    TEST_ESP_OK(left_amp.send_audio_buf({left_buf, left_len}));

    TEST_ESP_OK(left_amp.power_on(false));
    TEST_ESP_OK(left_amp.deinit());

    heap_caps_free(left_buf);

    // Right channel
    right_amp_t    right_amp{};
    constexpr auto right_cfg = get_test_config();

    TEST_ESP_OK(right_amp.init(right_cfg));
    TEST_ESP_OK(right_amp.power_on(true));

    constexpr size_t right_len = audio::amp::max98357a_t<>::SAMPLE_RATE_HZ * 0.1; // 100ms
    auto*            right_buf = make_sine_buf(right_len);

    TEST_ESP_OK(right_amp.send_audio_buf({right_buf, right_len}));

    TEST_ESP_OK(right_amp.power_on(false));
    TEST_ESP_OK(right_amp.deinit());

    heap_caps_free(right_buf);
}

TEST_CASE("Timeout is surfaced when the buffer can't fully drain in time", "[max98357a][i2s]") {
    stereo_amp_t   amp{};
    constexpr auto cfg = get_test_config();

    TEST_ESP_OK(amp.init(cfg));
    TEST_ESP_OK(amp.power_on(true));

    // Large buffer, timeout too small: DMA can't move it all in time, so
    // send_audio_buf's byte count check should catch the short write and
    // return ESP_ERR_TIMEOUT rather than reporting success on a partial send.

    constexpr size_t len = audio::amp::max98357a_t<>::SAMPLE_RATE_HZ * 5; // 5s of data
    auto*            buf = make_sine_buf(len);

    const auto ret = amp.send_audio_buf({buf, len}, 4); // Timeout of 4s
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, ret);

    TEST_ESP_OK(amp.power_on(false));
    TEST_ESP_OK(amp.deinit());

    heap_caps_free(buf);
}
