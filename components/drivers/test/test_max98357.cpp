#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "unity.h"

#include "max98357.hpp"
#include "config.hpp"

#include <array>
#include <cstdint>
#include <cmath>
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

    // 100ms of a 440Hz sine wave at 48kHz, 32-bit signed samples, mono content
    // duplicated where the driver needs stereo framing.
    template<size_t N>
    std::array<int32_t, N> make_sine_buf() {
        std::array<int32_t, N> buf{};
        constexpr double       freq_hz     = 440.0;
        constexpr double       sample_rate = 48'000.0;
        for (size_t i = 0; i < N; i++) {
            const double sample = std::sin(2.0 * std::numbers::pi * freq_hz * static_cast<double>(i) / sample_rate);
            buf[i]              = static_cast<int32_t>(sample * static_cast<double>(INT32_MAX) * 0.5); // -6dB headroom
        }
        return buf;
    }

    using stereo_amp_t  = audio::amp::max98357a_t<audio::amp::gain_t::dB_9, audio::amp::mode_t::STEREO, true>;
    using left_amp_t    = audio::amp::max98357a_t<audio::amp::gain_t::dB_9, audio::amp::mode_t::LEFT_CHANNEL, true>;
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

TEST_CASE("send_audio_buf rejects calls before init or before power on", "[max98357a][i2s]") {
    stereo_amp_t   amp{};
    constexpr auto cfg  = get_test_config();
    const auto     sine = make_sine_buf<480>(); // 10ms @ 48kHz

    // Not initialized at all
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, amp.send_audio_buf(sine));

    TEST_ESP_OK(amp.init(cfg));

    // Initialized but not powered on
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, amp.send_audio_buf(sine));

    TEST_ESP_OK(amp.power_on(true));
    TEST_ESP_OK(amp.send_audio_buf(sine));

    TEST_ESP_OK(amp.power_on(false));
    TEST_ESP_OK(amp.deinit());
}

TEST_CASE("Stereo mode transmits a full-length buffer", "[max98357a][i2s][audible]") {
    stereo_amp_t   amp{};
    constexpr auto cfg = get_test_config();

    TEST_ESP_OK(amp.init(cfg));
    TEST_ESP_OK(amp.power_on(true));

    // 1 second of tone. Confirms send_audio_buf's byte-count check against a
    // buffer that spans many DMA descriptor refills, not just one.
    const auto buf = make_sine_buf<48'000>();
    TEST_ESP_OK(amp.send_audio_buf(buf));

    TEST_ESP_OK(amp.power_on(false));
    TEST_ESP_OK(amp.deinit());
}

// LEFT_CHANNEL takes the static_assert-guarded branch in power_on() that
// drives sd_pin as an output directly, instead of the input/bias-resistor
// path STEREO/RIGHT_CHANNEL take. This only proves the code path executes
// without error, not that the resistor divider on real hardware is correct
// for right/stereo modes -- that still needs the physical resistors stuffed
// and an ear (or a scope on the amp output) to confirm.
TEST_CASE("Left channel mode initializes and transmits", "[max98357a][i2s]") {
    left_amp_t     amp{};
    constexpr auto cfg = get_test_config();

    TEST_ESP_OK(amp.init(cfg));
    TEST_ESP_OK(amp.power_on(true));

    const auto buf = make_sine_buf<4800>(); // 100ms
    TEST_ESP_OK(amp.send_audio_buf(buf));

    TEST_ESP_OK(amp.power_on(false));
    TEST_ESP_OK(amp.deinit());
}

// use_gain_pin=false skips gpio_config/gpio_reset_pin on the gain pin
// entirely. Exercises that constexpr branch and confirms deinit doesn't
// try to reset a pin that was never configured.
TEST_CASE("Amp with no gain pin configured still inits and cleans up", "[max98357a][i2s]") {
    no_gain_pin_t  amp{};
    constexpr auto cfg = get_test_config();

    TEST_ESP_OK(amp.init(cfg));
    TEST_ESP_OK(amp.power_on(true));
    TEST_ESP_OK(amp.power_on(false));
    TEST_ESP_OK(amp.deinit());
}

TEST_CASE("Timeout is surfaced when the buffer can't fully drain in time", "[max98357a][i2s]") {
    stereo_amp_t   amp{};
    constexpr auto cfg = get_test_config();

    TEST_ESP_OK(amp.init(cfg));
    TEST_ESP_OK(amp.power_on(true));

    // Large buffer, near-zero timeout: DMA can't move it all in time, so
    // send_audio_buf's byte-count check should catch the short write and
    // return ESP_ERR_TIMEOUT rather than reporting success on a partial send.
    // Flaky by nature -- depends on DMA throughput vs. timeout margin, adjust
    // buffer size/timeout if this is inconsistent on your hardware.
    const auto buf = make_sine_buf<48'000>(); // 1 second of audio
    const auto ret = amp.send_audio_buf(buf, 1);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, ret);

    TEST_ESP_OK(amp.power_on(false));
    TEST_ESP_OK(amp.deinit());
}
