#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "unity.h"

#include "config.hpp"
#include "inmp441.hpp"

#include <atomic>
#include <cstdint>

namespace {

    consteval auto get_test_config(bool use_right_chan = false) {
        return audio::mic::config_t{
            .use_right_chan = use_right_chan,
            .error_cb       = nullptr,
            .chip_en_pin    = config::INMP_CHIPEN_PIN,
            .bclk_pin       = config::INMP_BCLK_PIN,
            .din_pin        = config::INMP_DIN_PIN,
            .l_r_pin        = config::INMP_L_R_PIN,
            .ws_pin         = config::INMP_WS_PIN,
        };
    }

    // The streaming task fills buffers on its own schedule; poll instead of
    // assuming a buffer is ready immediately after start_stream().
    [[nodiscard]] std::expected<std::span<const int32_t, audio::mic::inmp441_t::RECV_BUF_SIZE_ELEMENTS>, esp_err_t>
    wait_for_filled_buffer(audio::mic::inmp441_t& mic, uint32_t timeout_ms = 2000) {
        constexpr uint32_t poll_interval_ms = 10;
        uint32_t           waited_ms{};

        while (waited_ms < timeout_ms) {
            auto result = mic.get_filled_buffer();
            if (result.has_value()) {
                return result;
            }
            vTaskDelay(pdMS_TO_TICKS(poll_interval_ms));
            waited_ms += poll_interval_ms;
        }

        return std::unexpected(ESP_ERR_TIMEOUT);
    }

} // namespace

TEST_CASE("Initialization and deinitialization", "[inmp441][i2s]") {
    audio::mic::inmp441_t mic{};
    const auto            cfg = get_test_config();

    TEST_ESP_OK(mic.init(cfg));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, mic.init(cfg));

    TEST_ESP_OK(mic.deinit());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, mic.deinit());
}

TEST_CASE("Destructor cleans up correctly while streaming", "[inmp441][i2s]") {
    // The destructor path goes through the same cleanup()/stream_task
    // notify handshake as deinit(), but nothing here calls deinit()
    // explicitly. This is the part of the driver most likely to deadlock
    // if the shutdown synchronization is wrong, so it gets its own test
    // rather than relying on every other test's teardown to catch it.
    const auto cfg = get_test_config();
    {
        audio::mic::inmp441_t mic{};
        TEST_ESP_OK(mic.init(cfg));
        TEST_ESP_OK(mic.start_stream());
        vTaskDelay(pdMS_TO_TICKS(20)); // Let the state machine get into a read
    } // ~inmp441_t() runs here; test hangs if cleanup's task notify handshake is broken

    TEST_PASS(); // Reaching this line at all is the assertion
}

TEST_CASE("Enable/disable rejects invalid transitions", "[inmp441][i2s]") {
    audio::mic::inmp441_t mic{};
    const auto            cfg = get_test_config();

    // Not initialized yet
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, mic.enable(false));

    TEST_ESP_OK(mic.init(cfg)); // init() leaves the mic enabled

    // Already enabled. Should return invalid state
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, mic.enable(true));

    // Test double disabling
    TEST_ESP_OK(mic.enable(false));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, mic.enable(false));

    // Enable on final time before deinitializing
    TEST_ESP_OK(mic.enable(true));
    TEST_ESP_OK(mic.deinit());
}

TEST_CASE("Cannot disable while streaming", "[inmp441][i2s]") {
    audio::mic::inmp441_t mic{};
    const auto            cfg = get_test_config();

    TEST_ESP_OK(mic.init(cfg));
    TEST_ESP_OK(mic.start_stream());

    // Cannot disable while streaming is still on going
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, mic.enable(false));

    TEST_ESP_OK(mic.stop_stream());
    TEST_ESP_OK(mic.enable(false));
    TEST_ESP_OK(mic.deinit());
}

TEST_CASE("start_stream/stop_stream rejects invalid transitions", "[inmp441][i2s]") {
    audio::mic::inmp441_t mic{};
    const auto            cfg = get_test_config();

    // Not initialized
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, mic.start_stream());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, mic.stop_stream());

    TEST_ESP_OK(mic.init(cfg));

    // Not streaming yet
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, mic.stop_stream());

    TEST_ESP_OK(mic.start_stream());
    // Already streaming
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, mic.start_stream());

    TEST_ESP_OK(mic.stop_stream());
    // Already stopped
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, mic.stop_stream());

    TEST_ESP_OK(mic.deinit());
}

TEST_CASE("get_filled_buffer and return_buffer reject bad state and bad pointers", "[inmp441][i2s]") {
    audio::mic::inmp441_t mic{};
    const auto            cfg = get_test_config();

    // Not initialized
    {
        auto result = mic.get_filled_buffer();
        TEST_ASSERT_FALSE(result.has_value());
        TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, result.error());
    }
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, mic.return_buffer(nullptr));

    TEST_ESP_OK(mic.init(cfg));

    // Initialized, streaming not started: nothing filled yet
    {
        auto result = mic.get_filled_buffer();
        TEST_ASSERT_FALSE(result.has_value());
        TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, result.error());
    }

    // Garbage pointer, neither buffer
    int32_t bogus{};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, mic.return_buffer(&bogus));

    TEST_ESP_OK(mic.deinit());
}

TEST_CASE("Streaming produces correctly sized buffers", "[inmp441][i2s]") {
    audio::mic::inmp441_t mic{};
    const auto            cfg = get_test_config();

    TEST_ESP_OK(mic.init(cfg));
    TEST_ESP_OK(mic.start_stream());

    auto result = wait_for_filled_buffer(mic);
    TEST_ASSERT_TRUE_MESSAGE(result.has_value(), "No buffer filled within timeout");

    const auto buf = result.value();
    TEST_ASSERT_EQUAL(audio::mic::inmp441_t::RECV_BUF_SIZE_ELEMENTS, buf.size());
    TEST_ASSERT_EQUAL(audio::mic::inmp441_t::RECV_BUF_SIZE_BYTES, buf.size_bytes());

    TEST_ESP_OK(mic.return_buffer(buf.data()));

    TEST_ESP_OK(mic.stop_stream());
    TEST_ESP_OK(mic.deinit());
}

TEST_CASE("Double buffering alternates between the two buffers", "[inmp441][i2s]") {
    audio::mic::inmp441_t mic{};
    const auto            cfg = get_test_config();

    TEST_ESP_OK(mic.init(cfg));
    TEST_ESP_OK(mic.start_stream());

    auto first = wait_for_filled_buffer(mic);
    TEST_ASSERT_TRUE_MESSAGE(first.has_value(), "First buffer never filled");

    const auto* first_ptr = first.value().data();
    TEST_ESP_OK(mic.return_buffer(first_ptr));

    auto second = wait_for_filled_buffer(mic);
    TEST_ASSERT_TRUE_MESSAGE(second.has_value(), "Second buffer never filled");

    const auto* second_ptr = second.value().data();
    TEST_ESP_OK(mic.return_buffer(second_ptr));

    // Confirms the state machine actually swaps buf1/buf2
    // instead of re-filling and re-returning the same one.
    TEST_ASSERT_NOT_EQUAL(first_ptr, second_ptr);

    TEST_ESP_OK(mic.stop_stream());
    TEST_ESP_OK(mic.deinit());
}

TEST_CASE("Returning a buffer twice fails the second time", "[inmp441][i2s]") {
    audio::mic::inmp441_t mic{};
    const auto            cfg = get_test_config();

    TEST_ESP_OK(mic.init(cfg));
    TEST_ESP_OK(mic.start_stream());

    auto result = wait_for_filled_buffer(mic);
    TEST_ASSERT_TRUE_MESSAGE(result.has_value(), "No buffer filled within timeout");
    const auto* ptr = result.value().data();

    TEST_ESP_OK(mic.return_buffer(ptr));

    // The buffer has been returned, so re-returning
    // it immediately should fail an invalid argument
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, mic.return_buffer(ptr));

    TEST_ESP_OK(mic.stop_stream());
    TEST_ESP_OK(mic.deinit());
}

TEST_CASE("Right channel selection initializes without error", "[inmp441][i2s]") {
    // This only proves the L/R gpio and slot_mask config path executes.
    // Confirming the mic is actually wired to the right channel of the bus
    // would require more setup, and is beyond the scope of this test.
    audio::mic::inmp441_t mic{};
    constexpr auto        cfg = get_test_config(/* use_right_chan = */ true);

    TEST_ESP_OK(mic.init(cfg));
    TEST_ESP_OK(mic.start_stream());

    auto result = wait_for_filled_buffer(mic);
    TEST_ASSERT_TRUE_MESSAGE(result.has_value(), "No buffer filled within timeout");
    TEST_ESP_OK(mic.return_buffer(result.value().data()));

    TEST_ESP_OK(mic.stop_stream());
    TEST_ESP_OK(mic.deinit());
}
