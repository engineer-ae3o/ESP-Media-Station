#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "unity.h"

#include "driver/spi_master.h"

#include "utils.hpp"
#include "config.hpp"
#include "xpt2046.hpp"
#include "ili9341.hpp"

#include <array>
#include <format>

namespace {

    // Allowed error (in pixels) between where the tester was told to press
    // and where the driver reports the press. ADC jitter + panel/finger contact
    // area + calibration slop all live in here.
    constexpr uint16_t TOLERANCE_PX = 15;

    // How long to block waiting for a human to press the screen before failing
    // the test outright instead of hanging the test runner forever.
    constexpr uint32_t TOUCH_TIMEOUT_MS = 15'000;

    consteval auto get_test_config() {
        return touch::config_t{
            .spi_host           = config::XPT_SPI_BUS,
            .clock_freq_hz      = config::XPT_SPI_CLK_SPEED_HZ,
            .queue_length       = 10,
            .cs_pin             = config::XPT_CS_PIN,
            .irq_pin            = config::XPT_IRQ_PIN,
            .screen_pixel_len_x = disp::ili9341_t::MAX_WIDTH,
            .screen_pixel_len_y = disp::ili9341_t::MAX_HEIGHT,
        };
    }

    struct spi_test_fixture_t {
        spi_test_fixture_t() {
            constexpr utils::spi_bus_config_t bus_config = {
                .bus            = config::XPT_SPI_BUS,
                .flags          = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS,
                .max_trans_size = config::XPT_MAX_TRANS_SIZE,
                .mosi_pin       = config::XPT_MOSI_PIN,
                .miso_pin       = config::XPT_MISO_PIN,
                .sclk_pin       = config::XPT_CLK_PIN,
            };
            TEST_ESP_OK(utils::init_spi_bus(bus_config));
        }

        ~spi_test_fixture_t() {
            TEST_ESP_OK(spi_bus_free(config::XPT_SPI_BUS));
        }

        spi_test_fixture_t(const spi_test_fixture_t&)            = delete;
        spi_test_fixture_t& operator=(const spi_test_fixture_t&) = delete;
        spi_test_fixture_t(spi_test_fixture_t&&)                 = delete;
        spi_test_fixture_t& operator=(spi_test_fixture_t&&)      = delete;
    };

    // Blocks until a coord_t shows up on the queue or the timeout expires.
    // Fails the test on timeout rather than hanging.
    [[nodiscard]] touch::coord_t wait_for_press(QueueHandle_t queue) {
        touch::coord_t coord{};
        const auto     received = xQueueReceive(queue, &coord, pdMS_TO_TICKS(TOUCH_TIMEOUT_MS));
        TEST_ASSERT_EQUAL_MESSAGE(pdTRUE, received, "Timed out waiting for a touch press");
        return coord;
    }

    void assert_near(const touch::coord_t& expected, const touch::coord_t& actual) {
        std::array<char, 128> msg{};
        (void)std::format_to(msg.begin(), "Expected near ({}, {}), got ({}, {})", expected.x, expected.y, actual.x, actual.y);
        TEST_ASSERT_UINT16_WITHIN_MESSAGE(TOLERANCE_PX, expected.x, actual.x, msg.data());
        TEST_ASSERT_UINT16_WITHIN_MESSAGE(TOLERANCE_PX, expected.y, actual.y, msg.data());
    }

} // namespace

TEST_CASE("Initialization and deinitialization", "[xpt2046][spi]") {
    [[maybe_unused]] spi_test_fixture_t spi_bus{};

    touch::xpt2046_t<> xpt{};
    constexpr auto     cfg = get_test_config();

    // Queue handle should be null before init
    TEST_ASSERT_NULL(xpt.get_event_queue());

    // Test valid init
    TEST_ESP_OK(xpt.init(cfg));
    TEST_ASSERT_NOT_NULL(xpt.get_event_queue());

    // Test double init (should fail with invalid state)
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, xpt.init(cfg));

    // Test valid deinit
    TEST_ESP_OK(xpt.deinit());
    TEST_ASSERT_NULL(xpt.get_event_queue());

    // Test double deinit (should fail with invalid state)
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, xpt.deinit());
}

TEST_CASE("No spurious touch events when idle", "[xpt2046][spi][manual]") {
    [[maybe_unused]] spi_test_fixture_t spi_bus{};

    touch::xpt2046_t<> xpt{};
    constexpr auto     cfg = get_test_config();

    TEST_ESP_OK(xpt.init(cfg));

    ESP_LOGI("XPT_TEST", ">>> Do NOT touch the screen for the next 5 seconds...");

    touch::coord_t coord{};
    const auto     ret = xQueueReceive(xpt.get_event_queue(), &coord, pdMS_TO_TICKS(5000));
    TEST_ASSERT_EQUAL_MESSAGE(pdFALSE, ret, "Received an unexpected touch event while idle (supposedly)");

    TEST_ESP_OK(xpt.deinit());
}

TEST_CASE("Touch detection maps to expected screen coordinates", "[xpt2046][spi][manual]") {
    [[maybe_unused]] spi_test_fixture_t spi_bus{};

    touch::xpt2046_t<> xpt{};
    constexpr auto     cfg = get_test_config();

    TEST_ESP_OK(xpt.init(cfg));
    auto* event_queue = xpt.get_event_queue();
    TEST_ASSERT_NOT_NULL(event_queue);

    struct target_t {
        uint16_t    x{}, y{};
        const char* label{};
    };

    constexpr std::array<target_t, 5> targets = {{
        {0, 0, "top left corner"},
        {cfg.screen_pixel_len_x - 1, 0, "top right corner"},
        {0, cfg.screen_pixel_len_y - 1, "bottom left corner"},
        {cfg.screen_pixel_len_x - 1, cfg.screen_pixel_len_y - 1, "bottom right corner"},
        {cfg.screen_pixel_len_x / 2, cfg.screen_pixel_len_y / 2, "center"},
    }};

    for (const auto& target : targets) {
        xQueueReset(event_queue); // Drop any stale event from a prior target
        ESP_LOGI("XPT_TEST", ">>> Press the %s of the screen ~(%u, %u) now...", target.label, target.x, target.y);

        const auto coord = wait_for_press(event_queue);
        assert_near({target.x, target.y}, coord);
    }

    TEST_ESP_OK(xpt.deinit());
}

TEST_CASE("Multiple sequential presses are all captured", "[xpt2046][spi][manual]") {
    [[maybe_unused]] spi_test_fixture_t spi_bus{};

    touch::xpt2046_t<> xpt{};
    constexpr auto     cfg = get_test_config();

    TEST_ESP_OK(xpt.init(cfg));
    auto* event_queue = xpt.get_event_queue();
    TEST_ASSERT_NOT_NULL(event_queue);
    xQueueReset(event_queue);

    constexpr int press_count = 5;
    ESP_LOGI("XPT_TEST", ">>> Press anywhere on the screen %d times, pausing briefly between each press", press_count);

    for (int i = 0; i < press_count; i++) {
        touch::coord_t coord{};
        const auto     received = xQueueReceive(event_queue, &coord, pdMS_TO_TICKS(TOUCH_TIMEOUT_MS));

        std::array<char, 64> msg{};
        (void)std::format_to(msg.begin(), "Timed out waiting for press {} of {}", i + 1, press_count);
        TEST_ASSERT_EQUAL_MESSAGE(pdTRUE, received, msg.data());

        TEST_ASSERT_TRUE(coord.x < cfg.screen_pixel_len_x);
        TEST_ASSERT_TRUE(coord.y < cfg.screen_pixel_len_y);
    }

    TEST_ESP_OK(xpt.deinit());
}
