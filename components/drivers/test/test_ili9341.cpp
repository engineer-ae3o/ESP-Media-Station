#include "driver/spi_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "unity.h"

#include "driver/spi_master.h"
#include "esp_heap_caps.h"

#include "ili9341.hpp"
#include "config.hpp"
#include "utils.hpp"

#include <array>
#include <cstdint>

namespace {

    consteval auto get_test_config() {
        return display::config_t{
            .spi_host           = config::ILI_SPI_BUS,
            .spi_clock_speed_hz = config::ILI_SPI_CLK_SPEED_HZ,
            .led_pin            = config::ILI_LED_PIN,
            .rst_pin            = config::ILI_RST_PIN,
            .cs_pin             = config::ILI_CS_PIN,
            .dc_pin             = config::ILI_DC_PIN,
            .rotation           = 0,
            .led_ledc_timer     = LEDC_TIMER_0,
            .led_ledc_channel   = LEDC_CHANNEL_0,
        };
    }

    struct spi_test_fixture_t {
        spi_test_fixture_t() {
            constexpr utils::spi_bus_config_t bus_config = {
                .bus            = config::ILI_SPI_BUS,
                .flags          = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_IOMUX_PINS,
                .max_trans_size = 32 * 1024, // Hardware limit
                .mosi_pin       = config::ILI_MOSI_PIN,
                .miso_pin       = GPIO_NUM_NC,
                .sclk_pin       = config::ILI_CLK_PIN,
            };
            TEST_ESP_OK(utils::init_spi_bus(bus_config));
        }

        ~spi_test_fixture_t() {
            TEST_ESP_OK(spi_bus_free(config::ILI_SPI_BUS));
        }

        spi_test_fixture_t(const spi_test_fixture_t&)            = delete;
        spi_test_fixture_t& operator=(const spi_test_fixture_t&) = delete;
        spi_test_fixture_t(spi_test_fixture_t&&)                 = delete;
        spi_test_fixture_t& operator=(spi_test_fixture_t&&)      = delete;
    };

} // namespace

TEST_CASE("Initialization and deinitialization", "[ili9341][spi]") {
    [[maybe_unused]] spi_test_fixture_t spi_bus{};

    display::ili9341_t display{};
    constexpr auto     cfg = get_test_config();

    // Requires LEDC timer setup first
    TEST_ESP_OK(display::ili9341_t::init_ledc_timer(cfg.led_ledc_timer, true));

    // Test valid init
    TEST_ESP_OK(display.init(cfg));

    // Test double init (Should fail with invalid state)
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, display.init(cfg));

    // Test valid deinit
    TEST_ESP_OK(display.deinit());

    // Test double deinit (Should fail with invalid state)
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, display.deinit());

    // Cleanup timer
    TEST_ESP_OK(display::ili9341_t::init_ledc_timer(cfg.led_ledc_timer, false));
}

TEST_CASE("Flush out of bounds", "[ili9341][spi]") {
    [[maybe_unused]] spi_test_fixture_t spi_bus{};

    display::ili9341_t display{};
    constexpr auto     cfg = get_test_config();

    TEST_ESP_OK(display::ili9341_t::init_ledc_timer(cfg.led_ledc_timer, true));
    TEST_ESP_OK(display.init(cfg));
    TEST_ESP_OK(display.set_brightness());

    std::array<uint16_t, 1024> dummy_data{};
    dummy_data.fill(0xF800);

    // Test with invalid data
    display::coord_t valid_coord{0, 0, 9, 0};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, display.flush(valid_coord, {}));

    // Test out of bounds X
    display::coord_t oob_x{display::ili9341_t::MAX_WIDTH, 0, display::ili9341_t::MAX_WIDTH + 5, 0};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, display.flush(oob_x, dummy_data));

    // Test out of bounds Y
    display::coord_t oob_y{0, display::ili9341_t::MAX_HEIGHT, 5, display::ili9341_t::MAX_HEIGHT + 5};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, display.flush(oob_y, dummy_data));

    // Test reversed coordinates (x1 > x2)
    display::coord_t rev_x{10, 0, 5, 0};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, display.flush(rev_x, dummy_data));

    TEST_ESP_OK(display.set_brightness(0));

    TEST_ESP_OK(display.deinit());
    TEST_ESP_OK(display::ili9341_t::init_ledc_timer(cfg.led_ledc_timer, false));
}

TEST_CASE("Flush Valid Data from PSRAM", "[ili9341][spi][psram]") {
    [[maybe_unused]] spi_test_fixture_t spi_bus{};

    display::ili9341_t display{};
    constexpr auto     cfg = get_test_config();

    TEST_ESP_OK(display::ili9341_t::init_ledc_timer(cfg.led_ledc_timer, true));
    TEST_ESP_OK(display.init(cfg));
    TEST_ESP_OK(display.set_brightness());

    // Allocate a full framebuffer buffer
    constexpr size_t pixel_count = display::ili9341_t::MAX_HEIGHT * display::ili9341_t::MAX_WIDTH;
    auto*            buf = static_cast<uint16_t*>(heap_caps_malloc(pixel_count * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA));
    TEST_ASSERT_NOT_NULL(buf);

    // Fill with red (RGB565)
    constexpr auto color = __builtin_bswap16(0xF800);
    for (size_t i = 0; i < pixel_count; i++) {
        buf[i] = color;
    }

    // Execute flush
    TEST_ESP_OK(display.flush({0, 0, 99, 99}, {buf, pixel_count}));
    vTaskDelay(pdMS_TO_TICKS(3000));

    heap_caps_free(buf);

    TEST_ESP_OK(display.set_brightness(0));

    TEST_ESP_OK(display.deinit());
    TEST_ESP_OK(display::ili9341_t::init_ledc_timer(cfg.led_ledc_timer, false));
}

TEST_CASE("Screen Fill", "[ili9341][fill_screen]") {
    [[maybe_unused]] spi_test_fixture_t spi_bus{};

    display::ili9341_t display{};
    constexpr auto     cfg = get_test_config();

    TEST_ESP_OK(display::ili9341_t::init_ledc_timer(cfg.led_ledc_timer, true));
    TEST_ESP_OK(display.init(cfg));
    TEST_ESP_OK(display.set_brightness());

    // Set screen to Green (little-endian conversion handled internally by set_screen)
    TEST_ESP_OK(display.set_screen(0x07E0));
    vTaskDelay(pdMS_TO_TICKS(3000));

    TEST_ESP_OK(display.set_brightness(0));

    TEST_ESP_OK(display.deinit());
    TEST_ESP_OK(display::ili9341_t::init_ledc_timer(cfg.led_ledc_timer, false));
}

TEST_CASE("Backlight Brightness Control", "[ili9341][ledc]") {
    [[maybe_unused]] spi_test_fixture_t spi_bus{};

    display::ili9341_t display{};
    constexpr auto     cfg = get_test_config();

    TEST_ESP_OK(display::ili9341_t::init_ledc_timer(cfg.led_ledc_timer, true));
    TEST_ESP_OK(display.init(cfg));

    // Arbitrary color
    TEST_ESP_OK(display.set_screen(0x2AD4, true));

    // Sweep brightness levels
    for (uint16_t level{}; level <= UINT8_MAX; level++) {
        TEST_ESP_OK(display.set_brightness(static_cast<uint8_t>(level)));
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    TEST_ESP_OK(display.set_brightness(0));

    // Attempting to set brightness without initialized driver should fail
    TEST_ESP_OK(display.deinit());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, display.set_brightness(128));

    TEST_ESP_OK(display::ili9341_t::init_ledc_timer(cfg.led_ledc_timer, false));
}
