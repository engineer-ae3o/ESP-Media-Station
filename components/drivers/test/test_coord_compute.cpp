#include "unity.h"

#include "coord_compute.hpp"

#include <array>
#include <cstdint>

namespace {

    constexpr uint16_t SCREEN_W = 240;
    constexpr uint16_t SCREEN_H = 320;

    constexpr uint16_t MAX_SAMPLE_LEN = 10;
    constexpr uint16_t TRIM_COUNT     = 2;

    using samples_t = std::array<uint16_t, MAX_SAMPLE_LEN>;

    consteval samples_t make_filled_array(uint16_t value) {
        samples_t arr{};
        arr.fill(value);
        return arr;
    }

} // namespace

TEST_CASE("All identical samples produce that exact value", "[touch_math]") {

    constexpr samples_t x_samples = make_filled_array(2000);
    constexpr samples_t y_samples = make_filled_array(2000);

    constexpr auto coord = touch::compute_coord<MAX_SAMPLE_LEN, TRIM_COUNT>(x_samples, y_samples, SCREEN_W, SCREEN_H);

    TEST_ASSERT_EQUAL_UINT16(115, coord.x);
    TEST_ASSERT_EQUAL_UINT16(153, coord.y);

    // For good measure since its compile time
    static_assert(coord.x == 115);
    static_assert(coord.y == 153);
}

TEST_CASE("Outliers beyond the trim window are excluded from the average", "[touch_math]") {
    // 2 lowest and 2 highest get trimmed (TRIM_COUNT = 2). Put deliberately
    // extreme values there; they must not move the result at all.
    constexpr samples_t x_samples = {0, 0, 2000, 2000, 2000, 2000, 2000, 2000, 4095, 4095};
    constexpr samples_t y_samples = {0, 0, 2000, 2000, 2000, 2000, 2000, 2000, 4095, 4095};

    constexpr auto coord = touch::compute_coord<MAX_SAMPLE_LEN, TRIM_COUNT>(x_samples, y_samples, SCREEN_W, SCREEN_H);

    // Get an independent "clean" average to compare against, which should be identical to the above result.
    constexpr samples_t clean_x_samples = make_filled_array(2000);
    constexpr samples_t clean_y_samples = make_filled_array(2000);

    constexpr auto clean_coord = touch::compute_coord<MAX_SAMPLE_LEN, TRIM_COUNT>(clean_x_samples, clean_y_samples, SCREEN_W, SCREEN_H);

    TEST_ASSERT_EQUAL_UINT16(clean_coord.x, coord.x);
    TEST_ASSERT_EQUAL_UINT16(clean_coord.y, coord.y);

    static_assert(clean_coord.x == coord.x);
    static_assert(clean_coord.y == coord.y);
}

TEST_CASE("Rounding: average that lands on >= .5 rounds up, not truncates", "[touch_math]") {

    // Trimmed middle 6: {2000, 2000, 2001, 2001, 2001, 2001};
    // sum = 12004; 12004 / 6 = 2000.666. Should round to 2001
    constexpr samples_t rounded_x_samples = {0, 0, 2000, 2000, 2001, 2001, 2001, 2001, 4095, 4095};
    constexpr samples_t rounded_y_samples = {0, 0, 2000, 2000, 2001, 2001, 2001, 2001, 4095, 4095};

    constexpr auto rounded_coord =
        touch::compute_coord<MAX_SAMPLE_LEN, TRIM_COUNT>(rounded_x_samples, rounded_y_samples, SCREEN_W, SCREEN_H);

    constexpr samples_t x_samples = make_filled_array(2001);
    constexpr samples_t y_samples = make_filled_array(2001);

    constexpr auto coord = touch::compute_coord<MAX_SAMPLE_LEN, TRIM_COUNT>(x_samples, y_samples, SCREEN_W, SCREEN_H);

    TEST_ASSERT_EQUAL_UINT16(rounded_coord.x, coord.x);
    TEST_ASSERT_EQUAL_UINT16(rounded_coord.y, coord.y);

    static_assert(rounded_coord.x == coord.x);
    static_assert(rounded_coord.y == coord.y);
}

TEST_CASE("Minimum calibration value maps to screen origin", "[touch_math]") {

    constexpr samples_t x_samples = make_filled_array(touch::calibration_data_t::x_min);
    constexpr samples_t y_samples = make_filled_array(touch::calibration_data_t::y_min);

    constexpr auto coord = touch::compute_coord<MAX_SAMPLE_LEN, TRIM_COUNT>(x_samples, y_samples, SCREEN_W, SCREEN_H);

    TEST_ASSERT_EQUAL_UINT16(0, coord.x);
    TEST_ASSERT_EQUAL_UINT16(0, coord.y);

    static_assert(coord.x == 0);
    static_assert(coord.y == 0);
}

TEST_CASE("Maximum calibration value maps to screen far edge", "[touch_math]") {

    constexpr samples_t x_samples = make_filled_array(touch::calibration_data_t::x_max);
    constexpr samples_t y_samples = make_filled_array(touch::calibration_data_t::y_max);

    constexpr auto coord = touch::compute_coord<MAX_SAMPLE_LEN, TRIM_COUNT>(x_samples, y_samples, SCREEN_W, SCREEN_H);

    TEST_ASSERT_EQUAL_UINT16(SCREEN_W - 1, coord.x);
    TEST_ASSERT_EQUAL_UINT16(SCREEN_H - 1, coord.y);

    static_assert(coord.x == SCREEN_W - 1);
    static_assert(coord.y == SCREEN_H - 1);
}

TEST_CASE("Values beyond calibration bounds get clamped, not extrapolated", "[touch_math]") {

    // Below x_min/y_min entirely
    constexpr samples_t below_min_x_samples = make_filled_array(0);
    constexpr samples_t below_min_y_samples = make_filled_array(0);

    constexpr auto below = touch::compute_coord<MAX_SAMPLE_LEN, TRIM_COUNT>(below_min_x_samples, below_min_y_samples, SCREEN_W, SCREEN_H);

    TEST_ASSERT_EQUAL_UINT16(0, below.x);
    TEST_ASSERT_EQUAL_UINT16(0, below.y);

    static_assert(below.x == 0);
    static_assert(below.y == 0);

    // Above x_max/y_max entirely (ADC ceiling is 4095, well above x_max = 3750)
    constexpr samples_t above_max_x_samples = make_filled_array(4095);
    constexpr samples_t above_max_y_samples = make_filled_array(4095);

    constexpr auto above = touch::compute_coord<MAX_SAMPLE_LEN, TRIM_COUNT>(above_max_x_samples, above_max_y_samples, SCREEN_W, SCREEN_H);

    TEST_ASSERT_EQUAL_UINT16(SCREEN_W - 1, above.x);
    TEST_ASSERT_EQUAL_UINT16(SCREEN_H - 1, above.y);

    static_assert(above.x == SCREEN_W - 1);
    static_assert(above.y == SCREEN_H - 1);
}

TEST_CASE("X and Y are computed independently", "[touch_math]") {

    constexpr samples_t x_samples = make_filled_array(touch::calibration_data_t::x_min);
    constexpr samples_t y_samples = make_filled_array(touch::calibration_data_t::y_max);

    constexpr auto coord = touch::compute_coord<MAX_SAMPLE_LEN, TRIM_COUNT>(x_samples, y_samples, SCREEN_W, SCREEN_H);

    TEST_ASSERT_EQUAL_UINT16(0, coord.x);
    TEST_ASSERT_EQUAL_UINT16(SCREEN_H - 1, coord.y);

    static_assert(coord.x == 0);
    static_assert(coord.y == SCREEN_H - 1);
}
