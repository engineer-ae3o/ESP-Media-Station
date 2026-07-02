// Runs anywhere with a C++20 compiler + Unity. No ESP-IDF, no FreeRTOS, no
// hardware. Compile on host, e.g.:
//   g++ -std=c++20 test_touch_math.cpp unity.c -I. -o test_touch_math && ./test_touch_math
// (or wire into idf.py's linux target if you want it in the same test runner
// as the hardware tests.)

#include "unity.h"
#include "coord_compute.hpp"

#include <array>
#include <cstdint>

namespace {

    constexpr uint16_t SCREEN_W = 240;
    constexpr uint16_t SCREEN_H = 320;

    using samples_t = std::array<uint16_t, 10>;

} // namespace

TEST_CASE("All identical samples produce that exact value", "[coord_compute]") {
    samples_t x_samples{};
    samples_t y_samples{};
    x_samples.fill(2000);
    y_samples.fill(2000);

    const auto coord = touch::compute_coord<10, 2>(x_samples, y_samples, SCREEN_W, SCREEN_H);

    // Verified by compiling touch_math.hpp standalone against these inputs, not
    // hand-computed: interpolation step still truncates (unchanged from original).
    TEST_ASSERT_EQUAL_UINT16(115, coord.x);
    TEST_ASSERT_EQUAL_UINT16(153, coord.y);
}

TEST_CASE("Outliers beyond the trim window are excluded from the average", "[touch_math]") {
    // 2 lowest and 2 highest get trimmed (TRIM_COUNT=2). Put deliberately
    // extreme values there; they must not move the result at all.
    samples_t x_samples = {0, 0, 2000, 2000, 2000, 2000, 2000, 2000, 4095, 4095};
    samples_t y_samples = {0, 0, 2000, 2000, 2000, 2000, 2000, 2000, 4095, 4095};

    const auto coord = touch::compute_coord<10, 2>(x_samples, y_samples, SCREEN_W, SCREEN_H);
    samples_t  clean_x{};
    samples_t  clean_y{};
    clean_x.fill(2000);
    clean_y.fill(2000);
    const auto clean_coord = touch::compute_coord<10, 2>(clean_x, clean_y, SCREEN_W, SCREEN_H);

    TEST_ASSERT_EQUAL_UINT16(clean_coord.x, coord.x);
    TEST_ASSERT_EQUAL_UINT16(clean_coord.y, coord.y);
}

TEST_CASE("Rounding: average that lands on .5 rounds up, not truncates", "[touch_math]") {
    // 6 samples survive trimming. Sum = 12001 -> 12001/6 = 2000.1666, average
    // via (sum + n/2)/n should round to 2000, not floor to 1999 or 2000.5.
    // Pick values that sum to something whose /6 has a clean .5 boundary instead:
    // sum = 12003 -> 2000.5 exactly. Rounding-to-nearest should give 2001 (or 2000,
    // implementation picks round-half-up here); truncation would give 2000.
    samples_t x_samples = {0, 0, 2000, 2000, 2001, 2001, 2001, 2001, 4095, 4095};
    samples_t y_samples = {0, 0, 2000, 2000, 2001, 2001, 2001, 2001, 4095, 4095};
    // Trimmed middle 6: {2000, 2000, 2001, 2001, 2001, 2001} sum=12004, /6 = 2000.666 -> rounds to 2001

    const auto coord = touch::compute_coord<10, 2>(x_samples, y_samples, SCREEN_W, SCREEN_H);

    samples_t rounded_x{};
    samples_t rounded_y{};
    rounded_x.fill(2001);
    rounded_y.fill(2001);
    const auto rounded_coord = touch::compute_coord<10, 2>(rounded_x, rounded_y, SCREEN_W, SCREEN_H);

    TEST_ASSERT_EQUAL_UINT16(rounded_coord.x, coord.x);
    TEST_ASSERT_EQUAL_UINT16(rounded_coord.y, coord.y);
}

TEST_CASE("Minimum calibration value maps to screen origin", "[touch_math]") {
    samples_t x_samples{};
    samples_t y_samples{};
    x_samples.fill(touch::calibration_data_t::x_min);
    y_samples.fill(touch::calibration_data_t::y_min);

    const auto coord = touch::compute_coord<10, 2>(x_samples, y_samples, SCREEN_W, SCREEN_H);

    TEST_ASSERT_EQUAL_UINT16(0, coord.x);
    TEST_ASSERT_EQUAL_UINT16(0, coord.y);
}

TEST_CASE("Maximum calibration value maps to screen far edge", "[touch_math]") {
    samples_t x_samples{};
    samples_t y_samples{};
    x_samples.fill(touch::calibration_data_t::x_max);
    y_samples.fill(touch::calibration_data_t::y_max);

    const auto coord = touch::compute_coord<10, 2>(x_samples, y_samples, SCREEN_W, SCREEN_H);

    TEST_ASSERT_EQUAL_UINT16(SCREEN_W - 1, coord.x);
    TEST_ASSERT_EQUAL_UINT16(SCREEN_H - 1, coord.y);
}

TEST_CASE("Values beyond calibration bounds get clamped, not extrapolated", "[touch_math]") {
    samples_t x_samples{};
    samples_t y_samples{};
    // Below x_min/y_min entirely
    x_samples.fill(0);
    y_samples.fill(0);

    const auto below = touch::compute_coord<10, 2>(x_samples, y_samples, SCREEN_W, SCREEN_H);
    TEST_ASSERT_EQUAL_UINT16(0, below.x);
    TEST_ASSERT_EQUAL_UINT16(0, below.y);

    // Above x_max/y_max entirely (ADC ceiling is 4095, well above x_max=3750)
    x_samples.fill(4095);
    y_samples.fill(4095);

    const auto above = touch::compute_coord<10, 2>(x_samples, y_samples, SCREEN_W, SCREEN_H);
    TEST_ASSERT_EQUAL_UINT16(SCREEN_W - 1, above.x);
    TEST_ASSERT_EQUAL_UINT16(SCREEN_H - 1, above.y);
}

TEST_CASE("X and Y are computed independently", "[touch_math]") {
    samples_t x_samples{};
    samples_t y_samples{};
    x_samples.fill(touch::calibration_data_t::x_min);
    y_samples.fill(touch::calibration_data_t::y_max);

    const auto coord = touch::compute_coord<10, 2>(x_samples, y_samples, SCREEN_W, SCREEN_H);

    TEST_ASSERT_EQUAL_UINT16(0, coord.x);
    TEST_ASSERT_EQUAL_UINT16(SCREEN_H - 1, coord.y);
}
