#pragma once

#include <array>
#include <numeric>
#include <cstdint>
#include <cstddef>
#include <algorithm>

namespace touch {

    struct coord_t {
        uint16_t x{}, y{};
    };

    struct calibration_data_t {
        constexpr static uint16_t x_min{375};
        constexpr static uint16_t y_min{375};
        constexpr static uint16_t x_max{3750};
        constexpr static uint16_t y_max{3750};
    };

    // Raw ADC samples in, screen coordinate out. Sorts, trims TRIM_COUNT
    // outliers off each end, averages what's left, clamps to calibration bounds,
    // then linearly interpolates into screen pixel space.
    template<size_t N, size_t TRIM_COUNT>
    [[nodiscard]] constexpr coord_t compute_coord(std::array<uint16_t, N> x_samples,
                                                  std::array<uint16_t, N> y_samples,
                                                  uint32_t                screen_pixel_len_x,
                                                  uint32_t                screen_pixel_len_y) {

        static_assert(N > TRIM_COUNT * 2, "Not enough samples left after trimming both ends");

        constexpr size_t valid_sample_count = N - (TRIM_COUNT * 2);

        std::ranges::sort(x_samples);
        std::ranges::sort(y_samples);

        // Get the sum of all samples while trimming the TRIM_COUNT lowest and highest samples
        const uint32_t x_sum = std::accumulate(x_samples.begin() + TRIM_COUNT, x_samples.end() - TRIM_COUNT, 0U);
        const uint32_t y_sum = std::accumulate(y_samples.begin() + TRIM_COUNT, y_samples.end() - TRIM_COUNT, 0U);

        // Round to nearest instead of truncating
        const uint32_t average_x = (x_sum + valid_sample_count / 2) / valid_sample_count;
        const uint32_t average_y = (y_sum + valid_sample_count / 2) / valid_sample_count;

        const uint32_t clamped_x = std::clamp(average_x, calibration_data_t::x_min, calibration_data_t::x_max);
        const uint32_t clamped_y = std::clamp(average_y, calibration_data_t::y_min, calibration_data_t::y_max);

        // Linearly interpolate into screen pixel space. Subtract 1 from
        // the screen pixel length since screen pixels are zero indexed.
        const uint32_t screen_x =
            (clamped_x - calibration_data_t::x_min) * (screen_pixel_len_x - 1) / (calibration_data_t::x_max - calibration_data_t::x_min);

        const uint32_t screen_y =
            (clamped_y - calibration_data_t::y_min) * (screen_pixel_len_y - 1) / (calibration_data_t::y_max - calibration_data_t::y_min);

        return {screen_x, screen_y};
    }

} // namespace touch
