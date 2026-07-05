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

    template<size_t N, size_t TRIM_COUNT>
    [[nodiscard]] constexpr uint16_t compute_trimmed_mean(std::array<uint16_t, N> samples, uint16_t screen_pixel_len) {

        static_assert(N > TRIM_COUNT * 2, "Not enough samples left after trimming both ends");

        constexpr size_t valid_sample_count = N - (TRIM_COUNT * 2);

        std::ranges::sort(samples);

        // Get the sum of all samples while trimming the TRIM_COUNT lowest and highest samples
        const uint16_t x_sum = std::accumulate(samples.begin() + TRIM_COUNT, samples.end() - TRIM_COUNT, 0U);

        // Take the average and round to nearest instead of truncating
        const uint16_t average = (x_sum + valid_sample_count / 2) / valid_sample_count;

        return average;
    }

    // Raw ADC samples in, screen coordinate out. Sorts, trims TRIM_COUNT
    // outliers off each end, averages what's left, clamps to calibration bounds,
    // then linearly interpolates into screen pixel space.
    template<size_t N, size_t TRIM_COUNT>
    [[nodiscard]] constexpr coord_t compute_coord(std::array<uint16_t, N> x_samples,
                                                  std::array<uint16_t, N> y_samples,
                                                  uint16_t                screen_pixel_len_x,
                                                  uint16_t                screen_pixel_len_y) {

        auto trimmed_x = compute_trimmed_mean<N, TRIM_COUNT>(x_samples, screen_pixel_len_x);
        auto trimmed_y = compute_trimmed_mean<N, TRIM_COUNT>(y_samples, screen_pixel_len_y);

        const uint16_t clamped_x = std::clamp(trimmed_x, calibration_data_t::x_min, calibration_data_t::x_max);
        const uint16_t clamped_y = std::clamp(trimmed_y, calibration_data_t::x_min, calibration_data_t::x_max);

        // Linearly interpolate into screen pixel space. Subtract 1 from
        // the screen pixel length since screen pixels are zero indexed.
        const uint16_t screen_x =
            (clamped_x - calibration_data_t::x_min) * (screen_pixel_len_x - 1) / (calibration_data_t::x_max - calibration_data_t::x_min);

        const uint16_t screen_y =
            (clamped_y - calibration_data_t::y_min) * (screen_pixel_len_y - 1) / (calibration_data_t::y_max - calibration_data_t::y_min);

        return {screen_x, screen_y};
    }

} // namespace touch
