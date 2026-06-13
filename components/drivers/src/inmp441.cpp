#pragma once

#include "inmp441.hpp"

namespace mic {

    inmp441_t::~inmp441_t() noexcept {
        if (m_is_initialized) {
            cleanup_resources();
        }
    }

    esp_err_t inmp441_t::init(const config_t& config) {
        if (m_is_initialized) {
            return ESP_ERR_INVALID_STATE;
        }

        m_config = config;

        // TODO: Handle initialization logic

        return ESP_OK;
    }

    esp_err_t inmp441_t::deinit() {
        if (!m_is_initialized) {
            return ESP_ERR_INVALID_STATE;
        }

        cleanup_resources();
        m_is_initialized = false;

        return ESP_OK;
    }

    esp_err_t inmp441_t::get_oneshot_sample(std::span<uint32_t> data) {
        if (!m_is_initialized || m_is_streaming) {
            return ESP_ERR_INVALID_STATE;
        }

        if (data.data() == nullptr || data.size() == 0) {
            return ESP_ERR_INVALID_ARG;
        }

        // TODO: Handle oneshot sampling logic

        return ESP_OK;
    }

    esp_err_t inmp441_t::start_stream() {
        if (!m_is_initialized || m_is_streaming) {
            return ESP_ERR_INVALID_STATE;
        }

        // TODO: Handle streaming mode starting logic

        m_is_streaming = true;

        return ESP_OK;
    }

    esp_err_t inmp441_t::stop_stream() {
        if (!m_is_initialized || !m_is_streaming) {
            return ESP_ERR_INVALID_STATE;
        }

        // TODO: Handle streaming mode stopping logic

        m_is_streaming = false;

        return ESP_OK;
    }

    std::expected<std::span<const uint32_t>, esp_err_t> inmp441_t::get_free_buffer(uint32_t timeout_ms) const {
        if (!m_is_initialized) {
            return std::unexpected(ESP_ERR_INVALID_STATE);
        }

        // TODO: Handle buffer returning logic

        return {};
    }

    // Helpers
    void inmp441_t::cleanup_resources() {
    }

} // namespace mic
