#pragma once

#include "utils.hpp"
#include "codec.hpp"
#include "inmp441.hpp"

#include "esp_err.h"
#include "esp_log.h"

#include "esp_audio_enc.h"
#include "esp_audio_dec.h"
#include "esp_audio_enc_default.h"
#include "esp_audio_dec_default.h"

#include <span>
#include <cstdint>
#include <utility>
#include <cstring>
#include <expected>

namespace audio::codec_opus {

    constexpr inline uint32_t BIT_RATE          = 40'000;
    constexpr inline uint32_t FRAME_DURATION_MS = 20;

    constexpr inline uint32_t SAMPLES_PER_FRAME = (mic::inmp441_t::SAMPLE_RATE_HZ * FRAME_DURATION_MS) / 1'000;
    constexpr inline uint32_t FRAME_SIZE_BYTES  = SAMPLES_PER_FRAME * (ESP_AUDIO_BIT16 / 8) * 1; // 1 channel

    static_assert(FRAME_DURATION_MS == 20, "Frame duration must be 20ms");

    // Header for opus data and buffer
    using stream_header_t = size_t;

    struct frame_header_t {
        size_t   size{};
        uint32_t timestamp_ms{};
    };

    inline void init() {
        esp_audio_enc_register_default();
        esp_audio_dec_register_default();
    }

    inline void deinit() {
        esp_audio_enc_unregister_default();
        esp_audio_dec_unregister_default();
    }

    template<bool use_encoder = true>
    class stream_opus_t {
    public:
        struct config_t {
            int bit_rate{};
            int complexity{};
            int sample_rate{};

            esp_opus_enc_application_t mode{};
        };

        constexpr static config_t default_config = {
            .bit_rate    = 40'000,
            .complexity  = 4,
            .sample_rate = mic::inmp441_t::SAMPLE_RATE_HZ,
            .mode        = ESP_OPUS_ENC_APPLICATION_VOIP,
        };

        [[nodiscard]] static std::expected<stream_opus_t, esp_err_t> create(const config_t config = default_config) {
            stream_opus_t instance{};
            if (auto ret = instance.start(config); ret != ESP_OK) {
                return std::unexpected(ret);
            }
            return instance;
        }

        ~stream_opus_t() noexcept {
            end();
        }

        stream_opus_t(const stream_opus_t&)            = delete;
        stream_opus_t& operator=(const stream_opus_t&) = delete;

        stream_opus_t(stream_opus_t&& other) noexcept {
            m_encoder        = std::exchange(other.m_encoder, nullptr);
            m_decoder        = std::exchange(other.m_decoder, nullptr);
            m_num_of_frames  = std::exchange(other.m_num_of_frames, 0);
            m_in_frame_size  = std::exchange(other.m_in_frame_size, 0);
            m_out_frame_size = std::exchange(other.m_out_frame_size, 0);
        }

        stream_opus_t& operator=(stream_opus_t&& other) noexcept {
            if (this != &other) {
                end();
                m_encoder        = std::exchange(other.m_encoder, nullptr);
                m_decoder        = std::exchange(other.m_decoder, nullptr);
                m_num_of_frames  = std::exchange(other.m_num_of_frames, 0);
                m_in_frame_size  = std::exchange(other.m_in_frame_size, 0);
                m_out_frame_size = std::exchange(other.m_out_frame_size, 0);
            }
            return *this;
        }

        [[nodiscard]] esp_err_t encode(std::span<uint8_t> pcm_in, std::span<uint8_t> opus_out) {
            if (!use_encoder) {
                return ESP_ERR_NOT_SUPPORTED;
            }

            if (pcm_in.empty() || pcm_in.data() == nullptr || opus_out.empty() || opus_out.data() == nullptr) {
                return ESP_ERR_INVALID_ARG;
            }

            // Ensure the PCM buffer size is a multiple of the frame size
            if ((pcm_in.size_bytes() % FRAME_SIZE_BYTES) != 0) {
                return ESP_ERR_INVALID_SIZE;
            }

            const size_t NUM_OF_FRAMES = pcm_in.size_bytes() / FRAME_SIZE_BYTES;
            m_num_of_frames += NUM_OF_FRAMES;

            // Approximate buffer size needed
            const size_t buffer_size_needed_bytes = sizeof(stream_header_t) + (sizeof(frame_header_t) * NUM_OF_FRAMES) +
                                                    (static_cast<size_t>(m_out_frame_size) * NUM_OF_FRAMES);

            if (opus_out.size_bytes() < buffer_size_needed_bytes) {
                return ESP_ERR_INVALID_SIZE;
            }

            // Track our current position in the encoded buffer
            uint8_t* out_buf      = opus_out.data();
            size_t   out_buf_left = opus_out.size_bytes();

            static bool first_iteration = true;

            if (first_iteration) {
                // Header for the entire stream to be encoded.
                // Will be updated after `end()` is called.
                const stream_header_t stream_header = 0;
                memcpy(out_buf, &stream_header, sizeof(stream_header_t));

                // Move head after placing the stream header
                out_buf += sizeof(stream_header_t);
                out_buf_left -= sizeof(stream_header_t);

                first_iteration = false;
            }

            // Chunk the given buffer for encoding
            size_t out_bytes_encoded = 0;
            size_t in_bytes_encoded  = 0;

            for (size_t i = 0; i < NUM_OF_FRAMES; i++) {
                // Reserve memory for the frame header
                out_buf += sizeof(frame_header_t);
                out_buf_left -= sizeof(frame_header_t);

                // Details of the PCM data to be encoded
                esp_audio_enc_in_frame_t in_frame = {
                    .buffer = pcm_in.data() + in_bytes_encoded, // Advance head of input buffer
                    .len    = FRAME_SIZE_BYTES,
                };

                esp_audio_enc_out_frame_t out_frame = {
                    .buffer        = out_buf,
                    .len           = out_buf_left,
                    .encoded_bytes = 0,
                    .pts           = 0,
                };

                auto ret = esp_audio_enc_process(m_encoder, &in_frame, &out_frame);
                if (ret != ESP_AUDIO_ERR_OK) {
                    ESP_LOGE(TAG, "Error while encoding: %d. Iteration: %zu", ret, i);
                    return ESP_ERR_NOT_FINISHED;
                }

                // Header for each frame
                const frame_header_t header = {
                    .size         = out_frame.encoded_bytes,
                    .timestamp_ms = static_cast<uint32_t>(out_frame.pts),
                };

                // Walk back by the size of the frame header and place the frame header, before the encoded data
                memcpy((out_buf - sizeof(frame_header_t)), &header, sizeof(frame_header_t));

                // Update our state
                in_bytes_encoded += FRAME_SIZE_BYTES;
                out_bytes_encoded += out_frame.encoded_bytes;

                out_buf += out_frame.encoded_bytes;
                out_buf_left -= out_frame.encoded_bytes;
            }

            ESP_LOGI(TAG, "Compression done. Input length: %zu bytes. Compressed length: %zu", in_bytes_encoded, out_bytes_encoded);

            return ESP_OK;
        }

        [[nodiscard]] esp_err_t decode(std::span<uint8_t> opus_in, std::span<uint8_t> pcm_out) {
            if (use_encoder) {
                return ESP_ERR_NOT_SUPPORTED;
            }

            return ESP_OK;
        }

    private:
        esp_audio_enc_handle_t m_encoder{};
        esp_audio_dec_handle_t m_decoder{};

        int    m_in_frame_size{};
        int    m_out_frame_size{};
        size_t m_num_of_frames{};

        static constexpr const char* TAG = "CODEC";

        // Member functions
        stream_opus_t() = default;

        esp_err_t start(const config_t config = default_config) {
            if constexpr (use_encoder) {
                // Configure the opus encoder
                esp_opus_enc_config_t opus_enc_config = {
                    .sample_rate      = config.sample_rate,
                    .channel          = ESP_AUDIO_MONO,
                    .bits_per_sample  = ESP_AUDIO_BIT16,
                    .bitrate          = config.bit_rate,
                    .frame_duration   = ESP_OPUS_ENC_FRAME_DURATION_20_MS,
                    .application_mode = config.mode,
                    .complexity       = config.complexity,
                    .enable_fec       = true,
                    .enable_dtx       = false,
                    .enable_vbr       = true,
                };

                esp_audio_enc_config_t enc_config = {
                    .type   = ESP_AUDIO_TYPE_OPUS,
                    .cfg    = &opus_enc_config,
                    .cfg_sz = sizeof(esp_opus_enc_config_t),
                };

                if (auto ret = esp_audio_enc_open(&enc_config, &m_encoder); ret != ESP_AUDIO_ERR_OK) {
                    ESP_LOGE(TAG, "Failed to configure the opus encoder: %d", std::to_underlying(ret));
                    return ESP_FAIL;
                }

                if (auto ret = esp_audio_enc_get_frame_size(m_encoder, &m_in_frame_size, &m_out_frame_size); ret != ESP_AUDIO_ERR_OK) {
                    ESP_LOGE(TAG, "Failed to get frame size: %d", std::to_underlying(ret));
                    return ESP_ERR_INVALID_RESPONSE;
                }

                const size_t SAMPLES_PER_FRAME = (static_cast<size_t>(config.sample_rate) * FRAME_DURATION_MS) / 1'000U;
                const size_t FRAME_SIZE_BYTES  = SAMPLES_PER_FRAME * (ESP_AUDIO_BIT16 / 8) * 1; // 1 channel

                if (m_in_frame_size != FRAME_SIZE_BYTES) {
                    ESP_LOGE(TAG, "Mismatched sizes between calculated frame size and one calculated by esp_audio_enc_get_frame_size(...)");
                    return ESP_ERR_INVALID_SIZE;
                }

            } else {
                // Configure the opus decoder
                esp_opus_dec_cfg_t opus_dec_config = {
                    .sample_rate    = mic::inmp441_t::SAMPLE_RATE_HZ,
                    .channel        = ESP_AUDIO_MONO,
                    .frame_duration = ESP_OPUS_DEC_FRAME_DURATION_20_MS,
                    .self_delimited = false,
                };

                esp_audio_dec_cfg_t dec_config = {
                    .type   = ESP_AUDIO_TYPE_OPUS,
                    .cfg    = &opus_dec_config,
                    .cfg_sz = sizeof(esp_opus_dec_cfg_t),
                };

                if (auto ret = esp_audio_dec_open(&dec_config, &m_decoder); ret != ESP_AUDIO_ERR_OK) {
                    ESP_LOGE(TAG, "Failed to configure the opus decoder: %d", std::to_underlying(ret));
                    return ESP_FAIL;
                }
            }

            return ESP_OK;
        }

        void end() {
            if (m_encoder) {
                esp_audio_enc_close(m_encoder);
                m_encoder = nullptr;
            }
        }
    };

} // namespace audio::codec_opus
