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
#include <concepts>
#include <expected>

namespace audio::codec::opus {

    constexpr inline uint32_t FRAME_DURATION_MS = 20;

    static_assert(FRAME_DURATION_MS == 20, "Frame duration must be 20ms");

    // Header of opus stream
    struct stream_header_t {
        size_t number_of_frames{};
        size_t total_stream_size_bytes{};
        size_t largest_frame_size_bytes{};
    };

    // Header of each frame in an opus stream
    struct frame_header_t {
        size_t   size_bytes{};
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

    enum class stream_mode_t : uint8_t {
        ANALYZE = 0,
        ENCODER,
        DECODER,
    };

    struct config_t {
        int bit_rate{};
        int complexity{};
        int sample_rate{};

        esp_opus_enc_application_t mode{};
    };

    template<stream_mode_t stream_mode>
    class stream_t {
    public:
        constexpr static config_t default_config = {
            .bit_rate    = 40'000,
            .complexity  = 4,
            .sample_rate = mic::inmp441_t::SAMPLE_RATE_HZ,
            .mode        = ESP_OPUS_ENC_APPLICATION_VOIP,
        };

        [[nodiscard]] static std::expected<stream_t, esp_err_t> create(const config_t& config = default_config) {
            stream_t instance{};
            if (auto ret = instance.start(config); ret != ESP_OK) {
                return std::unexpected(ret);
            }
            return instance;
        }

        ~stream_t() noexcept {
            end();
        }

        stream_t(const stream_t&)            = delete;
        stream_t& operator=(const stream_t&) = delete;

        stream_t(stream_t&& other) noexcept {
            m_encoder                = std::exchange(other.m_encoder, nullptr);
            m_decoder                = std::exchange(other.m_decoder, nullptr);
            m_num_of_frames          = std::exchange(other.m_num_of_frames, 0);
            m_pcm_frame_size         = std::exchange(other.m_pcm_frame_size, 0);
            m_opus_frame_size_approx = std::exchange(other.m_opus_frame_size_approx, 0);
        }

        stream_t& operator=(stream_t&& other) noexcept {
            if (this != &other) {
                end();
                m_encoder                = std::exchange(other.m_encoder, nullptr);
                m_decoder                = std::exchange(other.m_decoder, nullptr);
                m_num_of_frames          = std::exchange(other.m_num_of_frames, 0);
                m_pcm_frame_size         = std::exchange(other.m_pcm_frame_size, 0);
                m_opus_frame_size_approx = std::exchange(other.m_opus_frame_size_approx, 0);
            }
            return *this;
        }

        std::expected<std::span<uint8_t>, esp_err_t> encode(std::span<uint8_t> pcm_in, std::span<uint8_t> opus_out) {
            if constexpr (stream_mode != stream_mode_t::ENCODER) {
                return std::unexpected(ESP_ERR_NOT_SUPPORTED);

            } else {
                if (pcm_in.empty() || pcm_in.data() == nullptr || opus_out.empty() || opus_out.data() == nullptr) {
                    return std::unexpected(ESP_ERR_INVALID_ARG);
                }

                // Ensure the PCM input buffer size is a multiple of the input frame size
                if ((pcm_in.size_bytes() % m_pcm_frame_size) != 0) {
                    return std::unexpected(ESP_ERR_INVALID_SIZE);
                }

                // Track our current position in the encoded buffer
                uint8_t* out_buf      = opus_out.data();
                size_t   out_buf_left = opus_out.size_bytes();

                if (m_first_iteration) [[unlikely]] {
                    // Header for the stream to be encoded.
                    // When encoding is done, get_num_of_frames_processed() should be called to get the total number
                    // of frames written so it can be placed at the start of the stream before being stored or used.
                    // For now, just reserve space at the head of the output opus stream for the header.

                    out_buf += sizeof(stream_header_t);
                    out_buf_left -= sizeof(stream_header_t);

                    m_first_iteration    = false;
                    m_num_of_frames      = 0;
                    m_total_stream_size  = sizeof(stream_header_t);
                    m_largest_frame_size = 0;
                }

                // Get the number of frames we're to encode this iteration
                const size_t NUM_OF_FRAMES = pcm_in.size_bytes() / m_pcm_frame_size;
                m_num_of_frames += NUM_OF_FRAMES;

                // Approximate the opus output buffer size needed to store the encoded PCM frames. Just a rough estimate.
                const size_t buffer_size_needed_bytes = sizeof(stream_header_t) + (sizeof(frame_header_t) * NUM_OF_FRAMES) +
                                                        (static_cast<size_t>(m_opus_frame_size_approx) * NUM_OF_FRAMES);

                if (out_buf_left < buffer_size_needed_bytes) {
                    return std::unexpected(ESP_ERR_INVALID_SIZE);
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
                        .len    = m_pcm_frame_size,
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
                        return std::unexpected(ESP_ERR_NOT_FINISHED);
                    }

                    // Header for each frame
                    const frame_header_t frame_header = {
                        .size_bytes   = out_frame.encoded_bytes,
                        .timestamp_ms = static_cast<uint32_t>(out_frame.pts),
                    };

                    // Walk back by the size of the frame header and place the frame header, before the frame
                    memcpy((out_buf - sizeof(frame_header_t)), &frame_header, sizeof(frame_header_t));

                    // Update our state
                    in_bytes_encoded += m_pcm_frame_size;
                    out_bytes_encoded += out_frame.encoded_bytes;

                    out_buf += out_frame.encoded_bytes;
                    out_buf_left -= out_frame.encoded_bytes;

                    m_largest_frame_size = std::max(out_frame.encoded_bytes, m_largest_frame_size);
                    m_total_stream_size += sizeof(frame_header_t) + out_frame.encoded_bytes;
                }

                ESP_LOGI(TAG, "Compression done. Input length: %zu bytes. Compressed length: %zu", in_bytes_encoded, out_bytes_encoded);

                // Return a span corresponding to the size of the original buffer that we consumed
                return std::span{opus_out.data(), opus_out.size_bytes() - out_buf_left};
            }
        }

        // Used only in ENCODER mode. When encoding is done from the calling side, this function should be called
        // to get the final stream header so it can be placed at the head of the opus stream wherever its to be placed.
        // The driver doesn't do this automatically because the stream span passed at the first call to encode(...) could
        // be reused, so the driver writing to this same location would corrupt memory that it does not own. So it's up
        // to the caller to call this function once to get the updated header when they are done with encoding.
        std::expected<stream_header_t, esp_err_t> get_stream_header() {
            if constexpr (stream_mode != stream_mode_t::ENCODER) {
                return std::unexpected(ESP_ERR_NOT_SUPPORTED);
            } else {
                return stream_header_t{m_num_of_frames, m_total_stream_size, m_largest_frame_size};
            }
        }

        esp_err_t decode(std::span<uint8_t> opus_in, std::span<uint8_t> pcm_out) {
            if constexpr (stream_mode != stream_mode_t::DECODER) {
                return ESP_ERR_NOT_SUPPORTED;
            } else {

                return ESP_OK;
            }
        }

        // These three should only be used in stream_mode_t::ANALYZE mode
        [[nodiscard]] static std::expected<stream_header_t, esp_err_t> get_stream_header(std::span<const uint8_t> opus_stream) {
            if constexpr (stream_mode != stream_mode_t::ANALYZE) {
                return std::unexpected(ESP_ERR_NOT_SUPPORTED);

            } else {
                if (opus_stream.empty() || opus_stream.data() == nullptr) {
                    return std::unexpected(ESP_ERR_INVALID_ARG);
                }

                if (opus_stream.size_bytes() < sizeof(stream_header_t)) {
                    return std::unexpected(ESP_ERR_INVALID_SIZE);
                }

                stream_header_t stream_header{};
                memcpy(&stream_header, opus_stream.data(), sizeof(stream_header_t));

                return stream_header;
            }
        }

        [[nodiscard]] static std::expected<frame_header_t, esp_err_t> get_frame_header(std::span<const uint8_t> opus_frame) {
            if constexpr (stream_mode != stream_mode_t::ANALYZE) {
                return std::unexpected(ESP_ERR_NOT_SUPPORTED);

            } else {
                if (opus_frame.empty() || opus_frame.data() == nullptr) {
                    return std::unexpected(ESP_ERR_INVALID_ARG);
                }

                if (opus_frame.size_bytes() < sizeof(frame_header_t)) {
                    return std::unexpected(ESP_ERR_INVALID_SIZE);
                }

                frame_header_t frame_header{};
                memcpy(&frame_header, opus_frame.data(), sizeof(frame_header_t));

                return frame_header;
            }
        }

        [[nodiscard]] static std::expected<std::span<const uint8_t>, esp_err_t> iterate_opus_stream(std::span<const uint8_t> opus) {
            if constexpr (stream_mode != stream_mode_t::ANALYZE) {
                return std::unexpected(ESP_ERR_NOT_SUPPORTED);

            } else {
                const auto stream_header = get_stream_header(opus).value_or({});
                if (stream_header.number_of_frames == 0) {
                    return std::unexpected(ESP_ERR_INVALID_SIZE);
                }

                // Track our current position in the buffer and skip stream header so we point to the header of the first frame
                const uint8_t* out_buf = opus.data() + sizeof(stream_header_t);
                size_t         frame_length{};

                return std::span{out_buf, frame_length};
            }
        }

    private:
        // None of the member variables are used in stream_mode_t::ANALYZE mode
        esp_audio_enc_handle_t m_encoder{};
        esp_audio_dec_handle_t m_decoder{};

        // Sizes (approx.) of an input frame (PCM) and the output frame (opus) to be passed to the codec for encoding or decoding ops
        int m_pcm_frame_size{};
        int m_opus_frame_size_approx{};

        // Keeps track of whether the call to encode(...) or decode(...) is the first.
        // This is done to know when we should reserve space for the stream header since it appears once at the head of a stream.
        bool m_first_iteration{true};

        // Keeps track of the number of opus frames processed (encoded or decoded) at any point.
        size_t m_num_of_frames{};

        // Holds the total encoded stream size (the stream header and frame headers included) at any point.
        // Not used in decoder mode since the total size is already known, since it's in the header of the given stream.
        size_t m_total_stream_size{};

        // Holds the size of the largest frame processed at any point.
        size_t m_largest_frame_size{};

        static constexpr const char* TAG = "CODEC";

        // Helpers
        stream_t() = default;

        esp_err_t start(const config_t config) {
            if constexpr (stream_mode == stream_mode_t::ENCODER) {
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

                if (auto ret = esp_audio_enc_get_frame_size(m_encoder, &m_pcm_frame_size, &m_opus_frame_size_approx);
                    ret != ESP_AUDIO_ERR_OK) {
                    ESP_LOGE(TAG, "Failed to get frame size: %d", std::to_underlying(ret));
                    return ESP_ERR_INVALID_RESPONSE;
                }

                const size_t SAMPLES_PER_FRAME = (static_cast<size_t>(config.sample_rate) * FRAME_DURATION_MS) / 1'000U;
                const size_t FRAME_SIZE_BYTES  = SAMPLES_PER_FRAME * (ESP_AUDIO_BIT16 / 8) * 1; // 1 channel

                if (m_pcm_frame_size != FRAME_SIZE_BYTES) {
                    ESP_LOGE(TAG, "Mismatched sizes between calculated frame size and one calculated by esp_audio_enc_get_frame_size(...)");
                    return ESP_ERR_INVALID_SIZE;
                }

            } else if constexpr (stream_mode == stream_mode_t::DECODER) {
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
            if constexpr (stream_mode == stream_mode_t::ENCODER) {
                if (m_encoder) {
                    esp_audio_enc_close(m_encoder);
                    m_encoder = nullptr;
                }
            } else if constexpr (stream_mode == stream_mode_t::DECODER) {
                if (m_decoder) {
                    esp_audio_dec_close(m_decoder);
                    m_decoder = nullptr;
                }
            }
        }
    };

} // namespace audio::codec::opus
