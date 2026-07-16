#pragma once

#include "utils.hpp"
#include "inmp441.hpp"

#include "esp_err.h"
#include "esp_log.h"

#include "esp_audio_enc.h"
#include "esp_audio_dec.h"
#include "esp_audio_enc_default.h"
#include "esp_audio_dec_default.h"

#include <span>
#include <memory>
#include <cassert>
#include <cstdint>
#include <utility>
#include <cstring>
#include <concepts>
#include <expected>
#include <algorithm>

namespace audio::codec::opus {

    constexpr inline uint32_t FRAME_DURATION_MS = 20;

    static_assert(FRAME_DURATION_MS == 20, "Frame duration must be 20ms");

    // Header of opus stream
    struct stream_header_t {
        uint32_t number_of_frames{};
        uint32_t total_stream_size_bytes{};
        uint32_t largest_frame_size_bytes{};
    };

    // Header of each frame in an opus stream
    struct frame_header_t {
        uint32_t size_bytes{};
        uint32_t timestamp_ms{};
    };

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

    // These define the implementation of the input opus stream sources. This
    // is because the data could either be from a file or it could be in a flat
    // contiguous buffer, and iterating over the entire range of data in these
    // sources have different implementations. So we implement the iteration here
    // and use concepts to ensure they meet the constraints and API required by
    // the interfaces that use them (DECODE and ANALYZE both use it since they have
    // to iterate over an opus stream and get the frames).
    struct frame_view_t {
        frame_header_t           header{};
        std::span<const uint8_t> data;
    };

    template<typename T>
    concept stream_source_t = requires(T& source) {
        typename T::data_t;
        { source.create(std::declval<typename T::data_t>()) } -> std::same_as<std::expected<T, esp_err_t>>;
        { source.next() } -> std::same_as<std::expected<frame_view_t, esp_err_t>>;
    };

    template<stream_mode_t stream_mode = stream_mode_t::ANALYZE>
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
            m_total_stream_size      = std::exchange(other.m_total_stream_size, 0);
            m_largest_frame_size     = std::exchange(other.m_largest_frame_size, 0);
            m_first_iteration        = std::exchange(other.m_first_iteration, true);
            m_opus_frame_size_approx = std::exchange(other.m_opus_frame_size_approx, 0);
        }

        stream_t& operator=(stream_t&& other) noexcept {
            if (this != &other) {
                end();
                m_encoder                = std::exchange(other.m_encoder, nullptr);
                m_decoder                = std::exchange(other.m_decoder, nullptr);
                m_num_of_frames          = std::exchange(other.m_num_of_frames, 0);
                m_pcm_frame_size         = std::exchange(other.m_pcm_frame_size, 0);
                m_total_stream_size      = std::exchange(other.m_total_stream_size, 0);
                m_largest_frame_size     = std::exchange(other.m_largest_frame_size, 0);
                m_first_iteration        = std::exchange(other.m_first_iteration, true);
                m_opus_frame_size_approx = std::exchange(other.m_opus_frame_size_approx, 0);
            }
            return *this;
        }

        /**
         * @brief Registers the supported codecs used by the codec library.
         * 
         * @note Should be called first before creating any codec instance.
         */
        static void init() {
            esp_audio_enc_register_default();
            esp_audio_dec_register_default();
        }

        /**
         * @brief Unregisters the supported codecs used by the codec library.
         * 
         * @note Should be called last when deinitializing all codec instances.
         */
        static void deinit() {
            esp_audio_enc_unregister_default();
            esp_audio_dec_unregister_default();
        }

        /**
         * @brief Receives a PCM buffer and transforms to opus. 
         * 
         * @param pcm_in   The buffer containing raw PCM data. Must be 16 bit PCM and the
         *                 size of the buffer should be aligned to the size of an input frame.
         * @param opus_out The buffer into which the encoded opus stream is written into.
         *
         * @return The amount of the opus_out span in bytes that was consumed by the driver, and
         *         a bool representing whether or not the encoding was complete or was a partial success.
         *
         * @note This is to be used only in encoder mode.
         */
        [[nodiscard]] std::expected<std::pair<uint32_t, bool>, esp_err_t> encode(std::span<uint8_t> pcm_in, std::span<uint8_t> opus_out) {
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
                uint32_t out_buf_left = opus_out.size_bytes();

                if (m_first_iteration) [[unlikely]] {
                    // Header for the stream to be encoded.
                    // When encoding is done, get_stream_header() should be called to get the total number
                    // of frames written so it can be placed at the start of the stream before being stored or used.
                    // For now, just reserve space at the head of the output opus stream for the header.

                    if (out_buf_left < sizeof(stream_header_t)) {
                        return std::unexpected(ESP_ERR_INVALID_SIZE);
                    }

                    out_buf += sizeof(stream_header_t);
                    out_buf_left -= sizeof(stream_header_t);

                    m_first_iteration    = false;
                    m_num_of_frames      = 0;
                    m_total_stream_size  = sizeof(stream_header_t);
                    m_largest_frame_size = 0;
                }

                // Get the number of frames we're to encode this iteration
                const uint32_t NUM_OF_FRAMES = pcm_in.size_bytes() / m_pcm_frame_size;

                // Approximate the opus output buffer size needed to store the encoded PCM frames. Just a rough estimate.
                const uint32_t total_buf_size_needed =
                    (sizeof(frame_header_t) * NUM_OF_FRAMES) + (m_opus_frame_size_approx * NUM_OF_FRAMES);

                if (out_buf_left < total_buf_size_needed) {
                    return std::unexpected(ESP_ERR_INVALID_SIZE);
                }

                // Chunk the given buffer for encoding
                uint32_t out_bytes_encoded = 0;
                uint32_t in_bytes_encoded  = 0;

                for (uint32_t i = 0; i < NUM_OF_FRAMES; i++) {
                    // Reserve memory for the frame header
                    if (out_buf_left < sizeof(frame_header_t)) {
                        if (i == 0) {
                            return std::unexpected(ESP_ERR_NO_MEM);
                        }
                        // But if we've already encoded some frames, no point in returning a complete failure
                        else {
                            // Instead, return the size of the valid samples we've encoded so far, and let the caller know that this was a partial success
                            return std::pair{(opus_out.size_bytes() - out_buf_left - sizeof(frame_header_t)), false};
                        }
                    }

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
                        ESP_LOGE(TAG, "Error while encoding: %d. Iteration: %u", ret, i);
                        // If this is the first iteration, return an error since nothing has been encoded yet
                        if (i == 0) {
                            return std::unexpected(ESP_ERR_NOT_FINISHED);
                        }
                        // But if we've already encoded some frames, no point in returning a complete failure
                        else {
                            // Instead, return the size of the valid samples we've encoded so far, and let the caller know that this was a partial success
                            return std::pair{(opus_out.size_bytes() - out_buf_left - sizeof(frame_header_t)), false};
                        }
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

                    m_num_of_frames++;
                    m_total_stream_size += sizeof(frame_header_t) + out_frame.encoded_bytes;
                    m_largest_frame_size = std::max(out_frame.encoded_bytes, m_largest_frame_size);
                }

                ESP_LOGI(TAG, "Compression done. Input length: %u bytes. Compressed length: %u", in_bytes_encoded, out_bytes_encoded);

                // Return the size of the opus stream buffer that was consumed.
                return std::pair{(opus_out.size_bytes() - out_buf_left), true};
            }
        }

        /**
         * @brief When encoding is finished from the calling side, this function should be called to get the final
         *        stream header so it can be placed at the head of the opus stream wherever its to be placed.
         *        The driver doesn't do this automatically because the stream span passed at the first call to
         *        encode(...) could be reused, so the driver writing to this same location would corrupt memory
         *        that it does not own. So it's up to the caller to call this function once to get the updated
         *        header when they are done with encoding.
         * 
         * @return The final stream header after all encoding is done.
         *
         * @note This is to be used only in encoder mode.
         */
        [[nodiscard]] std::expected<stream_header_t, esp_err_t> get_stream_header() {
            if constexpr (stream_mode != stream_mode_t::ENCODER) {
                return std::unexpected(ESP_ERR_NOT_SUPPORTED);
            } else {
                return stream_header_t{m_num_of_frames, m_total_stream_size, m_largest_frame_size};
            }
        }

        /**
         * @brief Receives an opus buffer and transforms to 16 bit PCM. 
         * 
         * @param opus_in The source containing the opus frames.
         * @param pcm_out The buffer into which the decoded PCM stream is written into.
         *
         * @return ESP_OK on success, error code otherwise.
         *
         * @note This is to be used only in decoder mode.
         */
        [[nodiscard]] esp_err_t decode(stream_source_t auto& opus_in, std::span<uint8_t> pcm_out) {
            if constexpr (stream_mode != stream_mode_t::DECODER) {
                return ESP_ERR_NOT_SUPPORTED;
            } else {

                return ESP_OK;
            }
        }

        /**
         * @brief Get the stream header of the given opus stream.
         * 
         * @param opus_stream The stream. Pretty straightforward.
         * 
         * @return The stream header.
         *
         * @note This is to be used only in analyze mode.
         */
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

        /**
         * @brief Get the frame header of given opus frame.
         * 
         * @param opus_frame The opus frame.
         * 
         * @return The frame header.
         *
         * @note This is to be used only in analyze mode.
         */
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

    private:
        // None of the member variables are used in stream_mode_t::ANALYZE mode
        esp_audio_enc_handle_t m_encoder{};
        esp_audio_dec_handle_t m_decoder{};

        // Sizes (approx.) of an input frame (PCM) and the output frame (opus) to be passed to the codec for encoding or decoding operations
        int m_pcm_frame_size{};
        int m_opus_frame_size_approx{};

        // Keeps track of whether the call to encode(...) or decode(...) is the first.
        // This is done to know when we should reserve space for the stream header since it appears once at the head of a stream.
        bool m_first_iteration{true};

        // Keeps track of the number of opus frames processed (encoded or decoded) at any point.
        uint32_t m_num_of_frames{};

        // Holds the total encoded stream size (the stream header and frame headers included) at any point.
        // Not used in decoder mode since the total size is already known, since it's in the header of the given stream.
        uint32_t m_total_stream_size{};

        // Holds the size of the largest frame processed at any point.
        uint32_t m_largest_frame_size{};

        static constexpr const char* TAG = "CODEC";

        // Helpers
        stream_t() = default;

        [[nodiscard]] esp_err_t start(const config_t config) {
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

                const uint32_t SAMPLES_PER_FRAME = (static_cast<uint32_t>(config.sample_rate) * FRAME_DURATION_MS) / 1'000U;
                const uint32_t FRAME_SIZE_BYTES  = SAMPLES_PER_FRAME * (ESP_AUDIO_BIT16 / 8) * 1; // 1 channel

                if (m_pcm_frame_size != FRAME_SIZE_BYTES) {
                    ESP_LOGE(TAG, "Mismatched sizes between calculated frame size and one calculated by esp_audio_enc_get_frame_size(...)");
                    return ESP_ERR_INVALID_SIZE;
                }

                // Sanity check that the size is not negative
                if (m_opus_frame_size_approx <= 0) {
                    ESP_LOGE(TAG, "Encoder returned invalid frame size approx: %d", m_opus_frame_size_approx);
                    return ESP_ERR_INVALID_SIZE;
                }

            } else if constexpr (stream_mode == stream_mode_t::DECODER) {
                // Configure the opus decoder
                esp_opus_dec_cfg_t opus_dec_config = {
                    .sample_rate    = static_cast<uint32_t>(config.sample_rate),
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

    // Takes in a span to a contiguous, flat and byte addressable buffer and hands out frames from it.
    class contiguous_frames_t {
    public:
        using data_t = std::span<uint8_t>;

        contiguous_frames_t()  = delete;
        ~contiguous_frames_t() = default;

        contiguous_frames_t(const contiguous_frames_t&)            = delete;
        contiguous_frames_t& operator=(const contiguous_frames_t&) = delete;

        contiguous_frames_t(contiguous_frames_t&&)            = default;
        contiguous_frames_t& operator=(contiguous_frames_t&&) = default;

        [[nodiscard]] static std::expected<contiguous_frames_t, esp_err_t> create(data_t buf) {
            // Read stream header from the front of the passed in span
            auto stream_header = stream_t<>::get_stream_header(buf).value_or({});
            if (stream_header.number_of_frames == 0 || buf.size_bytes() < stream_header.total_stream_size_bytes) {
                return std::unexpected(ESP_ERR_INVALID_SIZE);
            }

            // Get the pointer to the head of the first frame (skip the stream header)
            return contiguous_frames_t{(buf.data() + sizeof(stream_header_t)),
                                       stream_header.number_of_frames,
                                       (stream_header.total_stream_size_bytes - sizeof(stream_header_t))};
        }

        [[nodiscard]] std::expected<frame_view_t, esp_err_t> next() {
            if (m_frames_remaining == 0 || m_bytes_remaining == 0) {
                return std::unexpected(ESP_ERR_NOT_FOUND);
            }

            auto frame_header = stream_t<>::get_frame_header({m_frame_head, m_bytes_remaining}).value_or({});
            if (frame_header.size_bytes == 0) {
                return std::unexpected(ESP_ERR_INVALID_SIZE);
            }

            // Construct the frame. Advance past the frame header and point to the data directly
            const frame_view_t frame = {.header = frame_header, .data = {(m_frame_head + sizeof(frame_header_t)), frame_header.size_bytes}};

            // Advance our internal state
            m_frame_head += (sizeof(frame_header_t) + frame_header.size_bytes);
            m_frames_remaining--;
            m_bytes_remaining -= (sizeof(frame_header_t) + frame_header.size_bytes);

            return frame;
        }

    private:
        uint8_t* m_frame_head{};
        uint32_t m_frames_remaining{};
        uint32_t m_bytes_remaining{};

        contiguous_frames_t(uint8_t* first_frame, uint32_t num_frames, uint32_t bytes_remaining)
            : m_frame_head(first_frame), m_frames_remaining(num_frames), m_bytes_remaining(bytes_remaining) {
        }
    };

    static_assert(stream_source_t<contiguous_frames_t>);

    // Takes in the file path to a file containing an opus stream.
    class file_frames_t {
    public:
        using data_t = const char*;

        file_frames_t() = delete;

        ~file_frames_t() noexcept {
            if (m_file) {
                fclose(m_file);
                m_file = nullptr;
            }
        }

        file_frames_t(const file_frames_t&)            = delete;
        file_frames_t& operator=(const file_frames_t&) = delete;

        file_frames_t(file_frames_t&& other) noexcept {
            m_file              = std::exchange(other.m_file, nullptr);
            m_frames_remaining  = std::exchange(other.m_frames_remaining, 0);
            m_temp_buf_capacity = std::exchange(other.m_temp_buf_capacity, 0);
            m_temp_buf          = std::move(other.m_temp_buf);
        }

        file_frames_t& operator=(file_frames_t&& other) noexcept {
            if (this != &other) {
                if (m_file) {
                    fclose(m_file);
                }
                m_file              = std::exchange(other.m_file, nullptr);
                m_frames_remaining  = std::exchange(other.m_frames_remaining, 0);
                m_temp_buf_capacity = std::exchange(other.m_temp_buf_capacity, 0);
                m_temp_buf          = std::move(other.m_temp_buf);
            }
            return *this;
        }

        [[nodiscard]] static std::expected<file_frames_t, esp_err_t> create(data_t file_path) {
            FILE* file = fopen(file_path, "rb");
            if (file == nullptr) {
                return std::unexpected(ESP_ERR_NOT_FOUND);
            }

            // Read the stream header
            stream_header_t stream_header{};
            if (fread(&stream_header, 1, sizeof(stream_header_t), file) != sizeof(stream_header_t)) {
                fclose(file);
                return std::unexpected(ESP_ERR_INVALID_SIZE);
            }

            bool          success{};
            file_frames_t instance{file, stream_header, success};
            if (!success) {
                return std::unexpected(ESP_ERR_NO_MEM);
            }

            return instance;
        }

        [[nodiscard]] std::expected<frame_view_t, esp_err_t> next() {
            if (m_frames_remaining == 0) {
                return std::unexpected(ESP_ERR_NOT_FOUND);
            }

            return {};
        }

    private:
        FILE*    m_file{};
        uint32_t m_frames_remaining{};
        uint32_t m_temp_buf_capacity{};

        std::unique_ptr<uint8_t[]> m_temp_buf;

        file_frames_t(FILE* file, stream_header_t stream_header, bool& success)
            : m_file(file), m_frames_remaining(stream_header.number_of_frames),
              m_temp_buf_capacity(stream_header.largest_frame_size_bytes) {
            // make_unique(...) throws on failure
            m_temp_buf.reset(new (std::nothrow) uint8_t[m_temp_buf_capacity]);
            success = (m_temp_buf != nullptr);
        }
    };

    static_assert(stream_source_t<file_frames_t>);

} // namespace audio::codec::opus
