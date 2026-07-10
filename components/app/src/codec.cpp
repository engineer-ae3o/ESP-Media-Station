#include "utils.hpp"
#include "codec.hpp"
#include "inmp441.hpp"

#include "esp_err.h"
#include "esp_log.h"

#include "esp_audio_enc.h"
#include "esp_audio_dec.h"
#include "esp_audio_enc_default.h"
#include "esp_audio_dec_default.h"

#include <cstring>
#include <span>
#include <cstdint>
#include <source_location>

namespace audio::codec_opus {

    namespace {

        constexpr const char* TAG = "CODEC";

        static_assert(FRAME_DURATION_MS == 20, "Frame duration must be 20ms");

        void try_esp_audio_func(esp_audio_err_t func, std::source_location location = std::source_location::current()) {
            if (func != ESP_AUDIO_ERR_OK) {
                ESP_LOGE(TAG, "%s:(%s):Line %d failed: %d", location.file_name(), location.function_name(), location.line(), func);
                utils::fatal(location);
            }
        }

        struct stream_header_t {
            size_t num_of_frames{};
        };

        struct frame_header_t {
            size_t   size{};
            uint32_t timestamp_ms{};
        };

        esp_audio_enc_handle_t g_encoder{};
        esp_audio_dec_handle_t g_decoder{};

        int g_in_frame_size{};
        int g_out_frame_size{};

    } // namespace

    void init() {
        // Register built in encoders and decoders
        try_esp_audio_func(esp_audio_enc_register_default());
        try_esp_audio_func(esp_audio_dec_register_default());

        // Configure the opus encoder
        esp_opus_enc_config_t opus_enc_config = {
            .sample_rate      = mic::inmp441_t::SAMPLE_RATE_HZ,
            .channel          = ESP_AUDIO_MONO,
            .bits_per_sample  = ESP_AUDIO_BIT16,
            .bitrate          = BIT_RATE,
            .frame_duration   = ESP_OPUS_ENC_FRAME_DURATION_20_MS,
            .application_mode = ESP_OPUS_ENC_APPLICATION_VOIP,
            .complexity       = 3,
            .enable_fec       = true,
            .enable_dtx       = false,
            .enable_vbr       = true,
        };

        esp_audio_enc_config_t enc_config = {
            .type   = ESP_AUDIO_TYPE_OPUS,
            .cfg    = &opus_enc_config,
            .cfg_sz = sizeof(esp_opus_enc_config_t),
        };
        try_esp_audio_func(esp_audio_enc_open(&enc_config, &g_encoder));

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
        try_esp_audio_func(esp_audio_dec_open(&dec_config, &g_decoder));

        try_esp_audio_func(esp_audio_enc_get_frame_size(g_encoder, &g_in_frame_size, &g_out_frame_size));
        assert(g_in_frame_size == FRAME_SIZE_BYTES);

        ESP_LOGI(TAG, "Frame size: %d bytes. Recommended output buffer size per frame: %d bytes", g_in_frame_size, g_out_frame_size);
    }

    void deinit() {
        esp_audio_enc_close(g_encoder);
        esp_audio_dec_close(g_decoder);

        g_encoder = nullptr;
        g_decoder = nullptr;

        esp_audio_enc_unregister_default();
        esp_audio_dec_unregister_default();
    }

    [[nodiscard]] esp_err_t encode(std::span<uint8_t> pcm_in, std::span<uint8_t> opus_out) {
        if (pcm_in.empty() || pcm_in.data() == nullptr) {
            return ESP_ERR_INVALID_ARG;
        }

        // Ensure the PCM buffer size is a multiple of the frame size
        if ((pcm_in.size_bytes() % FRAME_SIZE_BYTES) != 0) {
            return ESP_ERR_INVALID_SIZE;
        }

        const size_t NUM_OF_FRAMES = pcm_in.size_bytes() / FRAME_SIZE_BYTES;

        // Approximate buffer size needed
        const size_t buffer_size_needed_bytes =
            sizeof(stream_header_t) + (sizeof(frame_header_t) * NUM_OF_FRAMES) + (static_cast<size_t>(g_out_frame_size) * NUM_OF_FRAMES);

        if (opus_out.size_bytes() < buffer_size_needed_bytes) {
            return ESP_ERR_INVALID_SIZE;
        }

        // Track our current position in the encoded buffer
        uint8_t* out_buf      = opus_out.data();
        size_t   out_buf_left = opus_out.size_bytes();

        // Header for the entire stream to be encoded. Place at the start of the buffer
        const stream_header_t stream_header = {
            .num_of_frames = NUM_OF_FRAMES,
        };
        memcpy(out_buf, &stream_header, sizeof(stream_header_t));

        // Move head after placing the stream header
        out_buf += sizeof(stream_header_t);
        out_buf_left -= sizeof(stream_header_t);

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

            auto ret = esp_audio_enc_process(g_encoder, &in_frame, &out_frame);
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
        if (opus_in.empty() || pcm_out.data() == nullptr) {
            return ESP_ERR_INVALID_ARG;
        }

        return ESP_OK;
    }

} // namespace audio::codec_opus
