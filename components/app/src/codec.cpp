#include "codec.hpp"

namespace audio::codec_opus {

    std::expected<size_t, esp_err_t> get_num_of_frames(std::span<const uint8_t> opus) {
        if (opus.empty() || opus.data() == nullptr) {
            return std::unexpected(ESP_ERR_INVALID_ARG);
        }

        if (opus.size_bytes() < sizeof(stream_header_t)) {
            return std::unexpected(ESP_ERR_INVALID_SIZE);
        }

        stream_header_t header{};
        memcpy(&header, opus.data(), sizeof(stream_header_t));

        return header.num_of_frames;
    }

    std::expected<frame_header_t, esp_err_t> get_frame_header(std::span<const uint8_t> opus_frame) {
        if (opus_frame.empty() || opus_frame.data() == nullptr) {
            return std::unexpected(ESP_ERR_INVALID_ARG);
        }

        if (opus_frame.size_bytes() < sizeof(frame_header_t)) {
            return std::unexpected(ESP_ERR_INVALID_SIZE);
        }

        frame_header_t header{};
        memcpy(&header, opus_frame.data(), sizeof(frame_header_t));

        return header;
    }

    std::expected<std::span<const uint8_t>, esp_err_t> get_nth_frame(std::span<uint8_t> opus, uint32_t idx) {
        const auto num_of_frames = get_num_of_frames(opus).value_or(0);
        if (num_of_frames == 0 || idx >= num_of_frames) {
            return std::unexpected(ESP_ERR_INVALID_ARG);
        }

        // Track our current position in the buffer and skip stream header so we point to the header of the first frame
        const uint8_t* out_buf = opus.data() + sizeof(stream_header_t);
        size_t         frame_length{};

        // Walk the buffer till we hit the matching index
        for (uint32_t i = 0; i < num_of_frames; i++) {
            // Header for nth frame
            frame_header_t header{};
            memcpy(&header, opus.data(), sizeof(frame_header_t));

            // Move head of the buffer by the size of this frame and its header
            out_buf += sizeof(frame_header_t) + header.size;
        }

        return std::span{out_buf, frame_length};
    }

} // namespace audio::codec_opus
