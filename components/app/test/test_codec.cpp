#include "unity.h"

#include "codec.hpp"

#include "esp_heap_caps.h"

#include <array>
#include <cmath>
#include <cerrno>
#include <cstdio>
#include <memory>
#include <cstring>
#include <numbers>
#include <cstdint>
#include <sys/stat.h>

namespace {

    using namespace audio::codec::opus;
    using namespace audio::codec;

    constexpr uint32_t SAMPLE_RATE_HZ     = 48'000;
    constexpr uint32_t OUT_BUF_SIZE_BYTES = 4096;
    constexpr uint32_t FRAME_DURATION_MS  = 20;
    constexpr uint32_t SECONDS_TO_TEST    = 5;

    // LittleFS is mounted at /lfs. Tests that touch disk get their own
    // subdirectory so they don't scatter files across the mount, and that
    // directory is torn down again once those tests are done with it.
    constexpr const char* TEST_DIR_PATH  = "/lfs/codec";
    constexpr const char* TEST_FILE_PATH = "/lfs/codec/opus_test_stream.bin";

    consteval config_t get_encoder_config() {
        static_assert(FRAME_DURATION_MS == 20);
        return config_t{
            .bit_rate          = 40'000,
            .complexity        = 4,
            .sample_rate       = SAMPLE_RATE_HZ,
            .frame_duration_ms = FRAME_DURATION_MS,
            .mode              = ESP_OPUS_ENC_APPLICATION_VOIP,
            .duration_type     = ESP_OPUS_ENC_FRAME_DURATION_20_MS,
        };
    }

    consteval config_t get_decoder_config() {
        static_assert(FRAME_DURATION_MS == 20);
        return config_t{
            .bit_rate          = 40'000,
            .complexity        = 4,
            .sample_rate       = SAMPLE_RATE_HZ,
            .frame_duration_ms = FRAME_DURATION_MS,
            .mode              = ESP_OPUS_ENC_APPLICATION_VOIP, // Not needed in decoder mode
            .duration_type     = ESP_OPUS_DEC_FRAME_DURATION_20_MS,
        };
    }

    consteval config_t get_mismatched_encoder_config() {
        config_t cfg      = get_encoder_config();
        cfg.duration_type = ESP_OPUS_DEC_FRAME_DURATION_20_MS; // Use the wrong type for duration_type here
        return cfg;
    }

    consteval config_t get_mismatched_decoder_config() {
        config_t cfg      = get_decoder_config();
        cfg.duration_type = ESP_OPUS_ENC_FRAME_DURATION_20_MS; // Use the wrong type for duration_type here
        return cfg;
    }

    // Registers/unregisters the codec library per test.
    struct codec_fixture_t {
        codec_fixture_t() {
            stream_t<>::init();
        }

        ~codec_fixture_t() {
            stream_t<>::deinit();
        }

        codec_fixture_t(const codec_fixture_t&)            = delete;
        codec_fixture_t& operator=(const codec_fixture_t&) = delete;
        codec_fixture_t(codec_fixture_t&&)                 = delete;
        codec_fixture_t& operator=(codec_fixture_t&&)      = delete;
    };

    /**
     * @brief Takes in the number of 16 bit PCM samples to generate, allocates a buffer
     *        for it, generates a 440Hz sine wave in the buffer, and transfers ownership
     *        of the buffer to the caller. 
     * 
     * @param sample_count Number of 16 bit PCM samples to generate.
     * 
     * @return The buffer holding the PCM.
     */
    std::unique_ptr<int16_t[]> make_sine_pcm(uint32_t sample_count) {
        constexpr float freq_hz = 440;
        constexpr float pi      = std::numbers::pi_v<float>;

        std::unique_ptr<int16_t[]> buf(new (std::nothrow) int16_t[sample_count]);
        TEST_ASSERT_NOT_NULL_MESSAGE(buf, "Failed to allocate PCM buffer for opus test");

        for (uint32_t i = 0; i < sample_count; i++) {
            const float sample = std::sin(2 * pi * freq_hz * static_cast<float>(i) / SAMPLE_RATE_HZ);
            buf[i]             = static_cast<int16_t>(sample * INT16_MAX);
        }

        return buf;
    }

    // Encodes SECONDS_TO_TEST worth of sine PCM into a freshly allocated,
    // fully headered opus stream buffer.
    struct encoded_stream_t {
        std::unique_ptr<int16_t[]> pcm;  // 16 bit PCM data
        std::unique_ptr<uint8_t[]> opus; // Just a buffer of raw bytes

        uint32_t opus_capacity{};
        uint32_t opus_used{};
        uint32_t frame_count{};

        encoded_stream_t()  = default;
        ~encoded_stream_t() = default;

        encoded_stream_t(const encoded_stream_t&)            = delete;
        encoded_stream_t& operator=(const encoded_stream_t&) = delete;

        encoded_stream_t(encoded_stream_t&& other) noexcept {
            pcm           = std::move(other.pcm);
            opus          = std::move(other.opus);
            opus_used     = std::exchange(other.opus_used, 0);
            frame_count   = std::exchange(other.frame_count, 0);
            opus_capacity = std::exchange(other.opus_capacity, 0);
        }

        encoded_stream_t& operator=(encoded_stream_t&& other) noexcept {
            if (this != &other) {
                pcm           = std::move(other.pcm);
                opus          = std::move(other.opus);
                opus_used     = std::exchange(other.opus_used, 0);
                frame_count   = std::exchange(other.frame_count, 0);
                opus_capacity = std::exchange(other.opus_capacity, 0);
            }
            return *this;
        }
    };

    /**
     * @brief 
     * 
     * @return The built encoded opus stream. 
     */
    encoded_stream_t build_encoded_stream() {
        auto encoder = stream_t<opus::mode_t::ENCODER>::create(get_encoder_config());
        TEST_ASSERT_TRUE_MESSAGE(encoder.has_value(), "Failed to create encoder instance while building test fixture stream");

        // Get the size of buffer we have to allocate to store the PCM data as well find the number of PCM samples
        constexpr uint32_t sample_count       = SAMPLE_RATE_HZ * SECONDS_TO_TEST;
        constexpr uint32_t pcm_buf_size_bytes = sample_count * sizeof(int16_t); // 16 bit PCM

        // Manually calculate the PCM frame size
        constexpr uint32_t SAMPLES_PER_FRAME     = (get_encoder_config().sample_rate * get_encoder_config().frame_duration_ms) / 1'000;
        constexpr uint32_t PCM_FRAME_SIZE_ACTUAL = SAMPLES_PER_FRAME * (ESP_AUDIO_BIT16 / 8) * 1; // 1 channel

        // Check that the manually calculated PCM input frame size matches the one from stream_t<>::get_input_frame_size()
        TEST_ASSERT_EQUAL(PCM_FRAME_SIZE_ACTUAL, encoder->get_input_frame_size());

        // Create the stream
        encoded_stream_t result{};
        result.pcm           = make_sine_pcm(sample_count);
        result.opus_capacity = pcm_buf_size_bytes + (1024 * 1024 * sizeof(frame_header_t)); // Generous headroom

        result.opus.reset((new (std::nothrow) uint8_t[result.opus_capacity]));
        TEST_ASSERT_NOT_NULL_MESSAGE(result.opus, "Failed to allocate opus output buffer for test fixture stream");

        auto ret =
            encoder->encode({reinterpret_cast<uint8_t*>(result.pcm.get()), pcm_buf_size_bytes}, {result.opus.get(), result.opus_capacity});
        TEST_ASSERT_TRUE_MESSAGE(ret.has_value(), "Encoding the fixture stream failed");

        const auto& [written, consumed, complete] = ret.value();
        TEST_ASSERT_TRUE_MESSAGE(complete, "Fixture stream encode reported partial success unexpectedly");
        TEST_ASSERT_EQUAL_MESSAGE(pcm_buf_size_bytes, consumed, "Fixture stream did not consume the full PCM buffer");

        const auto header = encoder->get_stream_header();
        TEST_ASSERT_TRUE_MESSAGE(header.has_value(), "Failed to retrieve stream header for fixture stream");

        // Stamp the finalized header at the head of the stream, as documented on get_stream_header(...).
        memcpy(result.opus.get(), &header.value(), sizeof(stream_header_t));

        result.opus_used   = written.size_bytes();
        result.frame_count = header->number_of_frames;

        return result;
    }

    // Creates TEST_DIR_PATH on construction and removes it (along with any
    // file left inside it) on destruction, so file_stream_t tests get a
    // clean, scoped place on LittleFS to write to.
    struct lfs_test_dir_fixture_t {
        lfs_test_dir_fixture_t() {
            if (mkdir(TEST_DIR_PATH, 0755) != 0 && errno != EEXIST) {
                TEST_FAIL_MESSAGE("Failed to create test directory on LittleFS");
            }
        }

        ~lfs_test_dir_fixture_t() {
            remove(TEST_FILE_PATH); // Clear out any file so rmdir isn't blocked by ENOTEMPTY
            rmdir(TEST_DIR_PATH);
        }

        lfs_test_dir_fixture_t(const lfs_test_dir_fixture_t&)            = delete;
        lfs_test_dir_fixture_t& operator=(const lfs_test_dir_fixture_t&) = delete;
        lfs_test_dir_fixture_t(lfs_test_dir_fixture_t&&)                 = delete;
        lfs_test_dir_fixture_t& operator=(lfs_test_dir_fixture_t&&)      = delete;
    };

} // namespace

TEST_CASE("Encoder create rejects a mismatched duration_type", "[opus][encoder]") {
    [[maybe_unused]] codec_fixture_t fixture{};

    auto encoder = stream_t<opus::mode_t::ENCODER>::create(get_mismatched_encoder_config());
    TEST_ASSERT_FALSE(encoder.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, encoder.error());
}

TEST_CASE("Decoder create rejects a mismatched duration_type", "[opus][decoder]") {
    [[maybe_unused]] codec_fixture_t fixture{};

    auto decoder = stream_t<opus::mode_t::DECODER>::create(get_mismatched_decoder_config());
    TEST_ASSERT_FALSE(decoder.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, decoder.error());
}

TEST_CASE("Encoder initializes with a valid config and reports a usable frame size", "[opus][encoder]") {
    [[maybe_unused]] codec_fixture_t fixture{};

    auto encoder = stream_t<opus::mode_t::ENCODER>::create(get_encoder_config());
    TEST_ASSERT_TRUE(encoder.has_value());
    TEST_ASSERT_GREATER_THAN_UINT32(0, encoder->get_input_frame_size());
}

TEST_CASE("Encoder is cleaned up correctly by the destructor mid-stream", "[opus][encoder]") {
    // Mirrors the INMP441 destructor test: the interesting failure mode is a
    // hang or crash inside esp_audio_enc_close(...) when the encoder is torn
    // down without ever calling get_stream_header(...) first.
    [[maybe_unused]] codec_fixture_t fixture{};

    {
        auto encoder = stream_t<opus::mode_t::ENCODER>::create(get_encoder_config());
        TEST_ASSERT_TRUE(encoder.has_value());

        auto pcm = make_sine_pcm(encoder->get_input_frame_size() / sizeof(int16_t));

        std::unique_ptr<uint8_t[]> out(new (std::nothrow) uint8_t[OUT_BUF_SIZE_BYTES]);
        TEST_ASSERT_NOT_NULL(out);

        [[maybe_unused]] auto ret =
            encoder->encode({reinterpret_cast<uint8_t*>(pcm.get()), encoder->get_input_frame_size()}, {out.get(), OUT_BUF_SIZE_BYTES});

    } // ~stream_t() runs here

    TEST_PASS();
}

TEST_CASE("Encoder rejects invalid encode arguments", "[opus][encoder]") {
    [[maybe_unused]] codec_fixture_t fixture{};

    auto encoder = stream_t<opus::mode_t::ENCODER>::create(get_encoder_config());
    TEST_ASSERT_TRUE(encoder.has_value());

    std::array<uint8_t, 4096> out{};
    std::array<uint8_t, 512>  pcm{};

    // Empty spans
    auto empty_in  = encoder->encode({}, out);
    auto empty_out = encoder->encode(pcm, {});
    TEST_ASSERT_FALSE(empty_in.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, empty_in.error());
    TEST_ASSERT_FALSE(empty_out.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, empty_out.error());

    // PCM size not a multiple of the required input frame size
    const auto frame_size    = encoder->get_input_frame_size();
    const auto misaligned_sz = frame_size + (frame_size / 2);
    auto*      misaligned    = static_cast<uint8_t*>(heap_caps_malloc(misaligned_sz, MALLOC_CAP_8BIT));
    TEST_ASSERT_NOT_NULL(misaligned);

    auto misaligned_result = encoder->encode({misaligned, misaligned_sz}, out);
    TEST_ASSERT_FALSE(misaligned_result.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, misaligned_result.error());
    heap_caps_free(misaligned);

    // Output buffer too small to even hold the stream header
    std::array<uint8_t, 2> tiny_out{};
    auto*                  one_frame = static_cast<uint8_t*>(heap_caps_malloc(frame_size, MALLOC_CAP_8BIT));
    TEST_ASSERT_NOT_NULL(one_frame);

    auto tiny_out_result = encoder->encode({one_frame, frame_size}, tiny_out);
    TEST_ASSERT_FALSE(tiny_out_result.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, tiny_out_result.error());
    heap_caps_free(one_frame);
}

TEST_CASE("Encoding a full buffer produces a header matching the encoded frames", "[opus][encoder]") {
    [[maybe_unused]] codec_fixture_t fixture{};

    auto stream = build_encoded_stream();

    TEST_ASSERT_GREATER_THAN_UINT32(0, stream.frame_count);
    TEST_ASSERT_GREATER_THAN_UINT32(sizeof(stream_header_t), stream.opus_used);

    // Expect roughly SECONDS_TO_TEST / (FRAME_DURATION_MS / 1000) frames, give or take
    // whatever got rounded off when usable_pcm_bytes was truncated to a whole frame count.
    constexpr uint32_t expected_frames = (SECONDS_TO_TEST * 1000) / FRAME_DURATION_MS;
    TEST_ASSERT_UINT32_WITHIN(1, expected_frames, stream.frame_count);
}

TEST_CASE("Encoder reports partial success when the output buffer is too small to finish", "[opus][encoder]") {
    [[maybe_unused]] codec_fixture_t fixture{};

    auto encoder = stream_t<opus::mode_t::ENCODER>::create(get_encoder_config());
    TEST_ASSERT_TRUE(encoder.has_value());

    const uint32_t frame_size  = encoder->get_input_frame_size();
    const uint32_t frame_count = 20; // Enough frames that a tiny output buffer can't fit them all
    const uint32_t pcm_bytes   = frame_size * frame_count;
    auto           pcm         = make_sine_pcm(pcm_bytes / sizeof(int16_t));

    // Deliberately too small: room for the stream header plus only a couple of frames.
    constexpr uint32_t small_out_sz = sizeof(stream_header_t) + (sizeof(frame_header_t) * 2) + 64;

    auto* small_out = static_cast<uint8_t*>(heap_caps_malloc(small_out_sz, MALLOC_CAP_8BIT));
    TEST_ASSERT_NOT_NULL(small_out);

    auto result = encoder->encode({reinterpret_cast<uint8_t*>(pcm.get()), pcm_bytes}, {small_out, small_out_sz});
    TEST_ASSERT_TRUE_MESSAGE(result.has_value(), "Partial encode should still return a value, not an error");

    const auto& [written, consumed, complete] = result.value();
    TEST_ASSERT_FALSE_MESSAGE(complete, "Encode should report incomplete when the buffer runs out early");
    TEST_ASSERT_LESS_THAN_UINT32(pcm_bytes, consumed);
    TEST_ASSERT_GREATER_THAN_UINT32(0, consumed);

    heap_caps_free(small_out);
}

TEST_CASE("Decoder rejects invalid decode arguments", "[opus][decoder]") {
    [[maybe_unused]] codec_fixture_t fixture{};

    auto decoder = stream_t<opus::mode_t::DECODER>::create(get_decoder_config());
    TEST_ASSERT_TRUE(decoder.has_value());

    auto stream = build_encoded_stream();
    auto source = contiguous_stream_t::create({stream.opus.get(), stream.opus_used});
    TEST_ASSERT_TRUE(source.has_value());

    // Empty output buffer
    auto empty_result = decoder->decode(*source, {});
    TEST_ASSERT_FALSE(empty_result.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, empty_result.error());

    // Output buffer too small to hold even a single decoded PCM frame
    std::array<uint8_t, 2> tiny_out{};
    auto                   tiny_result = decoder->decode(*source, tiny_out);
    TEST_ASSERT_FALSE(tiny_result.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, tiny_result.error());
}

TEST_CASE("Round trip: encoded stream decodes back to the expected PCM length via contiguous_stream_t", "[opus][codec][integration]") {
    [[maybe_unused]] codec_fixture_t fixture{};

    auto stream = build_encoded_stream();

    auto decoder = stream_t<opus::mode_t::DECODER>::create(get_decoder_config());
    TEST_ASSERT_TRUE(decoder.has_value());

    auto source = contiguous_stream_t::create({stream.opus.get(), stream.opus_used});
    TEST_ASSERT_TRUE(source.has_value());

    // Output buffer sized to hold every frame decoded from the source stream.
    constexpr uint32_t samples_per_frame = (SAMPLE_RATE_HZ * FRAME_DURATION_MS) / 1'000;
    const uint32_t     pcm_out_capacity  = stream.frame_count * samples_per_frame * sizeof(int16_t);
    auto*              pcm_out           = static_cast<uint8_t*>(heap_caps_malloc(pcm_out_capacity, MALLOC_CAP_8BIT));
    TEST_ASSERT_NOT_NULL(pcm_out);

    auto decode_result = decoder->decode(*source, {pcm_out, pcm_out_capacity});
    TEST_ASSERT_TRUE_MESSAGE(decode_result.has_value(), "Round-trip decode failed");

    const auto& [decoded, complete] = decode_result.value();
    TEST_ASSERT_TRUE_MESSAGE(complete, "Round-trip decode reported partial success unexpectedly");
    TEST_ASSERT_EQUAL(pcm_out_capacity, decoded.size_bytes());

    heap_caps_free(pcm_out);
}

TEST_CASE("contiguous_stream_t rejects a truncated or empty buffer", "[opus][stream_source]") {
    std::array<uint8_t, 2> tiny{};
    auto                   result = contiguous_stream_t::create(tiny);
    TEST_ASSERT_FALSE(result.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, result.error());
}

TEST_CASE("contiguous_stream_t rejects a zeroed-out stream header", "[opus][stream_source]") {
    std::array<uint8_t, sizeof(stream_header_t) + 32> buf{}; // header fields default to 0

    auto result = contiguous_stream_t::create(buf);
    TEST_ASSERT_FALSE(result.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, result.error());
}

TEST_CASE("contiguous_stream_t iterates exactly the frame count in the header, then reports not found", "[opus][stream_source]") {
    [[maybe_unused]] codec_fixture_t fixture{};

    auto stream = build_encoded_stream();
    auto source = contiguous_stream_t::create({stream.opus.get(), stream.opus_used});
    TEST_ASSERT_TRUE(source.has_value());

    uint32_t counted{};
    while (true) {
        auto frame = source->next();
        if (!frame.has_value()) {
            TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, frame.error());
            break;
        }
        TEST_ASSERT_GREATER_THAN_UINT32(0, frame->frame_header.size_bytes);
        counted++;
    }

    TEST_ASSERT_EQUAL_UINT32(stream.frame_count, counted);

    // Exhausted source should keep reporting not found rather than resurrecting frames
    auto again = source->next();
    TEST_ASSERT_FALSE(again.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, again.error());
}

TEST_CASE("contiguous_stream_t stops rather than trusting a corrupted frame size", "[opus][stream_source]") {
    [[maybe_unused]] codec_fixture_t fixture{};

    auto stream = build_encoded_stream();

    // Corrupt the very first frame's size_bytes field (immediately after the stream header)
    // to something impossibly large. next() should refuse to trust it and bail out cleanly
    // instead of walking the frame head off into unrelated memory.
    frame_header_t corrupt_header{.size_bytes = 0xFFFF'FFFF, .timestamp_ms = 0};
    memcpy(stream.opus.get() + sizeof(stream_header_t), &corrupt_header, sizeof(frame_header_t));

    auto source = contiguous_stream_t::create({stream.opus.get(), stream.opus_used});
    TEST_ASSERT_TRUE(source.has_value());

    auto first = source->next();
    TEST_ASSERT_FALSE(first.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_ALLOWED, first.error());

    // State should be zeroed out after the corrupt frame, not left half-advanced
    auto second = source->next();
    TEST_ASSERT_FALSE(second.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, second.error());
}

TEST_CASE("file_stream_t reports not found for a nonexistent file", "[opus][stream_source][file]") {
    auto result = file_stream_t::create("/lfs/this_file_does_not_exist.bin");
    TEST_ASSERT_FALSE(result.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, result.error());
}

TEST_CASE("file_stream_t round trips a stream written to disk with the same frame count as contiguous_stream_t",
          "[opus][stream_source][file]") {
    [[maybe_unused]] codec_fixture_t        fixture{};
    [[maybe_unused]] lfs_test_dir_fixture_t dir_fixture{};

    auto stream = build_encoded_stream();

    FILE* file = fopen(TEST_FILE_PATH, "wb");
    TEST_ASSERT_NOT_NULL_MESSAGE(file, "Failed to open test file for writing under /lfs/codec");
    TEST_ASSERT_EQUAL(stream.opus_used, fwrite(stream.opus.get(), 1, stream.opus_used, file));
    fclose(file);

    auto file_source = file_stream_t::create(TEST_FILE_PATH);
    TEST_ASSERT_TRUE(file_source.has_value());

    uint32_t counted{};
    while (true) {
        auto frame = file_source->next();
        if (!frame.has_value()) {
            TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, frame.error());
            break;
        }
        counted++;
    }

    TEST_ASSERT_EQUAL_UINT32(stream.frame_count, counted);

    // dir_fixture's destructor removes opus_test_stream.bin and rmdir's /lfs/codec here
}

TEST_CASE("ANALYZE mode rejects invalid or too-small buffers", "[opus][analyze]") {
    // stream_t<> defaults to opus::mode_t::ANALYZE; its header accessors are
    // static and require neither an encoder nor a decoder to be spun up.
    std::array<uint8_t, 2> tiny{};

    auto empty_stream_header = stream_t<>::get_stream_header({});
    TEST_ASSERT_FALSE(empty_stream_header.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, empty_stream_header.error());

    auto small_stream_header = stream_t<>::get_stream_header(tiny);
    TEST_ASSERT_FALSE(small_stream_header.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, small_stream_header.error());

    auto empty_frame_header = stream_t<>::get_frame_header({});
    TEST_ASSERT_FALSE(empty_frame_header.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, empty_frame_header.error());

    auto small_frame_header = stream_t<>::get_frame_header(tiny);
    TEST_ASSERT_FALSE(small_frame_header.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, small_frame_header.error());
}

TEST_CASE("ANALYZE mode parses stream and frame headers from a real encoded stream", "[opus][analyze]") {
    [[maybe_unused]] codec_fixture_t fixture{};

    auto stream = build_encoded_stream();

    auto stream_header = stream_t<>::get_stream_header({stream.opus.get(), stream.opus_used});
    TEST_ASSERT_TRUE(stream_header.has_value());
    TEST_ASSERT_EQUAL_UINT32(stream.frame_count, stream_header->number_of_frames);
    TEST_ASSERT_EQUAL_UINT32(stream.opus_used, stream_header->total_stream_size);

    // First frame header sits immediately after the stream header
    auto frame_header =
        stream_t<>::get_frame_header({stream.opus.get() + sizeof(stream_header_t), stream.opus_used - sizeof(stream_header_t)});
    TEST_ASSERT_TRUE(frame_header.has_value());
    TEST_ASSERT_GREATER_THAN_UINT32(0, frame_header->size_bytes);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(stream_header->largest_opus_frame_size, frame_header->size_bytes);
}
