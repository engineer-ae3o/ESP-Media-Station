#include "unity.h"

#include "esp_heap_caps.h"

#include "codec.hpp"

#include <array>
#include <cmath>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <numbers>
#include <cstdint>
#include <unistd.h>
#include <sys/stat.h>

namespace {

    using namespace audio::codec::opus;

    constexpr int SAMPLE_RATE_HZ    = 48'000;
    constexpr int FRAME_DURATION_MS = 20;
    constexpr int SECONDS_TO_TEST   = 2;

    // LittleFS is mounted at /lfs. Tests that touch disk get their own
    // subdirectory so they don't scatter files across the mount, and that
    // directory is torn down again once those tests are done with it.
    constexpr const char* TEST_DIR_PATH  = "/lfs/codec";
    constexpr const char* TEST_FILE_PATH = "/lfs/codec/opus_test_stream.bin";

    consteval auto get_encoder_config() {
        return config_t{
            .bit_rate          = 40'000,
            .complexity        = 4,
            .sample_rate       = SAMPLE_RATE_HZ,
            .frame_duration_ms = FRAME_DURATION_MS,
            .mode              = ESP_OPUS_ENC_APPLICATION_VOIP,
            .duration_type     = ESP_OPUS_ENC_FRAME_DURATION_20_MS,
        };
    }

    consteval auto get_decoder_config() {
        return config_t{
            .bit_rate          = 40'000,
            .complexity        = 4,
            .sample_rate       = SAMPLE_RATE_HZ,
            .frame_duration_ms = FRAME_DURATION_MS,
            .mode              = ESP_OPUS_ENC_APPLICATION_VOIP, // Not needed in decoder mode
            .duration_type     = ESP_OPUS_DEC_FRAME_DURATION_20_MS,
        };
    }

    // Deliberately hands the encoder a decoder-shaped duration_type (and vice
    // versa) to exercise the std::get_if(...) rejection path in start(...).
    consteval config_t get_mismatched_encoder_config() {
        config_t cfg      = get_encoder_config();
        cfg.duration_type = ESP_OPUS_DEC_FRAME_DURATION_20_MS;
        return cfg;
    }

    consteval config_t get_mismatched_decoder_config() {
        config_t cfg      = get_decoder_config();
        cfg.duration_type = ESP_OPUS_ENC_FRAME_DURATION_20_MS;
        return cfg;
    }

    // Registers/unregisters the codec library once per test.
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

    int16_t* make_sine_pcm(size_t sample_count) {
        constexpr float freq_hz = 440;
        constexpr float pi      = std::numbers::pi_v<float>;

        auto* buf = static_cast<int16_t*>(heap_caps_malloc(sample_count * sizeof(int16_t), MALLOC_CAP_8BIT));
        TEST_ASSERT_NOT_NULL_MESSAGE(buf, "Failed to allocate PCM buffer for opus test");

        for (size_t i = 0; i < sample_count; i++) {
            const auto sample = std::sin(2.0F * pi * freq_hz * static_cast<float>(i) / static_cast<float>(SAMPLE_RATE_HZ));
            buf[i]            = static_cast<int16_t>(sample * static_cast<float>(INT16_MAX));
        }

        return buf;
    }

    // Encodes SECONDS_TO_TEST worth of sine PCM into a freshly allocated,
    // fully headered opus stream buffer. Caller owns both the PCM and opus
    // buffers and must heap_caps_free(...) them.
    struct encoded_stream_t {
        int16_t* pcm{};
        uint8_t* opus{};
        size_t   opus_capacity{};
        size_t   opus_used{};
        uint32_t frame_count{};
    };

    encoded_stream_t build_encoded_stream() {
        auto encoder = stream_t<stream_mode_t::ENCODER>::create(get_encoder_config());
        TEST_ASSERT_TRUE_MESSAGE(encoder.has_value(), "Failed to create encoder while building test fixture stream");

        const auto pcm_frame_size  = encoder->get_input_frame_size();
        const auto total_samples   = static_cast<size_t>(SAMPLE_RATE_HZ) * SECONDS_TO_TEST;
        const auto total_pcm_bytes = total_samples * sizeof(int16_t);

        // Round down to a whole number of input frames, same constraint encode(...) enforces.
        const auto usable_pcm_bytes = (total_pcm_bytes / pcm_frame_size) * pcm_frame_size;

        encoded_stream_t result{};
        result.pcm           = make_sine_pcm(usable_pcm_bytes / sizeof(int16_t));
        result.opus_capacity = usable_pcm_bytes + (1024 * 1024 * sizeof(frame_header_t)); // generous headroom
        result.opus          = static_cast<uint8_t*>(heap_caps_malloc(result.opus_capacity, MALLOC_CAP_8BIT));
        TEST_ASSERT_NOT_NULL_MESSAGE(result.opus, "Failed to allocate opus output buffer for test fixture stream");

        auto encode_result =
            encoder->encode({reinterpret_cast<uint8_t*>(result.pcm), usable_pcm_bytes}, {result.opus, result.opus_capacity});
        TEST_ASSERT_TRUE_MESSAGE(encode_result.has_value(), "Encoding the fixture stream failed");

        const auto& [written, consumed, complete] = encode_result.value();
        TEST_ASSERT_TRUE_MESSAGE(complete, "Fixture stream encode reported partial success unexpectedly");
        TEST_ASSERT_EQUAL_MESSAGE(usable_pcm_bytes, consumed, "Fixture stream did not consume the full PCM buffer");

        const auto header = encoder->get_stream_header();
        TEST_ASSERT_TRUE_MESSAGE(header.has_value(), "Failed to retrieve stream header for fixture stream");

        // Stamp the finalized header at the head of the stream, as documented on get_stream_header(...).
        memcpy(result.opus, &header.value(), sizeof(stream_header_t));

        result.opus_used   = written.size_bytes();
        result.frame_count = header->number_of_frames;
        return result;
    }

    void free_encoded_stream(encoded_stream_t& stream) {
        heap_caps_free(stream.pcm);
        heap_caps_free(stream.opus);
        stream = {};
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

    auto encoder = stream_t<stream_mode_t::ENCODER>::create(get_mismatched_encoder_config());
    TEST_ASSERT_FALSE(encoder.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, encoder.error());
}

TEST_CASE("Decoder create rejects a mismatched duration_type", "[opus][decoder]") {
    [[maybe_unused]] codec_fixture_t fixture{};

    auto decoder = stream_t<stream_mode_t::DECODER>::create(get_mismatched_decoder_config());
    TEST_ASSERT_FALSE(decoder.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, decoder.error());
}

TEST_CASE("Encoder initializes with a valid config and reports a usable frame size", "[opus][encoder]") {
    [[maybe_unused]] codec_fixture_t fixture{};

    auto encoder = stream_t<stream_mode_t::ENCODER>::create(get_encoder_config());
    TEST_ASSERT_TRUE(encoder.has_value());
    TEST_ASSERT_GREATER_THAN_UINT32(0, encoder->get_input_frame_size());
}

TEST_CASE("Encoder is cleaned up correctly by the destructor mid-stream", "[opus][encoder]") {
    // Mirrors the INMP441 destructor test: the interesting failure mode is a
    // hang or crash inside esp_audio_enc_close(...) when the encoder is torn
    // down without ever calling get_stream_header(...) first.
    [[maybe_unused]] codec_fixture_t fixture{};

    {
        auto encoder = stream_t<stream_mode_t::ENCODER>::create(get_encoder_config());
        TEST_ASSERT_TRUE(encoder.has_value());

        auto* pcm = make_sine_pcm(encoder->get_input_frame_size() / sizeof(int16_t));
        auto* out = static_cast<uint8_t*>(heap_caps_malloc(4096, MALLOC_CAP_8BIT));
        TEST_ASSERT_NOT_NULL(out);

        [[maybe_unused]] auto ret = encoder->encode({reinterpret_cast<uint8_t*>(pcm), encoder->get_input_frame_size()}, {out, 4096});

        heap_caps_free(pcm);
        heap_caps_free(out);
    } // ~stream_t() runs here

    TEST_PASS();
}

TEST_CASE("Encoder rejects invalid encode arguments", "[opus][encoder]") {
    [[maybe_unused]] codec_fixture_t fixture{};

    auto encoder = stream_t<stream_mode_t::ENCODER>::create(get_encoder_config());
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

    free_encoded_stream(stream);
}

TEST_CASE("Encoder reports partial success when the output buffer is too small to finish", "[opus][encoder]") {
    [[maybe_unused]] codec_fixture_t fixture{};

    auto encoder = stream_t<stream_mode_t::ENCODER>::create(get_encoder_config());
    TEST_ASSERT_TRUE(encoder.has_value());

    const auto frame_size  = encoder->get_input_frame_size();
    const auto frame_count = 20U; // Enough frames that a tiny output buffer can't fit them all
    const auto pcm_bytes   = static_cast<size_t>(frame_size) * frame_count;
    auto*      pcm         = make_sine_pcm(pcm_bytes / sizeof(int16_t));

    // Deliberately too small: room for the stream header plus only a couple of frames.
    const size_t small_out_sz = sizeof(stream_header_t) + (sizeof(frame_header_t) * 2) + 64;
    auto*        small_out    = static_cast<uint8_t*>(heap_caps_malloc(small_out_sz, MALLOC_CAP_8BIT));
    TEST_ASSERT_NOT_NULL(small_out);

    auto result = encoder->encode({reinterpret_cast<uint8_t*>(pcm), pcm_bytes}, {small_out, small_out_sz});
    TEST_ASSERT_TRUE_MESSAGE(result.has_value(), "Partial encode should still return a value, not an error");

    const auto& [written, consumed, complete] = result.value();
    TEST_ASSERT_FALSE_MESSAGE(complete, "Encode should report incomplete when the buffer runs out early");
    TEST_ASSERT_LESS_THAN_UINT32(pcm_bytes, consumed);
    TEST_ASSERT_GREATER_THAN_UINT32(0, consumed);

    heap_caps_free(pcm);
    heap_caps_free(small_out);
}

TEST_CASE("Decoder rejects invalid decode arguments", "[opus][decoder]") {
    [[maybe_unused]] codec_fixture_t fixture{};

    auto decoder = stream_t<stream_mode_t::DECODER>::create(get_decoder_config());
    TEST_ASSERT_TRUE(decoder.has_value());

    auto stream = build_encoded_stream();
    auto source = contiguous_stream_t::create({stream.opus, stream.opus_used});
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

    free_encoded_stream(stream);
}

TEST_CASE("Round trip: encoded stream decodes back to the expected PCM length via contiguous_stream_t", "[opus][codec][integration]") {
    [[maybe_unused]] codec_fixture_t fixture{};

    auto stream = build_encoded_stream();

    auto decoder = stream_t<stream_mode_t::DECODER>::create(get_decoder_config());
    TEST_ASSERT_TRUE(decoder.has_value());

    auto source = contiguous_stream_t::create({stream.opus, stream.opus_used});
    TEST_ASSERT_TRUE(source.has_value());

    // Output buffer sized to hold every frame decoded from the source stream.
    constexpr uint32_t samples_per_frame = (SAMPLE_RATE_HZ * FRAME_DURATION_MS) / 1'000;
    const size_t       pcm_out_capacity  = static_cast<size_t>(stream.frame_count) * samples_per_frame * sizeof(int16_t);
    auto*              pcm_out           = static_cast<uint8_t*>(heap_caps_malloc(pcm_out_capacity, MALLOC_CAP_8BIT));
    TEST_ASSERT_NOT_NULL(pcm_out);

    auto decode_result = decoder->decode(*source, {pcm_out, pcm_out_capacity});
    TEST_ASSERT_TRUE_MESSAGE(decode_result.has_value(), "Round-trip decode failed");

    const auto& [decoded, complete] = decode_result.value();
    TEST_ASSERT_TRUE_MESSAGE(complete, "Round-trip decode reported partial success unexpectedly");
    TEST_ASSERT_EQUAL(pcm_out_capacity, decoded.size_bytes());

    heap_caps_free(pcm_out);
    free_encoded_stream(stream);
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
    auto source = contiguous_stream_t::create({stream.opus, stream.opus_used});
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

    free_encoded_stream(stream);
}

TEST_CASE("contiguous_stream_t stops rather than trusting a corrupted frame size", "[opus][stream_source]") {
    [[maybe_unused]] codec_fixture_t fixture{};

    auto stream = build_encoded_stream();

    // Corrupt the very first frame's size_bytes field (immediately after the stream header)
    // to something impossibly large. next() should refuse to trust it and bail out cleanly
    // instead of walking the frame head off into unrelated memory.
    frame_header_t corrupt_header{.size_bytes = 0xFFFF'FFFF, .timestamp_ms = 0};
    memcpy(stream.opus + sizeof(stream_header_t), &corrupt_header, sizeof(frame_header_t));

    auto source = contiguous_stream_t::create({stream.opus, stream.opus_used});
    TEST_ASSERT_TRUE(source.has_value());

    auto first = source->next();
    TEST_ASSERT_FALSE(first.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_ALLOWED, first.error());

    // State should be zeroed out after the corrupt frame, not left half-advanced
    auto second = source->next();
    TEST_ASSERT_FALSE(second.has_value());
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, second.error());

    free_encoded_stream(stream);
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
    TEST_ASSERT_EQUAL(stream.opus_used, fwrite(stream.opus, 1, stream.opus_used, file));
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

    free_encoded_stream(stream);
    // dir_fixture's destructor removes opus_test_stream.bin and rmdir's /lfs/codec here
}

TEST_CASE("ANALYZE mode rejects invalid or too-small buffers", "[opus][analyze]") {
    // stream_t<> defaults to stream_mode_t::ANALYZE; its header accessors are
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

    auto stream_header = stream_t<>::get_stream_header({stream.opus, stream.opus_used});
    TEST_ASSERT_TRUE(stream_header.has_value());
    TEST_ASSERT_EQUAL_UINT32(stream.frame_count, stream_header->number_of_frames);
    TEST_ASSERT_EQUAL_UINT32(stream.opus_used, stream_header->total_stream_size);

    // First frame header sits immediately after the stream header
    auto frame_header = stream_t<>::get_frame_header({stream.opus + sizeof(stream_header_t), stream.opus_used - sizeof(stream_header_t)});
    TEST_ASSERT_TRUE(frame_header.has_value());
    TEST_ASSERT_GREATER_THAN_UINT32(0, frame_header->size_bytes);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(stream_header->largest_opus_frame_size, frame_header->size_bytes);

    free_encoded_stream(stream);
}
