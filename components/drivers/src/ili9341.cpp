#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ili9341.hpp"
#include "utils.hpp"

#include "esp_attr.h"
#include "esp_log.h"

#include <array>
#include <utility>

namespace disp {

    ili9341_t::~ili9341_t() noexcept {
        if (m_is_initialized) {
            cleanup_resources();
        }
    }

    esp_err_t ili9341_t::init(const config_t& config) {
        if (m_is_initialized) {
            return ESP_ERR_INVALID_STATE;
        }

        m_config = config;

        // Configure DC and RESET pins
        const gpio_config_t pin_config = {
            .pin_bit_mask =
                static_cast<uint64_t>(1ULL << std::to_underlying(m_config.dc_pin) | 1ULL << std::to_underlying(m_config.rst_pin)),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };

        auto ret = gpio_config(&pin_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure RST and DC pins: %s", esp_err_to_name(ret));
            return ret;
        }

        // Configure SPI device
        const spi_device_interface_config_t dev_cfg = {
            .command_bits     = 0,
            .address_bits     = 0,
            .dummy_bits       = 0,
            .mode             = 0, // The ILI9341 accepts a CPOL-CPHA of 0-0
            .clock_source     = SPI_CLK_SRC_DEFAULT,
            .duty_cycle_pos   = 128, // Default param
            .cs_ena_pretrans  = 0,
            .cs_ena_posttrans = 0,
            .clock_speed_hz   = static_cast<int>(m_config.spi_clock_speed_hz),
            .input_delay_ns   = 0,
            .sample_point     = SPI_SAMPLING_POINT_PHASE_0,
            .spics_io_num     = m_config.cs_pin,
            .flags            = 0,
            .queue_size       = TRANS_QUEUE_SIZE,
            .pre_cb           = nullptr,
            .post_cb          = nullptr,
        };

        ret = spi_bus_add_device(m_config.spi_host, &dev_cfg, &m_device_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI device configure failed: %s", esp_err_to_name(ret));
            cleanup_resources();
            return ret;
        }

        // Hardware reset
        // Toggle the reset pin
        gpio_set_level(m_config.rst_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(m_config.rst_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(120));

        // Send initialization sequence to the ili9341
        ret = init_sequence();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to transmit the initialization sequence: %s", esp_err_to_name(ret));
            cleanup_resources();
            return ret;
        }

        m_is_initialized = true;
        ESP_LOGI(TAG, "Initialization complete");

        return ESP_OK;
    }

    esp_err_t ili9341_t::deinit() {
        if (!m_is_initialized) {
            return ESP_ERR_INVALID_STATE;
        }

        cleanup_resources();
        m_is_initialized = false;

        return ESP_OK;
    }

    esp_err_t ili9341_t::flush(const coord_t& coord, std::span<const uint16_t> data) {
        if (!m_is_initialized) {
            return ESP_ERR_INVALID_STATE;
        }

        if (data.data() == nullptr || data.size() == 0) {
            return ESP_ERR_INVALID_ARG;
        }

        if (m_config.rotation == 0 || m_config.rotation == 2) {
            // Potrait mode
            if (coord.x1 >= MAX_WIDTH || coord.x2 >= MAX_WIDTH || coord.x1 > coord.x2 || coord.y1 >= MAX_HEIGHT || coord.y2 >= MAX_HEIGHT ||
                coord.y1 > coord.y2) {
                return ESP_ERR_INVALID_ARG;
            }
        } else if (m_config.rotation == 1 || m_config.rotation == 3) {
            // Landscape mode on the hardware level: the coordinates get flipped, so shouldn't block
            // the flushes on what would appear to be out of bounds accesses. x is height, y is width
            if (coord.x1 >= MAX_HEIGHT || coord.x2 >= MAX_HEIGHT || coord.x1 > coord.x2 || coord.y1 >= MAX_WIDTH || coord.y2 >= MAX_WIDTH ||
                coord.y1 > coord.y2) {
                return ESP_ERR_INVALID_ARG;
            }
        } else {
            return ESP_ERR_INVALID_STATE;
        }

        // Set the pixel window
        TRY(set_window(coord));

        // Send the memory write command
        TRY(send_cmd(0x2CU));

        // Send the pixel data. The send_data(...) function expects uint8_t, so the
        // byte count has to be multiplied by two since the data here is uint16_t
        TRY(send_data({reinterpret_cast<const uint8_t*>(data.data()), (data.size() * 2)}));

        return ESP_OK;
    }

    esp_err_t ili9341_t::set_screen(uint16_t color, bool little_endian) {
        if (!m_is_initialized) {
            return ESP_ERR_INVALID_STATE;
        }

        // Calculate buffer size to use to fill the display
        constexpr size_t total_mem_needed_bytes{MAX_WIDTH * MAX_HEIGHT * 2};
        constexpr size_t num_of_times_to_send_color_buf{80};
        constexpr size_t mem_allocated_bytes{total_mem_needed_bytes / num_of_times_to_send_color_buf};
        constexpr size_t mem_for_16_bit_data{mem_allocated_bytes / 2};

        // Allocate a buffer to store the data for the color
        WORD_ALIGNED_ATTR std::array<uint16_t, mem_for_16_bit_data> buf{};

        // Swap the byte order if the data in `color` is little endian
        if (little_endian) [[likely]] {
            color = __builtin_bswap16(color);
        }

        // Fill the buffer with the color
        buf.fill(color);

        // Get offsets
        constexpr auto offset{MAX_HEIGHT / num_of_times_to_send_color_buf};
        uint16_t       y1{};
        uint16_t       y2{offset - 1};
        esp_err_t      ret{ESP_OK};

        for (size_t i{}; i < num_of_times_to_send_color_buf; i++) {
            ret = flush({0, y1, MAX_WIDTH - 1, y2}, buf);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to transmit color buffer on %zu iteration: %s", i, esp_err_to_name(ret));
                break;
            }
            // Get new height offsets
            y1 = y2 + 1;
            y2 += offset;
        }

        return ret;
    }

    // Helpers
    void ili9341_t::cleanup_resources() {
        if (m_device_handle) {
            spi_bus_remove_device(m_device_handle);
            m_device_handle = nullptr;
        }
        gpio_reset_pin(m_config.dc_pin);
        gpio_reset_pin(m_config.rst_pin);
        m_config = {};
    }

    esp_err_t ili9341_t::init_sequence() {
        // Commands and data bytes to initialize the ILI9341.
        // Too many to identify all. For more details, refer to the datasheet.
        constexpr uint8_t data_1[] = {0x03, 0x80, 0x02};
        TRY(send_cmd(0xEFU));
        TRY(send_data({data_1, sizeof(data_1)}));

        constexpr uint8_t data_2[] = {0x00, 0xC1, 0x30};
        TRY(send_cmd(0xCFU));
        TRY(send_data({data_2, sizeof(data_2)}));

        constexpr uint8_t data_3[] = {0x64, 0x03, 0x12, 0x81};
        TRY(send_cmd(0xEDU));
        TRY(send_data({data_3, sizeof(data_3)}));

        constexpr uint8_t data_4[] = {0x85, 0x00, 0x78};
        TRY(send_cmd(0xE8U));
        TRY(send_data({data_4, sizeof(data_4)}));

        const uint8_t data_5[] = {0x39, 0x2C, 0x00, 0x34, 0x02};
        TRY(send_cmd(0xCBU));
        TRY(send_data({data_5, sizeof(data_5)}));

        constexpr uint8_t data_6[] = {0x20};
        TRY(send_cmd(0xF7U));
        TRY(send_data({data_6, sizeof(data_6)}));

        constexpr uint8_t data_7[] = {0x00, 0x00};
        TRY(send_cmd(0xE8U));
        TRY(send_data({data_7, sizeof(data_7)}));

        constexpr uint8_t data_8[] = {0x23};
        TRY(send_cmd(0xC0U));
        TRY(send_data({data_8, sizeof(data_8)}));

        constexpr uint8_t data_9[] = {0x10};
        TRY(send_cmd(0xC1U));
        TRY(send_data({data_9, sizeof(data_9)}));

        constexpr uint8_t data_10[] = {0x3E, 0x28};
        TRY(send_cmd(0xC5U));
        TRY(send_data({data_10, sizeof(data_10)}));

        constexpr uint8_t data_11[] = {0x86};
        TRY(send_cmd(0xC7U));
        TRY(send_data({data_11, sizeof(data_11)}));

        constexpr uint8_t data_12[] = {0x00};
        TRY(send_cmd(0x37U));
        TRY(send_data({data_12, sizeof(data_12)}));

        constexpr uint8_t data_13[] = {0x55};
        TRY(send_cmd(0x3AU));
        TRY(send_data({data_13, sizeof(data_13)}));

        constexpr uint8_t data_14[] = {0x00, 0x18};
        TRY(send_cmd(0xB1U));
        TRY(send_data({data_14, sizeof(data_14)}));

        constexpr uint8_t data_15[] = {0x08, 0x82, 0x27};
        TRY(send_cmd(0xB6U));
        TRY(send_data({data_15, sizeof(data_15)}));

        const uint8_t data_16[] = {0x00};
        TRY(send_cmd(0xF2U));
        TRY(send_data({data_16, sizeof(data_16)}));

        const uint8_t data_17[] = {0x01};
        TRY(send_cmd(0x26U));
        TRY(send_data({data_17, sizeof(data_17)}));

        const uint8_t data_18[] = {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00};
        TRY(send_cmd(0xE0U));
        TRY(send_data({data_18, sizeof(data_18)}));

        const uint8_t data_19[] = {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F};
        TRY(send_cmd(0xE1U));
        TRY(send_data({data_19, sizeof(data_19)}));

        // Memory access control (rotation)
        uint8_t mem_acc_ctrl[] = {0x00};
        switch (m_config.rotation) {
            case 0:
                mem_acc_ctrl[0] = 0x08;
                break;
            case 1:
                mem_acc_ctrl[0] = 0x48;
                break;
            case 2:
                mem_acc_ctrl[0] = 0x88;
                break;
            case 3:
                mem_acc_ctrl[0] = 0xB8;
                break;
            default:
                return ESP_ERR_INVALID_ARG;
        }
        TRY(send_cmd(0x36U));
        TRY(send_data({mem_acc_ctrl, sizeof(mem_acc_ctrl)}));

        // Display inversion OFF
        TRY(send_cmd(0x20U));

        // Exit sleep
        TRY(send_cmd(0x11U));
        vTaskDelay(pdMS_TO_TICKS(150));

        // Display ON
        TRY(send_cmd(0x29U));
        vTaskDelay(pdMS_TO_TICKS(20));

        return ESP_OK;
    }

    esp_err_t ili9341_t::send_cmd(uint8_t cmd) {

        gpio_set_level(m_config.dc_pin, 0); // Command mode

        spi_transaction_t trans = {
            .flags            = 0,
            .cmd              = 0,
            .addr             = 0,
            .length           = 8, // 8 bits
            .rxlength         = 0, // We are only transmitting
            .override_freq_hz = 0,
            .user             = nullptr,
            .tx_buffer        = &cmd,
            .rx_buffer        = nullptr,
        };

        return spi_device_transmit(m_device_handle, &trans);
    }

    esp_err_t ili9341_t::set_window(const coord_t& coord) {

        // The ILI9341 uses big endian alignment so we send high byte first
        // Column address set
        const std::array<uint8_t, 4> caset_data = {
            static_cast<uint8_t>((coord.x1 >> 8) & 0xFFU),
            static_cast<uint8_t>(coord.x1 & 0xFFU),
            static_cast<uint8_t>((coord.x2 >> 8U) & 0xFFU),
            static_cast<uint8_t>(coord.x2 & 0xFFU),
        };
        TRY(send_cmd(0x2AU));
        TRY(send_data(caset_data));

        // Row address set
        const std::array<uint8_t, 4> raset_data = {
            static_cast<uint8_t>((coord.y1 >> 8) & 0xFF),
            static_cast<uint8_t>(coord.y1 & 0xFF),
            static_cast<uint8_t>((coord.y2 >> 8) & 0xFF),
            static_cast<uint8_t>(coord.y2 & 0xFF),
        };
        TRY(send_cmd(0x2BU));
        TRY(send_data(raset_data));

        return ESP_OK;
    }

    esp_err_t ili9341_t::send_data(std::span<const uint8_t> data) {

        gpio_set_level(m_config.dc_pin, 1); // Data mode

        spi_transaction_t trans = {
            .flags            = 0,
            .cmd              = 0,
            .addr             = 0,
            .length           = data.size() * 8, // Data length in bits
            .rxlength         = 0,               // We are only transmitting
            .override_freq_hz = 0,
            .user             = nullptr,
            .tx_buffer        = data.data(),
            .rx_buffer        = nullptr,
        };

        return spi_device_transmit(m_device_handle, &trans);
    }

} // namespace disp
