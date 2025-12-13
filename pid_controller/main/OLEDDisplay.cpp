#include "OLEDDisplay.h"
#include "esp_log.h"
#include <cstring>
#include <cmath>
#include <cstdio>

static const char *TAG = "OLED";

// SSD1306 Commands
#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF
#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA
#define SSD1306_SETVCOMDETECT 0xDB
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9
#define SSD1306_SETMULTIPLEX 0xA8
#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10
#define SSD1306_SETSTARTLINE 0x40
#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR 0x22
#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8
#define SSD1306_SEGREMAP 0xA0
#define SSD1306_CHARGEPUMP 0x8D

// Simple 5x7 font (only digits, space, dot, and some symbols)
static const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space (32)
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
};

OLEDDisplay::OLEDDisplay(const OLEDConfig& config)
    : m_config(config)
    , m_i2c_bus(nullptr)
    , m_i2c_dev(nullptr)
    , m_buffer(nullptr)
    , m_buffer_size(0)
    , m_initialized(false)
{
    m_buffer_size = (m_config.screen_width * m_config.screen_height) / 8;
}

OLEDDisplay::~OLEDDisplay() {
    if (m_i2c_dev) {
        i2c_master_bus_rm_device(m_i2c_dev);
    }
    if (m_i2c_bus) {
        i2c_del_master_bus(m_i2c_bus);
    }
    if (m_buffer) {
        free(m_buffer);
    }
}

bool OLEDDisplay::init() {
    // Allocate display buffer
    m_buffer = (uint8_t*)malloc(m_buffer_size);
    if (!m_buffer) {
        ESP_LOGE(TAG, "Failed to allocate display buffer");
        return false;
    }
    memset(m_buffer, 0, m_buffer_size);
    
    // Configure I2C bus
    i2c_master_bus_config_t bus_config = {};
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.scl_io_num = m_config.scl_pin;
    bus_config.sda_io_num = m_config.sda_pin;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;
    
    esp_err_t ret = i2c_new_master_bus(&bus_config, &m_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Add device to bus
    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = m_config.i2c_address;
    dev_config.scl_speed_hz = m_config.i2c_freq_hz;
    
    ret = i2c_master_bus_add_device(m_i2c_bus, &dev_config, &m_i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Initialize display
    initDisplay();
    
    m_initialized = true;
    ESP_LOGI(TAG, "OLED display initialized (%dx%d)", m_config.screen_width, m_config.screen_height);
    
    return true;
}

void OLEDDisplay::initDisplay() {
    // Initialization sequence for SSD1306
    sendCommand(SSD1306_DISPLAYOFF);
    sendCommand(SSD1306_SETDISPLAYCLOCKDIV);
    sendCommand(0x80);
    sendCommand(SSD1306_SETMULTIPLEX);
    sendCommand(m_config.screen_height - 1);
    sendCommand(SSD1306_SETDISPLAYOFFSET);
    sendCommand(0x00);
    sendCommand(SSD1306_SETSTARTLINE | 0x00);
    sendCommand(SSD1306_CHARGEPUMP);
    sendCommand(0x14);
    sendCommand(SSD1306_MEMORYMODE);
    sendCommand(0x00);
    sendCommand(SSD1306_SEGREMAP | 0x01);
    sendCommand(SSD1306_COMSCANDEC);
    
    if (m_config.screen_height == 64) {
        sendCommand(SSD1306_SETCOMPINS);
        sendCommand(0x12);
        sendCommand(SSD1306_SETCONTRAST);
        sendCommand(0xCF);
    } else {
        sendCommand(SSD1306_SETCOMPINS);
        sendCommand(0x02);
        sendCommand(SSD1306_SETCONTRAST);
        sendCommand(0x8F);
    }
    
    sendCommand(SSD1306_SETPRECHARGE);
    sendCommand(0xF1);
    sendCommand(SSD1306_SETVCOMDETECT);
    sendCommand(0x40);
    sendCommand(SSD1306_DISPLAYALLON_RESUME);
    sendCommand(SSD1306_NORMALDISPLAY);
    sendCommand(SSD1306_DISPLAYON);
}

bool OLEDDisplay::sendCommand(uint8_t cmd) {
    uint8_t data[2] = {0x00, cmd};  // 0x00 = command mode
    esp_err_t ret = i2c_master_transmit(m_i2c_dev, data, 2, 1000);
    return ret == ESP_OK;
}

bool OLEDDisplay::sendData(const uint8_t* data, size_t len) {
    // Send data in chunks with 0x40 prefix (data mode)
    const size_t chunk_size = 16;
    uint8_t buffer[chunk_size + 1];
    buffer[0] = 0x40;  // Data mode
    
    for (size_t i = 0; i < len; i += chunk_size) {
        size_t chunk_len = (len - i) < chunk_size ? (len - i) : chunk_size;
        memcpy(buffer + 1, data + i, chunk_len);
        
        esp_err_t ret = i2c_master_transmit(m_i2c_dev, buffer, chunk_len + 1, 1000);
        if (ret != ESP_OK) {
            return false;
        }
    }
    return true;
}

void OLEDDisplay::clear() {
    memset(m_buffer, 0, m_buffer_size);
}

void OLEDDisplay::display() {
    sendCommand(SSD1306_COLUMNADDR);
    sendCommand(0);
    sendCommand(m_config.screen_width - 1);
    
    sendCommand(SSD1306_PAGEADDR);
    sendCommand(0);
    sendCommand((m_config.screen_height / 8) - 1);
    
    sendData(m_buffer, m_buffer_size);
}

void OLEDDisplay::setPixel(uint8_t x, uint8_t y, bool on) {
    if (x >= m_config.screen_width || y >= m_config.screen_height) return;
    
    uint16_t index = x + (y / 8) * m_config.screen_width;
    if (on) {
        m_buffer[index] |= (1 << (y & 7));
    } else {
        m_buffer[index] &= ~(1 << (y & 7));
    }
}

void OLEDDisplay::drawChar(uint8_t x, uint8_t y, char c) {
    if (c < 32 || c > 96) return;
    
    const uint8_t* glyph = font5x7[c - 32];
    for (uint8_t i = 0; i < 5; i++) {
        for (uint8_t j = 0; j < 8; j++) {
            if (glyph[i] & (1 << j)) {
                setPixel(x + i, y + j, true);
            }
        }
    }
}

void OLEDDisplay::drawString(uint8_t x, uint8_t y, const char* str) {
    uint8_t pos_x = x;
    while (*str) {
        drawChar(pos_x, y, *str);
        pos_x += 6;  // 5 pixels + 1 spacing
        str++;
    }
}

void OLEDDisplay::drawNumber(uint8_t x, uint8_t y, float number, uint8_t decimals) {
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%.*f", decimals, number);
    drawString(x, y, buffer);
}

void OLEDDisplay::drawProgressBar(uint8_t x, uint8_t y, uint8_t width, uint8_t height, float percent) {
    // Draw border
    for (uint8_t i = 0; i < width; i++) {
        setPixel(x + i, y, true);
        setPixel(x + i, y + height - 1, true);
    }
    for (uint8_t i = 0; i < height; i++) {
        setPixel(x, y + i, true);
        setPixel(x + width - 1, y + i, true);
    }
    
    // Fill bar
    uint8_t fill_width = (uint8_t)((width - 2) * percent / 100.0f);
    for (uint8_t i = 0; i < fill_width; i++) {
        for (uint8_t j = 1; j < height - 1; j++) {
            setPixel(x + 1 + i, y + j, true);
        }
    }
}

void OLEDDisplay::updateStatus(float setpoint, float current_temp, float output_percent,
                               float dimmer_percent, bool relay_on) {
    clear();
    
    // Line 1: Setpoint
    drawString(0, 0, "SP:");
    drawNumber(24, 0, setpoint, 1);
    drawString(60, 0, "C");
    
    // Line 2: Current temperature
    drawString(0, 10, "T:");
    drawNumber(18, 10, current_temp, 1);
    drawString(54, 10, "C");
    
    // Line 3: Total output
    drawString(0, 20, "Out:");
    drawNumber(30, 20, output_percent, 0);
    drawString(54, 20, "%");
    
    // Line 4: Dimmer
    drawString(0, 30, "Dim:");
    drawNumber(30, 30, dimmer_percent, 0);
    drawString(54, 30, "%");
    
    // Line 5: Relay status
    drawString(0, 40, "Relay:");
    drawString(42, 40, relay_on ? "ON " : "OFF");
    
    // Progress bar for total output
    drawProgressBar(0, 52, 127, 10, output_percent);
    
    display();
}
