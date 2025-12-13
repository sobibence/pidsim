#include "MAX6675Sensor.h"
#include "esp_log.h"
#include <cmath>

static const char *TAG = "MAX6675";

MAX6675Sensor::MAX6675Sensor(gpio_num_t sck, gpio_num_t cs, gpio_num_t miso)
    : m_sck(sck)
    , m_cs(cs)
    , m_miso(miso)
    , m_spi(nullptr)
    , m_initialized(false)
    , m_connected(false)
    , m_offset(0.0f)
    , m_scale(1.0f)
{
}

MAX6675Sensor::~MAX6675Sensor() {
    if (m_initialized && m_spi) {
        spi_bus_remove_device(m_spi);
        spi_bus_free(SPI2_HOST);
    }
}

bool MAX6675Sensor::init() {
    // Configure SPI bus
    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = -1;  // MAX6675 is read-only
    bus_config.miso_io_num = m_miso;
    bus_config.sclk_io_num = m_sck;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = 0;
    
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Configure SPI device (MAX6675)
    spi_device_interface_config_t dev_config = {};
    dev_config.clock_speed_hz = 1 * 1000 * 1000;  // 1 MHz (MAX6675 max is 4.3 MHz)
    dev_config.spics_io_num = m_cs;
    dev_config.queue_size = 1;
    
    ret = spi_bus_add_device(SPI2_HOST, &dev_config, &m_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        spi_bus_free(SPI2_HOST);
        return false;
    }
    
    m_initialized = true;
    ESP_LOGI(TAG, "MAX6675 initialized successfully");
    return true;
}

float MAX6675Sensor::readCelsius() {
    if (!m_initialized) {
        ESP_LOGE(TAG, "Sensor not initialized");
        return std::nan("");
    }
    
    uint8_t rx_data[2] = {0};
    
    spi_transaction_t transaction = {};
    transaction.length = 16;  // 16 bits
    transaction.rx_buffer = rx_data;
    
    esp_err_t ret = spi_device_transmit(m_spi, &transaction);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transaction failed: %s", esp_err_to_name(ret));
        m_connected = false;
        return std::nan("");
    }
    
    // Combine the two bytes
    uint16_t value = (rx_data[0] << 8) | rx_data[1];
    
    // Check if thermocouple is connected (bit 2 should be 0)
    if (value & 0x04) {
        ESP_LOGW(TAG, "Thermocouple not connected");
        m_connected = false;
        return std::nan("");
    }
    
    m_connected = true;
    
    // Extract temperature (bits 15-3)
    value >>= 3;
    
    // Convert to Celsius (0.25°C per bit)
    float raw_temp = value * 0.25f;
    
    // Apply calibration: final = (raw * scale) + offset
    return (raw_temp * m_scale) + m_offset;
}

void MAX6675Sensor::setCalibration(float offset, float scale) {
    m_offset = offset;
    m_scale = scale;
    ESP_LOGI(TAG, "Calibration set: offset=%.2f°C, scale=%.4f", offset, scale);
}
