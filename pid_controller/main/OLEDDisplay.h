#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include <string>

// Display configuration
struct OLEDConfig {
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    uint8_t i2c_address;
    uint32_t i2c_freq_hz;
    uint8_t screen_width;   // Usually 128
    uint8_t screen_height;  // Usually 64 or 32
};

class OLEDDisplay {
public:
    OLEDDisplay(const OLEDConfig& config);
    ~OLEDDisplay();
    
    bool init();
    void clear();
    void display();
    
    // Update display with current system status
    void updateStatus(float setpoint, float current_temp, float output_percent, 
                     float dimmer_percent, bool relay_on);
    
    // Low-level drawing functions
    void drawString(uint8_t x, uint8_t y, const char* str);
    void drawNumber(uint8_t x, uint8_t y, float number, uint8_t decimals = 1);
    void drawProgressBar(uint8_t x, uint8_t y, uint8_t width, uint8_t height, float percent);
    void setPixel(uint8_t x, uint8_t y, bool on = true);
    
private:
    bool sendCommand(uint8_t cmd);
    bool sendData(const uint8_t* data, size_t len);
    void initDisplay();
    
    // Character rendering
    void drawChar(uint8_t x, uint8_t y, char c);
    
    OLEDConfig m_config;
    i2c_master_bus_handle_t m_i2c_bus;
    i2c_master_dev_handle_t m_i2c_dev;
    
    uint8_t* m_buffer;
    uint16_t m_buffer_size;
    
    bool m_initialized;
};

#endif // OLED_DISPLAY_H
