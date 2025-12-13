#ifndef MAX6675_SENSOR_H
#define MAX6675_SENSOR_H

#include "driver/spi_master.h"
#include "driver/gpio.h"

class MAX6675Sensor {
public:
    MAX6675Sensor(gpio_num_t sck, gpio_num_t cs, gpio_num_t miso);
    ~MAX6675Sensor();
    
    bool init();
    float readCelsius();
    bool isConnected() const { return m_connected; }
    
    // Calibration: final_temp = (raw_temp * scale) + offset
    void setCalibration(float offset, float scale = 1.0f);
    float getOffset() const { return m_offset; }
    float getScale() const { return m_scale; }
    
private:
    gpio_num_t m_sck;
    gpio_num_t m_cs;
    gpio_num_t m_miso;
    
    spi_device_handle_t m_spi;
    bool m_initialized;
    bool m_connected;
    
    // Calibration parameters
    float m_offset;  // Temperature offset in °C
    float m_scale;   // Temperature scale factor
};

#endif // MAX6675_SENSOR_H
