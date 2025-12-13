#ifndef ZERO_CROSS_DETECTOR_H
#define ZERO_CROSS_DETECTOR_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class ZeroCrossDetector {
public:
    ZeroCrossDetector(gpio_num_t zc_pin, uint32_t ac_frequency_hz = 50);
    ~ZeroCrossDetector();
    
    bool init();
    
    // Wait for next zero cross event (blocking)
    bool waitForZeroCross(uint32_t timeout_ms = 100);
    
    // Get time since last zero cross in microseconds
    uint32_t getTimeSinceZeroCross() const;
    
    // Get the half-period duration in microseconds
    uint32_t getHalfPeriod() const { return m_half_period_us; }
    
    // Get AC frequency
    uint32_t getFrequency() const { return m_ac_frequency; }
    
private:
    static void IRAM_ATTR zeroCrossISR(void* arg);
    
    gpio_num_t m_zc_pin;
    uint32_t m_ac_frequency;
    uint32_t m_half_period_us;
    
    SemaphoreHandle_t m_zc_semaphore;
    volatile uint64_t m_last_zc_time;
    
    bool m_initialized;
};

#endif // ZERO_CROSS_DETECTOR_H
