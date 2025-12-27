#ifndef HEATER_CONTROLLER_H
#define HEATER_CONTROLLER_H

#include "driver/gpio.h"
#include "ZeroCrossDetector.h"

// Configuration for dual heating elements
struct DualHeaterConfig {
    bool enable_dual_elements;      // Enable dual element mode
    float element_power_watts;      // Power per element (W)
    gpio_num_t dimmer_pin;          // GPIO for TRIAC gate (AC dimmer)
    gpio_num_t relay_pin;           // GPIO for relay control
    gpio_num_t zero_cross_pin;      // GPIO for zero-cross detection
    uint32_t ac_frequency_hz;       // AC mains frequency (50 or 60 Hz)
    uint32_t triac_pulse_us;        // TRIAC gate pulse width (microseconds)
};

class HeaterController {
public:
    HeaterController(const DualHeaterConfig& config);
    ~HeaterController();
    
    bool init();
    void setPower(float percentage);  // 0.0 to 1.0
    
    float getPowerPercentage() const { return m_target_power; }
    float getPowerWatts() const;
    float getMaxPower() const;
    
    // Get individual element states
    float getDimmerPercentage() const { return m_dimmer_power; }
    bool getRelayState() const { return m_relay_on; }
    
private:
    void setDimmerPower(float percentage);
    void setRelayState(bool on);
    void dimmerControlTask();
    static void dimmerTaskWrapper(void* arg);
    
    // Calculate RMS power for phase-controlled AC dimmer
    // For phase angle α (0 to π): P_rms/P_max = (1 - α/π + sin(2α)/(2π))
    float phaseAngleToPower(float phase_angle_rad) const;
    float powerToPhaseAngle(float power_percentage) const;
    void initPowerLookupTable();
    
    DualHeaterConfig m_config;
    ZeroCrossDetector* m_zero_cross;
    
    float m_target_power;     // Overall power percentage (0-1)
    float m_dimmer_power;     // Dimmer power percentage (0-1)
    bool m_relay_on;          // Relay state
    
    TaskHandle_t m_dimmer_task;
    bool m_running;
    bool m_initialized;
    
    // Lookup table: power (0.0 to 1.0) -> phase angle (0 to π)
    // 101 entries for 0%, 1%, 2%, ... 100%
    float m_power_to_angle_lut[101];
};

#endif // HEATER_CONTROLLER_H
