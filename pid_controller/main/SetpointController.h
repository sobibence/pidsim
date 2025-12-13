#ifndef SETPOINT_CONTROLLER_H
#define SETPOINT_CONTROLLER_H

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

class SetpointController {
public:
    SetpointController(adc_channel_t adc_channel, 
                       float min_temp, 
                       float max_temp,
                       adc_atten_t attenuation = ADC_ATTEN_DB_12);
    ~SetpointController();
    
    bool init();
    float readSetpoint();  // Returns temperature setpoint in °C
    
    // Get raw ADC value (0-4095 for 12-bit)
    int getRawADC() const { return m_last_raw_value; }
    
private:
    adc_channel_t m_adc_channel;
    float m_min_temp;
    float m_max_temp;
    adc_atten_t m_attenuation;
    
    adc_oneshot_unit_handle_t m_adc_handle;
    adc_cali_handle_t m_cali_handle;
    
    int m_last_raw_value;
    bool m_initialized;
    bool m_calibrated;
};

#endif // SETPOINT_CONTROLLER_H
