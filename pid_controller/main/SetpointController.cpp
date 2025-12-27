#include "SetpointController.h"
#include "esp_log.h"
#include <algorithm>

static const char *TAG = "SetpointCtrl";

SetpointController::SetpointController(adc_channel_t adc_channel,
                                       float min_temp,
                                       float max_temp,
                                       adc_atten_t attenuation)
    : m_adc_channel(adc_channel)
    , m_min_temp(min_temp)
    , m_max_temp(max_temp)
    , m_attenuation(attenuation)
    , m_adc_handle(nullptr)
    , m_cali_handle(nullptr)
    , m_last_raw_value(0)
    , m_initialized(false)
    , m_calibrated(false)
{
}

SetpointController::~SetpointController() {
    if (m_cali_handle) {
        adc_cali_delete_scheme_curve_fitting(m_cali_handle);
    }
    if (m_adc_handle) {
        adc_oneshot_del_unit(m_adc_handle);
    }
}

bool SetpointController::init() {
    // Configure ADC
    adc_oneshot_unit_init_cfg_t init_config = {};
    init_config.unit_id = ADC_UNIT_1;  // Use ADC1 (ADC2 conflicts with WiFi)
    init_config.ulp_mode = ADC_ULP_MODE_DISABLE;
    
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &m_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Configure channel
    adc_oneshot_chan_cfg_t chan_config = {};
    chan_config.atten = m_attenuation;
    chan_config.bitwidth = ADC_BITWIDTH_12;
    
    ret = adc_oneshot_config_channel(m_adc_handle, m_adc_channel, &chan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Try to initialize calibration
    adc_cali_curve_fitting_config_t cali_config = {};
    cali_config.unit_id = ADC_UNIT_1;
    cali_config.atten = m_attenuation;
    cali_config.bitwidth = ADC_BITWIDTH_12;
    
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &m_cali_handle);
    if (ret == ESP_OK) {
        m_calibrated = true;
        ESP_LOGI(TAG, "ADC calibration enabled");
    } else {
        ESP_LOGW(TAG, "ADC calibration not available, using raw values");
        m_calibrated = false;
    }
    
    m_initialized = true;
    ESP_LOGI(TAG, "Setpoint controller initialized (ADC channel %d, range: %.1f-%.1f°C)", 
             m_adc_channel, m_min_temp, m_max_temp);
    
    return true;
}

float SetpointController::readSetpoint() {
    if (!m_initialized) {
        ESP_LOGE(TAG, "Controller not initialized");
        return m_min_temp;
    }
    
    // Read ADC
    int adc_raw;
    esp_err_t ret = adc_oneshot_read(m_adc_handle, m_adc_channel, &adc_raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC: %s", esp_err_to_name(ret));
        return m_min_temp;
    }
    
    m_last_raw_value = adc_raw;
    
    // Direct voltage-to-temperature mapping:
    // 0V (0mV) = 40°C (minimum temperature)
    // 3.3V (3300mV) = 100°C (maximum temperature)
    // As voltage increases, temperature increases
    
    constexpr float V_MIN_MV = 0.0f;      // Voltage at min temp (40°C)
    constexpr float V_MAX_MV = 3300.0f;   // Voltage at max temp (100°C)
    
    float normalized;
    int voltage_mv = 0;
    
    if (m_calibrated) {
        ret = adc_cali_raw_to_voltage(m_cali_handle, adc_raw, &voltage_mv);
        if (ret == ESP_OK) {
            // Direct mapping: higher voltage = higher temperature
            normalized = (voltage_mv - V_MIN_MV) / (V_MAX_MV - V_MIN_MV);
            ESP_LOGI(TAG, "ADC: raw=%d, voltage=%dmV, normalized=%.3f", adc_raw, voltage_mv, normalized);
        } else {
            // Fallback to raw value mapping (direct)
            normalized = adc_raw / 4095.0f;
            ESP_LOGW(TAG, "ADC: raw=%d, voltage conversion failed, normalized=%.3f", adc_raw, normalized);
        }
    } else {
        // Raw value mapping (direct)
        normalized = adc_raw / 4095.0f;
        ESP_LOGI(TAG, "ADC: raw=%d (uncalibrated), normalized=%.3f", adc_raw, normalized);
    }
    
    // Clamp to 0-1 range
    normalized = std::max(0.0f, std::min(1.0f, normalized));
    
    // Map to temperature range
    float setpoint = m_min_temp + normalized * (m_max_temp - m_min_temp);
    
    return setpoint;
}
