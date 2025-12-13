#include "HeaterController.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>
#include <algorithm>

static const char *TAG = "HeaterController";

HeaterController::HeaterController(const DualHeaterConfig& config)
    : m_config(config)
    , m_zero_cross(nullptr)
    , m_target_power(0.0f)
    , m_dimmer_power(0.0f)
    , m_relay_on(false)
    , m_dimmer_task(nullptr)
    , m_running(false)
    , m_initialized(false)
{
}

HeaterController::~HeaterController() {
    if (m_running) {
        m_running = false;
        if (m_dimmer_task) {
            vTaskDelete(m_dimmer_task);
        }
    }
    if (m_zero_cross) {
        delete m_zero_cross;
    }
}

bool HeaterController::init() {
    // Initialize zero-cross detector
    m_zero_cross = new ZeroCrossDetector(m_config.zero_cross_pin, m_config.ac_frequency_hz);
    if (!m_zero_cross->init()) {
        ESP_LOGE(TAG, "Failed to initialize zero-cross detector");
        delete m_zero_cross;
        m_zero_cross = nullptr;
        return false;
    }
    
    // Configure TRIAC control pin
    gpio_config_t triac_conf = {};
    triac_conf.pin_bit_mask = (1ULL << m_config.dimmer_pin);
    triac_conf.mode = GPIO_MODE_OUTPUT;
    triac_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    triac_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    triac_conf.intr_type = GPIO_INTR_DISABLE;
    
    esp_err_t ret = gpio_config(&triac_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure TRIAC pin: %s", esp_err_to_name(ret));
        return false;
    }
    gpio_set_level(m_config.dimmer_pin, 0);
    
    // Configure relay pin if dual elements enabled
    if (m_config.enable_dual_elements) {
        gpio_config_t relay_conf = {};
        relay_conf.pin_bit_mask = (1ULL << m_config.relay_pin);
        relay_conf.mode = GPIO_MODE_OUTPUT;
        relay_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        relay_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        relay_conf.intr_type = GPIO_INTR_DISABLE;
        
        ret = gpio_config(&relay_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure relay pin: %s", esp_err_to_name(ret));
            return false;
        }
        gpio_set_level(m_config.relay_pin, 0);
    }
    
    // Create dimmer control task
    m_running = true;
    BaseType_t task_ret = xTaskCreate(
        dimmerTaskWrapper,
        "dimmer_ctrl",
        4096,
        this,
        configMAX_PRIORITIES - 1,  // High priority for timing accuracy
        &m_dimmer_task
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create dimmer control task");
        m_running = false;
        return false;
    }
    
    m_initialized = true;
    ESP_LOGI(TAG, "Heater controller initialized:");
    ESP_LOGI(TAG, "  Dual elements: %s", m_config.enable_dual_elements ? "YES" : "NO");
    ESP_LOGI(TAG, "  Element power: %.0fW", m_config.element_power_watts);
    ESP_LOGI(TAG, "  Max power: %.0fW", getMaxPower());
    ESP_LOGI(TAG, "  AC frequency: %luHz", m_config.ac_frequency_hz);
    
    return true;
}

void HeaterController::setPower(float percentage) {
    m_target_power = std::max(0.0f, std::min(1.0f, percentage));
    
    if (m_config.enable_dual_elements) {
        // Dual element logic:
        // 0-50%: Relay OFF, Dimmer 0-100%
        // 50-100%: Relay ON, Dimmer 0-100%
        if (m_target_power <= 0.5f) {
            setRelayState(false);
            setDimmerPower(m_target_power * 2.0f);  // Map 0-0.5 to 0-1
        } else {
            setRelayState(true);
            setDimmerPower((m_target_power - 0.5f) * 2.0f);  // Map 0.5-1 to 0-1
        }
    } else {
        // Single element: just control dimmer
        setRelayState(false);
        setDimmerPower(m_target_power);
    }
}

float HeaterController::getPowerWatts() const {
    if (m_config.enable_dual_elements) {
        float dimmer_watts = m_dimmer_power * m_config.element_power_watts;
        float relay_watts = m_relay_on ? m_config.element_power_watts : 0.0f;
        return dimmer_watts + relay_watts;
    } else {
        return m_dimmer_power * m_config.element_power_watts;
    }
}

float HeaterController::getMaxPower() const {
    if (m_config.enable_dual_elements) {
        return 2.0f * m_config.element_power_watts;
    } else {
        return m_config.element_power_watts;
    }
}

void HeaterController::setDimmerPower(float percentage) {
    m_dimmer_power = std::max(0.0f, std::min(1.0f, percentage));
}

void HeaterController::setRelayState(bool on) {
    if (m_relay_on != on && m_config.enable_dual_elements) {
        m_relay_on = on;
        gpio_set_level(m_config.relay_pin, on ? 1 : 0);
    }
}

void HeaterController::dimmerTaskWrapper(void* arg) {
    HeaterController* controller = (HeaterController*)arg;
    controller->dimmerControlTask();
}

void HeaterController::dimmerControlTask() {
    ESP_LOGI(TAG, "Dimmer control task started");
    
    while (m_running) {
        // Wait for zero-cross event
        if (!m_zero_cross->waitForZeroCross(100)) {
            continue;  // Timeout, try again
        }
        
        if (m_dimmer_power < 0.01f) {
            // Power too low, don't fire TRIAC
            continue;
        }
        
        if (m_dimmer_power > 0.99f) {
            // Full power, fire immediately
            gpio_set_level(m_config.dimmer_pin, 1);
            esp_rom_delay_us(m_config.triac_pulse_us);
            gpio_set_level(m_config.dimmer_pin, 0);
            continue;
        }
        
        // Calculate phase angle for desired power
        float phase_angle = powerToPhaseAngle(m_dimmer_power);
        
        // Convert to delay from zero-cross
        uint32_t half_period = m_zero_cross->getHalfPeriod();
        uint32_t delay_us = (uint32_t)(phase_angle / M_PI * half_period);
        
        // Wait for the calculated delay
        esp_rom_delay_us(delay_us);
        
        // Fire TRIAC
        gpio_set_level(m_config.dimmer_pin, 1);
        esp_rom_delay_us(m_config.triac_pulse_us);
        gpio_set_level(m_config.dimmer_pin, 0);
    }
    
    ESP_LOGI(TAG, "Dimmer control task stopped");
    vTaskDelete(NULL);
}

float HeaterController::phaseAngleToPower(float alpha) const {
    // RMS power formula for phase-controlled AC:
    // P/P_max = (1 - α/π + sin(2α)/(2π))
    // where α is the firing angle (0 to π)
    if (alpha <= 0.0f) return 1.0f;
    if (alpha >= M_PI) return 0.0f;
    
    return 1.0f - (alpha / M_PI) + (std::sin(2.0f * alpha) / (2.0f * M_PI));
}

float HeaterController::powerToPhaseAngle(float power) const {
    // Inverse of phaseAngleToPower
    // Use binary search to find phase angle for desired power
    if (power >= 1.0f) return 0.0f;
    if (power <= 0.0f) return M_PI;
    
    float alpha_min = 0.0f;
    float alpha_max = M_PI;
    float alpha_mid;
    
    // Binary search with tolerance
    for (int i = 0; i < 20; i++) {
        alpha_mid = (alpha_min + alpha_max) / 2.0f;
        float power_mid = phaseAngleToPower(alpha_mid);
        
        if (std::abs(power_mid - power) < 0.001f) {
            break;
        }
        
        if (power_mid > power) {
            alpha_min = alpha_mid;
        } else {
            alpha_max = alpha_mid;
        }
    }
    
    return alpha_mid;
}
