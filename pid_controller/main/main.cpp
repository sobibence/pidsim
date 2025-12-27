#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "PIDController.h"
#include "MAX6675Sensor.h"
#include "HeaterController.h"
#include "SetpointController.h"
#include "OLEDDisplay.h"

static const char *TAG = "PID_TEMP_CONTROL";

// ==================== CONFIGURATION ====================

// === Hardware Pin Configuration ===
constexpr gpio_num_t MAX6675_SCK      = GPIO_NUM_0;   // MAX6675 SPI Clock
constexpr gpio_num_t MAX6675_CS       = GPIO_NUM_13;   // MAX6675 Chip Select
constexpr gpio_num_t MAX6675_MISO     = GPIO_NUM_14;   // MAX6675 Data Out
constexpr gpio_num_t TRIAC_GATE_PIN   = GPIO_NUM_22;   // TRIAC gate control
constexpr gpio_num_t ZERO_CROSS_PIN   = GPIO_NUM_10;   // Zero-cross detector input
constexpr gpio_num_t RELAY_PIN        = GPIO_NUM_6;   // Relay control (for 2nd element)
constexpr adc_channel_t POT_ADC_CH    = ADC_CHANNEL_0; // ADC for potentiometer - GPIO_5

// === OLED Display Configuration ===
constexpr bool ENABLE_OLED_DISPLAY = true;     // Enable/disable OLED display
constexpr gpio_num_t OLED_SDA_PIN  = GPIO_NUM_12;  // I2C SDA for OLED
constexpr gpio_num_t OLED_SCL_PIN  = GPIO_NUM_11;  // I2C SCL for OLED
constexpr uint8_t OLED_I2C_ADDR    = 0x3C;     // I2C address (usually 0x3C or 0x3D)
constexpr uint8_t OLED_WIDTH       = 128;      // Screen width in pixels
constexpr uint8_t OLED_HEIGHT      = 64;       // Screen height in pixels (64 or 32)
constexpr uint32_t OLED_UPDATE_MS  = 500;      // Display update rate (milliseconds)

// === Temperature Sensor Calibration ===
// Adjust if your sensor reads incorrectly
// Formula: actual_temp = (sensor_reading * TEMP_SCALE) + TEMP_OFFSET
constexpr float TEMP_SENSOR_OFFSET = 0.0f;   // °C offset (e.g., +2.5 if reads 2.5°C low)
constexpr float TEMP_SENSOR_SCALE  = 1.0f;   // Scale factor (usually 1.0)

// === PID Parameters ===
// Tuned values from Python simulation
constexpr float PID_KP = 0.2f;       // Proportional gain
constexpr float PID_KI = 0.0019f;    // Integral gain
constexpr float PID_KD = 2.0f;       // Derivative gain

// === Setpoint Configuration ===
constexpr bool USE_POTENTIOMETER = true;      // true = pot controls setpoint, false = fixed
constexpr float FIXED_SETPOINT = 60.0f;        // Fixed setpoint when pot disabled (°C)
constexpr float POT_MIN_TEMP = 20.0f;          // Minimum temperature for pot (°C) - at 0V
constexpr float POT_MAX_TEMP = 110.0f;         // Maximum temperature for pot (°C) - at 1.7V

// === Dual Heating Element Configuration ===
constexpr bool ENABLE_DUAL_ELEMENTS = false;    // true = use dimmer + relay, false = dimmer only
constexpr float ELEMENT_POWER_WATTS = 1200.0f; // Power per heating element (W)
// Note: Total power = ELEMENT_POWER_WATTS * (DUAL_ELEMENTS ? 2 : 1)

// === AC Dimmer Configuration ===
constexpr uint32_t AC_FREQUENCY_HZ = 50;       // Mains frequency: 50Hz (EU) or 60Hz (US)
constexpr uint32_t TRIAC_PULSE_WIDTH_US = 10;  // TRIAC gate pulse width (microseconds)

// === Control Loop Timing ===
constexpr uint32_t CONTROL_LOOP_MS = 500;     // Update rate (milliseconds)
constexpr float CONTROL_LOOP_DT = CONTROL_LOOP_MS / 1000.0f;  // seconds

// === Feed-forward Compensation ===
// Static estimate of heat loss for better disturbance rejection
constexpr float FEEDFORWARD_POWER = 200.0f;    // Estimated heat loss (W)

// ==================== END CONFIGURATION ====================

// ==================== Main Application ====================

extern "C" void app_main() {
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "  PID Temperature Controller with AC Dimmer");
    ESP_LOGI(TAG, "==================================================");
    
    // ========== Initialize Temperature Sensor ==========
    MAX6675Sensor temp_sensor(MAX6675_SCK, MAX6675_CS, MAX6675_MISO);
    if (!temp_sensor.init()) {
        ESP_LOGE(TAG, "Failed to initialize temperature sensor!");
        return;
    }
    
    // Apply calibration
    temp_sensor.setCalibration(TEMP_SENSOR_OFFSET, TEMP_SENSOR_SCALE);
    
    // ========== Initialize Heater Controller ==========
    DualHeaterConfig heater_config = {};
    heater_config.enable_dual_elements = ENABLE_DUAL_ELEMENTS;
    heater_config.element_power_watts = ELEMENT_POWER_WATTS;
    heater_config.dimmer_pin = TRIAC_GATE_PIN;
    heater_config.relay_pin = RELAY_PIN;
    heater_config.zero_cross_pin = ZERO_CROSS_PIN;
    heater_config.ac_frequency_hz = AC_FREQUENCY_HZ;
    heater_config.triac_pulse_us = TRIAC_PULSE_WIDTH_US;
    
    HeaterController heater(heater_config);
    if (!heater.init()) {
        ESP_LOGE(TAG, "Failed to initialize heater controller!");
        return;
    }
    
    // ========== Initialize Setpoint Controller (Optional) ==========
    SetpointController* setpoint_ctrl = nullptr;
    if (USE_POTENTIOMETER) {
        setpoint_ctrl = new SetpointController(POT_ADC_CH, POT_MIN_TEMP, POT_MAX_TEMP);
        if (!setpoint_ctrl->init()) {
            ESP_LOGE(TAG, "Failed to initialize setpoint controller!");
            delete setpoint_ctrl;
            setpoint_ctrl = nullptr;
        }
    }
    
    // ========== Initialize OLED Display (Optional) ==========
    OLEDDisplay* display = nullptr;
    if (ENABLE_OLED_DISPLAY) {
        OLEDConfig display_config = {};
        display_config.sda_pin = OLED_SDA_PIN;
        display_config.scl_pin = OLED_SCL_PIN;
        display_config.i2c_address = OLED_I2C_ADDR;
        display_config.i2c_freq_hz = 400000;  // 400kHz
        display_config.screen_width = OLED_WIDTH;
        display_config.screen_height = OLED_HEIGHT;
        
        display = new OLEDDisplay(display_config);
        if (!display->init()) {
            ESP_LOGE(TAG, "Failed to initialize OLED display!");
            delete display;
            display = nullptr;
        } else {
            // Show splash screen
            display->clear();
            display->drawString(10, 10, "PID Controller");
            display->drawString(20, 25, "Starting...");
            display->display();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    // ========== Initialize PID Controller ==========
    float initial_setpoint = USE_POTENTIOMETER && setpoint_ctrl 
                            ? setpoint_ctrl->readSetpoint() 
                            : FIXED_SETPOINT;
    
    PIDController pid(PID_KP, PID_KI, PID_KD, initial_setpoint);
    pid.setOutputLimits(0.0f, 1.0f);
    pid.setIntegralLimits(-0.5f, 0.5f);  // Anti-windup
    
    // Calculate feed-forward normalized to max power
    float feedforward_norm = FEEDFORWARD_POWER / heater.getMaxPower();
    pid.setFeedforward(feedforward_norm);
    
    // ========== Print Configuration ==========
    ESP_LOGI(TAG, "Configuration:");
    ESP_LOGI(TAG, "  PID Gains: Kp=%.3f, Ki=%.4f, Kd=%.1f", PID_KP, PID_KI, PID_KD);
    ESP_LOGI(TAG, "  Setpoint: %s", USE_POTENTIOMETER ? "POTENTIOMETER" : "FIXED");
    if (!USE_POTENTIOMETER) {
        ESP_LOGI(TAG, "  Fixed Setpoint: %.1f°C", FIXED_SETPOINT);
    }
    ESP_LOGI(TAG, "  Dual Elements: %s", ENABLE_DUAL_ELEMENTS ? "YES" : "NO");
    ESP_LOGI(TAG, "  Max Power: %.0fW", heater.getMaxPower());
    ESP_LOGI(TAG, "  AC Frequency: %luHz", AC_FREQUENCY_HZ);
    ESP_LOGI(TAG, "  Feed-forward: %.0fW (%.1f%%)", FEEDFORWARD_POWER, feedforward_norm * 100.0f);
    ESP_LOGI(TAG, "  Temp Calibration: offset=%.2f°C, scale=%.4f", 
             TEMP_SENSOR_OFFSET, TEMP_SENSOR_SCALE);
    ESP_LOGI(TAG, "  OLED Display: %s", ENABLE_OLED_DISPLAY && display ? "ENABLED" : "DISABLED");
    ESP_LOGI(TAG, "==================================================\n");
    
    ESP_LOGI(TAG, "Starting control loop...\n");
    
    // ========== Control Loop ==========
    uint32_t iteration = 0;
    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t last_display_update = 0;
    
    while (true) {
        // Update setpoint from potentiometer if enabled
        if (USE_POTENTIOMETER && setpoint_ctrl) {
            float new_setpoint = setpoint_ctrl->readSetpoint();
            if (std::abs(new_setpoint - pid.getSetpoint()) > 0.5f) {
                pid.setSetpoint(new_setpoint);
                ESP_LOGI(TAG, "Setpoint changed to %.1f°C (ADC: %d)", 
                         new_setpoint, setpoint_ctrl->getRawADC());
            }
        }
        
        // Read current temperature
        float current_temp = temp_sensor.readCelsius();
        
        if (std::isnan(current_temp)) {
            ESP_LOGW(TAG, "Invalid temperature reading - sensor disconnected?");
            heater.setPower(0.0f);  // Safety: turn off heater
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_LOOP_MS));
            continue;
        }
        
        // Compute PID output
        float control_output = pid.compute(current_temp, CONTROL_LOOP_DT);
        
        // Apply to heater
        heater.setPower(control_output);
        
        // Update OLED display periodically
        if (ENABLE_OLED_DISPLAY && display) {
            uint32_t current_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (current_time_ms - last_display_update >= OLED_UPDATE_MS) {
                float dimmer_percent = heater.getDimmerPercentage();
                bool relay_state = heater.getRelayState();
                display->updateStatus(
                    pid.getSetpoint(),
                    current_temp,
                    control_output * 100.0f,
                    dimmer_percent,
                    relay_state
                );
                last_display_update = current_time_ms;
            }
        }
        
        // Detailed logging
        if (ENABLE_DUAL_ELEMENTS) {
            ESP_LOGI(TAG, "[%5lu] Temp: %5.2f°C | SP: %5.1f°C | Err: %+6.2f | "
                          "PID: P:%+.3f I:%+.3f D:%+.3f | Out: %5.1f%% | "
                          "Dimmer: %5.1f%% | Relay: %s | Power: %5.0fW",
                     iteration++,
                     current_temp,
                     pid.getSetpoint(),
                     pid.getError(),
                     pid.getP(), pid.getI(), pid.getD(),
                     control_output * 100.0f,
                     heater.getDimmerPercentage() * 100.0f,
                     heater.getRelayState() ? "ON " : "OFF",
                     heater.getPowerWatts());
        } else {
            ESP_LOGI(TAG, "[%5lu] Temp: %5.2f°C | SP: %5.1f°C | Err: %+6.2f | "
                          "PID: P:%+.3f I:%+.3f D:%+.3f | Out: %5.1f%% (%5.0fW)",
                     iteration++,
                     current_temp,
                     pid.getSetpoint(),
                     pid.getError(),
                     pid.getP(), pid.getI(), pid.getD(),
                     control_output * 100.0f,
                     heater.getPowerWatts());
        }
        
        // Wait for next control cycle
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_LOOP_MS));
    }
    
    // Cleanup (never reached in normal operation)
    if (setpoint_ctrl) {
        delete setpoint_ctrl;
    }
}