#include "ZeroCrossDetector.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "ZeroCross";

ZeroCrossDetector::ZeroCrossDetector(gpio_num_t zc_pin, uint32_t ac_frequency_hz)
    : m_zc_pin(zc_pin)
    , m_ac_frequency(ac_frequency_hz)
    , m_half_period_us(1000000 / (2 * ac_frequency_hz))
    , m_zc_semaphore(nullptr)
    , m_last_zc_time(0)
    , m_initialized(false)
{
}

ZeroCrossDetector::~ZeroCrossDetector() {
    if (m_initialized) {
        gpio_isr_handler_remove(m_zc_pin);
        if (m_zc_semaphore) {
            vSemaphoreDelete(m_zc_semaphore);
        }
    }
}

bool ZeroCrossDetector::init() {
    // Create semaphore for zero-cross events
    m_zc_semaphore = xSemaphoreCreateBinary();
    if (!m_zc_semaphore) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return false;
    }
    
    // Configure zero-cross detection pin
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << m_zc_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_POSEDGE;  // Rising edge on zero-cross
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Install ISR service
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Add ISR handler
    ret = gpio_isr_handler_add(m_zc_pin, zeroCrossISR, (void*)this);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler: %s", esp_err_to_name(ret));
        return false;
    }
    
    m_initialized = true;
    ESP_LOGI(TAG, "Zero-cross detector initialized on GPIO %d (%dHz, half-period=%luus)", 
             m_zc_pin, m_ac_frequency, m_half_period_us);
    return true;
}

bool ZeroCrossDetector::waitForZeroCross(uint32_t timeout_ms) {
    if (!m_initialized) {
        return false;
    }
    
    return xSemaphoreTake(m_zc_semaphore, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

uint32_t ZeroCrossDetector::getTimeSinceZeroCross() const {
    uint64_t now = esp_timer_get_time();
    return (uint32_t)(now - m_last_zc_time);
}

void IRAM_ATTR ZeroCrossDetector::zeroCrossISR(void* arg) {
    ZeroCrossDetector* detector = (ZeroCrossDetector*)arg;
    detector->m_last_zc_time = esp_timer_get_time();
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(detector->m_zc_semaphore, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}
