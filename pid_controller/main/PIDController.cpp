#include "PIDController.h"
#include <algorithm>

PIDController::PIDController(float kp, float ki, float kd, float setpoint)
    : m_kp(kp)
    , m_ki(ki)
    , m_kd(kd)
    , m_setpoint(setpoint)
    , m_p_term(0.0f)
    , m_i_term(0.0f)
    , m_d_term(0.0f)
    , m_error(0.0f)
    , m_last_error(0.0f)
    , m_output(0.0f)
    , m_feedforward(0.0f)
    , m_out_min(0.0f)
    , m_out_max(1.0f)
    , m_i_min(-0.5f)
    , m_i_max(0.5f)
    , m_filtered_input(0.0f)
    , m_last_filtered_input(0.0f)
    , m_filter_alpha(0.05f)  // 1/20 weight for new measurement
    , m_first_input(true)
    , m_output_alpha(0.3f)  // Output smoothing: 30% new, 70% old
    , m_last_output(0.0f)
{
}

void PIDController::setSetpoint(float setpoint) {
    m_setpoint = setpoint;
}

void PIDController::setTunings(float kp, float ki, float kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void PIDController::setOutputLimits(float min, float max) {
    m_out_min = min;
    m_out_max = max;
}

void PIDController::setIntegralLimits(float min, float max) {
    m_i_min = min;
    m_i_max = max;
}

void PIDController::setFeedforward(float ff) {
    m_feedforward = ff;
}

float PIDController::compute(float input, float dt) {
    // Update exponential moving average
    if (m_first_input) {
        m_filtered_input = input;
        m_last_filtered_input = input;
        m_first_input = false;
    } else {
        m_filtered_input = m_filter_alpha * input + (1.0f - m_filter_alpha) * m_filtered_input;
    }
    
    // Calculate error using filtered input
    m_error = m_setpoint - m_filtered_input;
    
    // Proportional term
    m_p_term = m_kp * m_error;
    
    // Integral term with anti-windup
    m_i_term += m_ki * m_error * dt;
    m_i_term = std::max(m_i_min, std::min(m_i_max, m_i_term));
    
    // Derivative term on measurement (prevents derivative kick on setpoint changes)
    m_d_term = -m_kd * (m_filtered_input - m_last_filtered_input) / dt;
    
    // Calculate raw output with feed-forward
    float raw_output = m_feedforward + m_p_term + m_i_term + m_d_term;
    
    // Apply output smoothing
    m_output = m_output_alpha * raw_output + (1.0f - m_output_alpha) * m_last_output;
    
    // Clamp output
    m_output = std::max(m_out_min, std::min(m_out_max, m_output));
    
    // Save states for next iteration
    m_last_filtered_input = m_filtered_input;
    m_last_error = m_error;
    m_last_output = m_output;
    m_output = std::max(m_out_min, std::min(m_out_max, m_output));
    
    // Save error for next iteration
    m_last_error = m_error;
    
    return m_output;
}

void PIDController::reset() {
    m_i_term = 0.0f;
    m_last_error = 0.0f;
    m_error = 0.0f;
    m_output = 0.0f;
    m_filtered_input = 0.0f;
    m_last_filtered_input = 0.0f;
    m_first_input = true;
    m_last_output = 0.0f;
}

void PIDController::setFilterWeight(float weight) {
    m_filter_alpha = std::max(0.0f, std::min(1.0f, weight));
}

void PIDController::setOutputSmoothing(float alpha) {
    m_output_alpha = std::max(0.0f, std::min(1.0f, alpha));
}
