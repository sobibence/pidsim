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
    // Calculate error
    m_error = m_setpoint - input;
    
    // Proportional term
    m_p_term = m_kp * m_error;
    
    // Integral term with anti-windup
    m_i_term += m_ki * m_error * dt;
    m_i_term = std::max(m_i_min, std::min(m_i_max, m_i_term));
    
    // Derivative term
    m_d_term = m_kd * (m_error - m_last_error) / dt;
    
    // Calculate output with feed-forward
    m_output = m_feedforward + m_p_term + m_i_term + m_d_term;
    
    // Clamp output
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
}
