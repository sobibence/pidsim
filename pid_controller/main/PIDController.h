#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float setpoint = 0.0);
    
    void setSetpoint(float setpoint);
    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void setIntegralLimits(float min, float max);
    void setFeedforward(float ff);
    
    float compute(float input, float dt);
    void reset();
    
    // Input filtering
    void setFilterWeight(float weight);
    float getFilteredInput() const { return m_filtered_input; }
    
    // Output smoothing
    void setOutputSmoothing(float alpha);
    
    // Getters for monitoring
    float getSetpoint() const { return m_setpoint; }
    float getError() const { return m_error; }
    float getP() const { return m_p_term; }
    float getI() const { return m_i_term; }
    float getD() const { return m_d_term; }
    float getOutput() const { return m_output; }
    
private:
    // PID gains
    float m_kp;
    float m_ki;
    float m_kd;
    
    // Setpoint
    float m_setpoint;
    
    // PID terms
    float m_p_term;
    float m_i_term;
    float m_d_term;
    
    // State
    float m_error;
    float m_last_error;
    float m_output;
    
    // Feed-forward
    float m_feedforward;
    
    // Limits
    float m_out_min;
    float m_out_max;
    float m_i_min;
    float m_i_max;
    
    // Input filtering (exponential moving average)
    float m_filtered_input;
    float m_last_filtered_input;  // For derivative on measurement
    float m_filter_alpha;  // Weight for new measurement (0.05 = 1/20)
    bool m_first_input;
    
    // Output smoothing
    float m_output_alpha;  // Weight for output smoothing (0 = no smoothing, 1 = no smoothing)
    float m_last_output;   // Previous smoothed output
};

#endif // PID_CONTROLLER_H
