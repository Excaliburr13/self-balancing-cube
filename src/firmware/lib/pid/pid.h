#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
//  PID Controller — Self-Balancing Cube
//  Features:
//    - Discrete PID with proper dt integration
//    - Anti-windup on integral term
//    - Low-pass filter on derivative term
//    - Runtime gain tuning via serial commands
//    - Output clamping
// ─────────────────────────────────────────────────────────────

struct PIDGains {
    float kp;
    float ki;
    float kd;
};

class CubePID {
public:
    CubePID();

    // Lifecycle
    void    begin(float kp, float ki, float kd,
                  float dt, float out_min, float out_max,
                  float deriv_cutoff_hz = 50.0f);
    void    reset();

    // Core
    float   compute(float setpoint, float measurement);

    // Runtime gain tuning — call from serial parser
    void    set_gains(float kp, float ki, float kd);
    void    set_kp(float kp) { _gains.kp = kp; }
    void    set_ki(float ki) { _gains.ki = ki; }
    void    set_kd(float kd) { _gains.kd = kd; }

    // Getters
    PIDGains getGains()     const { return _gains; }
    float    getError()     const { return _error; }
    float    getP()         const { return _p_term; }
    float    getI()         const { return _i_term; }
    float    getD()         const { return _d_term; }
    float    getOutput()    const { return _output; }

    // Debug
    void     print() const;

private:
    // Gains
    PIDGains _gains;

    // Timing
    float   _dt;

    // Output limits
    float   _out_min;
    float   _out_max;

    // State
    float   _error;
    float   _prev_error;
    float   _integral;
    float   _prev_derivative;

    // Terms (for telemetry)
    float   _p_term;
    float   _i_term;
    float   _d_term;
    float   _output;

    // Derivative low-pass filter coefficient
    float   _deriv_alpha;

    // Anti-windup limit
    float   _integral_limit;

    // Helper
    float   _clamp(float v, float lo, float hi);
    void    _compute_alpha(float cutoff_hz);
};
