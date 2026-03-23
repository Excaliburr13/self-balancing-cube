#include "pid.h"
#include "board_config.h"

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────
CubePID::CubePID()
    : _dt(0.001f), _out_min(-5.0f), _out_max(5.0f),
      _error(0), _prev_error(0), _integral(0), _prev_derivative(0),
      _p_term(0), _i_term(0), _d_term(0), _output(0),
      _deriv_alpha(0.239f), _integral_limit(10.0f)
{
    _gains = {0, 0, 0};
}

// ─────────────────────────────────────────────────────────────
//  begin() — configure the controller
// ─────────────────────────────────────────────────────────────
void CubePID::begin(float kp, float ki, float kd,
                           float dt, float out_min, float out_max,
                           float deriv_cutoff_hz) {
    _gains    = {kp, ki, kd};
    _dt       = dt;
    _out_min  = out_min;
    _out_max  = out_max;

    // Integral anti-windup limit: output range / ki (prevents runaway)
    _integral_limit = (ki > 0.0f)
                    ? (out_max - out_min) / (2.0f * ki)
                    : 1e6f;

    _compute_alpha(deriv_cutoff_hz);
    reset();

    DEBUG_SERIAL.println("[PID] Controller initialized");
    DEBUG_SERIAL.print("  Kp="); DEBUG_SERIAL.print(kp);
    DEBUG_SERIAL.print("  Ki="); DEBUG_SERIAL.print(ki);
    DEBUG_SERIAL.print("  Kd="); DEBUG_SERIAL.println(kd);
    DEBUG_SERIAL.print("  Deriv filter: fc=");
    DEBUG_SERIAL.print(deriv_cutoff_hz);
    DEBUG_SERIAL.print("Hz  alpha=");
    DEBUG_SERIAL.println(_deriv_alpha, 4);
}

// ─────────────────────────────────────────────────────────────
//  reset() — clear state without changing gains
// ─────────────────────────────────────────────────────────────
void CubePID::reset() {
    _error           = 0.0f;
    _prev_error      = 0.0f;
    _integral        = 0.0f;
    _prev_derivative = 0.0f;
    _p_term          = 0.0f;
    _i_term          = 0.0f;
    _d_term          = 0.0f;
    _output          = 0.0f;
}

// ─────────────────────────────────────────────────────────────
//  compute() — call every 1ms from control loop
//
//  setpoint    = desired angle (0.0 rad = balanced)
//  measurement = current angle from IMU (rad)
//  returns     = torque demand (N·m), clamped to [out_min, out_max]
// ─────────────────────────────────────────────────────────────
float CubePID::compute(float setpoint, float measurement) {
    // ── Error ─────────────────────────────────────────────────
    _error = setpoint - measurement;

    // ── Proportional ──────────────────────────────────────────
    _p_term = _gains.kp * _error;

    // ── Integral with anti-windup ─────────────────────────────
    _integral += _error * _dt;
    _integral  = _clamp(_integral, -_integral_limit, _integral_limit);
    _i_term    = _gains.ki * _integral;

    // ── Derivative with low-pass filter ───────────────────────
    // Raw derivative: rate of change of error
    float raw_deriv = (_error - _prev_error) / _dt;

    // IIR low-pass: y[n] = alpha*x[n] + (1-alpha)*y[n-1]
    float filt_deriv = _deriv_alpha * raw_deriv
                     + (1.0f - _deriv_alpha) * _prev_derivative;

    _d_term          = _gains.kd * filt_deriv;
    _prev_derivative = filt_deriv;

    // ── Sum and clamp output ──────────────────────────────────
    _output = _clamp(_p_term + _i_term + _d_term, _out_min, _out_max);

    // ── Store for next iteration ──────────────────────────────
    _prev_error = _error;

    return _output;
}

// ─────────────────────────────────────────────────────────────
//  set_gains() — runtime update (called from serial parser)
//  Recomputes integral limit and resets integral to avoid bump
// ─────────────────────────────────────────────────────────────
void CubePID::set_gains(float kp, float ki, float kd) {
    _gains = {kp, ki, kd};

    // Recompute anti-windup limit with new ki
    _integral_limit = (ki > 0.0f)
                    ? (_out_max - _out_min) / (2.0f * ki)
                    : 1e6f;

    // Reset integral on gain change to prevent bump
    _integral = 0.0f;

    DEBUG_SERIAL.print("[PID] Gains updated: Kp=");
    DEBUG_SERIAL.print(kp); DEBUG_SERIAL.print(" Ki=");
    DEBUG_SERIAL.print(ki); DEBUG_SERIAL.print(" Kd=");
    DEBUG_SERIAL.println(kd);
}

// ─────────────────────────────────────────────────────────────
//  _compute_alpha() — derive IIR coefficient from cutoff freq
// ─────────────────────────────────────────────────────────────
void CubePID::_compute_alpha(float cutoff_hz) {
    float wc   = 2.0f * 3.14159265f * cutoff_hz * _dt;
    _deriv_alpha = wc / (wc + 1.0f);
}

// ─────────────────────────────────────────────────────────────
//  _clamp() — inline utility
// ─────────────────────────────────────────────────────────────
float CubePID::_clamp(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ─────────────────────────────────────────────────────────────
//  print() — telemetry snapshot
// ─────────────────────────────────────────────────────────────
void CubePID::print() const {
    DEBUG_SERIAL.print("[PID] err=");  DEBUG_SERIAL.print(_error,  4);
    DEBUG_SERIAL.print(" P=");         DEBUG_SERIAL.print(_p_term, 4);
    DEBUG_SERIAL.print(" I=");         DEBUG_SERIAL.print(_i_term, 4);
    DEBUG_SERIAL.print(" D=");         DEBUG_SERIAL.print(_d_term, 4);
    DEBUG_SERIAL.print(" out=");       DEBUG_SERIAL.println(_output, 4);
}
