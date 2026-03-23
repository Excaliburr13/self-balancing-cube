#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include "board_config.h"

// ─────────────────────────────────────────────────────────────
//  Motor Output Layer — Self-Balancing Cube
//
//  3 brushless gimbal motors as reaction wheels:
//    Motor 1 — X axis (roll correction)
//    Motor 2 — Y axis (pitch correction)
//    Motor 3 — Z axis (yaw stabilization)
//
//  Uses SimpleFOC in torque/voltage mode
//  2204 gimbal motors — 12N14P pole configuration
// ─────────────────────────────────────────────────────────────

// Motor pole pairs — 2204 gimbal motor: 14 poles = 7 pairs
#define MOTOR_POLE_PAIRS    7

// Voltage limit per motor (V) — tune based on LiPo voltage
// 3S LiPo = 12.6V fully charged, use 10V limit for safety
#define MOTOR_VOLTAGE_LIMIT 10.0f

// Torque to voltage mapping
// Empirical: 1 N·m torque demand ≈ 3.0V at these motors
// Tune this after first hardware test
#define TORQUE_TO_VOLTAGE   3.0f

// ─────────────────────────────────────────────────────────────
//  Motor axis assignment
// ─────────────────────────────────────────────────────────────
enum MotorAxis {
    AXIS_X = 0,   // Roll  — primary balance axis (edge balance)
    AXIS_Y = 1,   // Pitch — secondary balance axis
    AXIS_Z = 2    // Yaw   — stabilization
};

// ─────────────────────────────────────────────────────────────
//  Motor state struct
// ─────────────────────────────────────────────────────────────
struct MotorState {
    float voltage;          // Current voltage command (V)
    float torque_demand;    // Torque requested by PID (N·m)
    bool  enabled;          // Motor enable state
    bool  initialized;      // SimpleFOC init success
};

// ─────────────────────────────────────────────────────────────
//  Motor Manager Class
// ─────────────────────────────────────────────────────────────
class MotorManager {
public:
    MotorManager();

    // Lifecycle
    bool    begin();                    // Init all 3 motors
    void    enable();                   // Enable all motors
    void    disable();                  // Disable all — SAFETY
    void    emergency_stop();           // Immediate zero torque

    // Torque commands (N·m) from PID output
    void    set_torque_x(float tau);    // Roll correction
    void    set_torque_y(float tau);    // Pitch correction
    void    set_torque_z(float tau);    // Yaw stabilization

    // Combined — call once per loop with PID outputs
    void    set_torques(float tau_x, float tau_y, float tau_z);

    // Loop update — must call every control loop tick
    void    update();

    // Getters
    MotorState getState(uint8_t axis) const { return _states[axis]; }
    bool       isEnabled()            const { return _enabled; }

    // Debug
    void    print() const;

private:
    // SimpleFOC motor objects
    BLDCMotor   _motor_x;
    BLDCMotor   _motor_y;
    BLDCMotor   _motor_z;

    // SimpleFOC 3-PWM drivers
    BLDCDriver3PWM _driver_x;
    BLDCDriver3PWM _driver_y;
    BLDCDriver3PWM _driver_z;

    // State
    MotorState  _states[3];
    bool        _enabled;

    // Internal
    float   _torque_to_voltage(float tau);
    void    _apply_voltage(BLDCMotor& motor, MotorState& state, float tau);
    bool    _init_motor(BLDCMotor& motor, BLDCDriver3PWM& driver,
                        uint8_t axis, const char* label);
};
