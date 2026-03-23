#include "motor.h"

// ─────────────────────────────────────────────────────────────
//  Constructor — configure SimpleFOC objects with pin map
// ─────────────────────────────────────────────────────────────
MotorManager::MotorManager()
    : _motor_x(MOTOR_POLE_PAIRS),
      _motor_y(MOTOR_POLE_PAIRS),
      _motor_z(MOTOR_POLE_PAIRS),
      _driver_x(MOTOR1_PWM_PIN, MOTOR1_DIR_PIN, MOTOR1_EN_PIN),
      _driver_y(MOTOR2_PWM_PIN, MOTOR2_DIR_PIN, MOTOR2_EN_PIN),
      _driver_z(MOTOR3_PWM_PIN, MOTOR3_DIR_PIN, MOTOR3_EN_PIN),
      _enabled(false)
{
    for (uint8_t i = 0; i < 3; i++) {
        _states[i] = {0.0f, 0.0f, false, false};
    }
}

// ─────────────────────────────────────────────────────────────
//  begin() — initialize all 3 motors via SimpleFOC
// ─────────────────────────────────────────────────────────────
bool MotorManager::begin() {
    DEBUG_SERIAL.println("[MOTOR] Initializing 3-axis reaction wheels...");

    bool ok = true;
    ok &= _init_motor(_motor_x, _driver_x, AXIS_X, "X (roll)");
    ok &= _init_motor(_motor_y, _driver_y, AXIS_Y, "Y (pitch)");
    ok &= _init_motor(_motor_z, _driver_z, AXIS_Z, "Z (yaw)");

    if (ok) {
        DEBUG_SERIAL.println("[MOTOR] All 3 motors initialized");
        DEBUG_SERIAL.print("[MOTOR] Voltage limit: ");
        DEBUG_SERIAL.print(MOTOR_VOLTAGE_LIMIT);
        DEBUG_SERIAL.println("V per motor");
    } else {
        DEBUG_SERIAL.println("[MOTOR] WARNING: One or more motors failed init");
    }

    return ok;
}

// ─────────────────────────────────────────────────────────────
//  _init_motor() — configure and init a single motor
// ─────────────────────────────────────────────────────────────
bool MotorManager::_init_motor(BLDCMotor& motor,
                                BLDCDriver3PWM& driver,
                                uint8_t axis,
                                const char* label) {
    // Driver config
    driver.voltage_power_supply = MOTOR_VOLTAGE_LIMIT;
    driver.voltage_limit        = MOTOR_VOLTAGE_LIMIT;
    driver.pwm_frequency        = PWM_FREQUENCY_HZ;

    if (!driver.init()) {
        DEBUG_SERIAL.print("[MOTOR] Driver init failed: "); 
        DEBUG_SERIAL.println(label);
        return false;
    }

    // Motor config — open loop voltage torque mode
    // No encoder needed for reaction wheels in this mode
    motor.linkDriver(&driver);
    motor.voltage_limit    = MOTOR_VOLTAGE_LIMIT;
    motor.velocity_limit   = 200.0f;    // rad/s — mechanical safety limit
    motor.controller       = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;
    motor.foc_modulation   = FOCModulationType::SpaceVectorPWM;

    // Disable serial monitoring inside SimpleFOC (we handle our own)
    motor.useMonitoring(DEBUG_SERIAL);

    motor.init();

    // Open loop — no position sensor for reaction wheels
    motor.initFOC();

    _states[axis].initialized = true;
    _states[axis].enabled     = false;

    DEBUG_SERIAL.print("[MOTOR] Ready: "); 
    DEBUG_SERIAL.println(label);
    return true;
}

// ─────────────────────────────────────────────────────────────
//  enable() / disable() / emergency_stop()
// ─────────────────────────────────────────────────────────────
void MotorManager::enable() {
    _motor_x.enable();
    _motor_y.enable();
    _motor_z.enable();
    for (uint8_t i = 0; i < 3; i++) _states[i].enabled = true;
    _enabled = true;
    DEBUG_SERIAL.println("[MOTOR] All motors ENABLED");
}

void MotorManager::disable() {
    _motor_x.disable();
    _motor_y.disable();
    _motor_z.disable();
    for (uint8_t i = 0; i < 3; i++) {
        _states[i].enabled = false;
        _states[i].voltage = 0.0f;
    }
    _enabled = false;
    DEBUG_SERIAL.println("[MOTOR] All motors DISABLED");
}

void MotorManager::emergency_stop() {
    // Zero voltage immediately — no graceful shutdown
    _motor_x.target = 0.0f;
    _motor_y.target = 0.0f;
    _motor_z.target = 0.0f;
    _motor_x.disable();
    _motor_y.disable();
    _motor_z.disable();
    _enabled = false;
}

// ─────────────────────────────────────────────────────────────
//  set_torques() — main call from control loop
//  tau values are PID outputs in N·m
// ─────────────────────────────────────────────────────────────
void MotorManager::set_torques(float tau_x, float tau_y, float tau_z) {
    if (!_enabled) return;
    _apply_voltage(_motor_x, _states[AXIS_X], tau_x);
    _apply_voltage(_motor_y, _states[AXIS_Y], tau_y);
    _apply_voltage(_motor_z, _states[AXIS_Z], tau_z);
}

void MotorManager::set_torque_x(float tau) {
    if (!_enabled) return;
    _apply_voltage(_motor_x, _states[AXIS_X], tau);
}

void MotorManager::set_torque_y(float tau) {
    if (!_enabled) return;
    _apply_voltage(_motor_y, _states[AXIS_Y], tau);
}

void MotorManager::set_torque_z(float tau) {
    if (!_enabled) return;
    _apply_voltage(_motor_z, _states[AXIS_Z], tau);
}

// ─────────────────────────────────────────────────────────────
//  update() — call every control loop tick
//  Runs SimpleFOC's internal FOC algorithm
// ─────────────────────────────────────────────────────────────
void MotorManager::update() {
    if (!_enabled) return;
    _motor_x.loopFOC();
    _motor_y.loopFOC();
    _motor_z.loopFOC();
    _motor_x.move();
    _motor_y.move();
    _motor_z.move();
}

// ─────────────────────────────────────────────────────────────
//  _apply_voltage() — convert torque demand to voltage command
// ─────────────────────────────────────────────────────────────
void MotorManager::_apply_voltage(BLDCMotor& motor,
                                   MotorState& state,
                                   float tau) {
    float v = _torque_to_voltage(tau);
    motor.target = v;
    state.torque_demand = tau;
    state.voltage       = v;
}

float MotorManager::_torque_to_voltage(float tau) {
    // Linear mapping — calibrate TORQUE_TO_VOLTAGE after first test
    float v = tau * TORQUE_TO_VOLTAGE;
    // Clamp to motor voltage limit
    if (v >  MOTOR_VOLTAGE_LIMIT) v =  MOTOR_VOLTAGE_LIMIT;
    if (v < -MOTOR_VOLTAGE_LIMIT) v = -MOTOR_VOLTAGE_LIMIT;
    // Apply deadband — below this voltage motors don't move
    if (fabsf(v) < MOTOR_DEADBAND * MOTOR_VOLTAGE_LIMIT) v = 0.0f;
    return v;
}

// ─────────────────────────────────────────────────────────────
//  print() — telemetry snapshot
// ─────────────────────────────────────────────────────────────
void MotorManager::print() const {
    const char* axes[] = {"X","Y","Z"};
    for (uint8_t i = 0; i < 3; i++) {
        DEBUG_SERIAL.print("[MOT-"); DEBUG_SERIAL.print(axes[i]);
        DEBUG_SERIAL.print("] tau="); DEBUG_SERIAL.print(_states[i].torque_demand, 3);
        DEBUG_SERIAL.print(" V=");   DEBUG_SERIAL.print(_states[i].voltage, 3);
        DEBUG_SERIAL.print(" en=");  DEBUG_SERIAL.println(_states[i].enabled ? "Y" : "N");
    }
}
