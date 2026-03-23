#pragma once
#include <Arduino.h>
#include "board_config.h"

// ─────────────────────────────────────────────────────────────
//  Debug & Telemetry Layer — Self-Balancing Cube
//
//  Outputs structured CSV over UART at configurable rate
//  Format: timestamp,roll,pitch,P,I,D,torque_x,torque_y,armed
//
//  Python plotter reads this stream and plots live
// ─────────────────────────────────────────────────────────────

// Telemetry output rate
#define TELEM_RATE_HZ       20      // 20 Hz output (every 50 loops)
#define TELEM_INTERVAL      (CONTROL_LOOP_HZ / TELEM_RATE_HZ)

// CSV header — must match print order in debug.cpp
#define CSV_HEADER  "t_ms,roll_deg,pitch_deg,P,I,D,tau_x,tau_y,armed"

// ─────────────────────────────────────────────────────────────
//  Telemetry data packet
// ─────────────────────────────────────────────────────────────
struct TelemetryPacket {
    uint32_t timestamp_ms;
    float    roll_deg;
    float    pitch_deg;
    float    p_term;
    float    i_term;
    float    d_term;
    float    torque_x;
    float    torque_y;
    bool     armed;
};

// ─────────────────────────────────────────────────────────────
//  Debug Manager Class
// ─────────────────────────────────────────────────────────────
class DebugManager {
public:
    DebugManager();

    // Lifecycle
    void    begin();

    // Call every control loop tick with current system state
    void    update(const TelemetryPacket& pkt, uint32_t loop_count);

    // Print CSV header — call once after init
    void    print_header();

    // One-shot prints
    void    log(const char* tag, const char* msg);
    void    log_float(const char* tag, const char* label, float val, uint8_t decimals = 3);
    void    log_value(const char* tag, const char* label, int32_t val);

    // Loop timing diagnostics
    void    mark_loop_start();
    void    mark_loop_end();
    uint32_t get_loop_time_us() const { return _loop_time_us; }

private:
    void    _print_csv(const TelemetryPacket& pkt);

    uint32_t _loop_start_us;
    uint32_t _loop_time_us;
    bool     _header_printed;
};
