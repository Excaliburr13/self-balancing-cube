#include "debug.h"

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────
DebugManager::DebugManager()
    : _loop_start_us(0), _loop_time_us(0), _header_printed(false)
{}

// ─────────────────────────────────────────────────────────────
//  begin()
// ─────────────────────────────────────────────────────────────
void DebugManager::begin() {
    DEBUG_SERIAL.println("[DEBUG] Telemetry layer ready");
    DEBUG_SERIAL.print("[DEBUG] Output rate: ");
    DEBUG_SERIAL.print(TELEM_RATE_HZ);
    DEBUG_SERIAL.println(" Hz");
    DEBUG_SERIAL.print("[DEBUG] Format: ");
    DEBUG_SERIAL.println(CSV_HEADER);
}

// ─────────────────────────────────────────────────────────────
//  update() — call every control loop tick
//  Only prints at TELEM_RATE_HZ to avoid overwhelming serial
// ─────────────────────────────────────────────────────────────
void DebugManager::update(const TelemetryPacket& pkt, uint32_t loop_count) {
    if (loop_count % TELEM_INTERVAL != 0) return;
    if (!_header_printed) {
        print_header();
        _header_printed = true;
    }
    _print_csv(pkt);
}

// ─────────────────────────────────────────────────────────────
//  print_header() — CSV column names
// ─────────────────────────────────────────────────────────────
void DebugManager::print_header() {
    DEBUG_SERIAL.println(CSV_HEADER);
}

// ─────────────────────────────────────────────────────────────
//  _print_csv() — one line per telemetry tick
//  Python plotter splits on commas to extract each field
// ─────────────────────────────────────────────────────────────
void DebugManager::_print_csv(const TelemetryPacket& pkt) {
    DEBUG_SERIAL.print(pkt.timestamp_ms);   DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(pkt.roll_deg,  2);   DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(pkt.pitch_deg, 2);   DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(pkt.p_term,    3);   DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(pkt.i_term,    3);   DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(pkt.d_term,    3);   DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(pkt.torque_x,  3);   DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(pkt.torque_y,  3);   DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.println(pkt.armed ? "1" : "0");
}

// ─────────────────────────────────────────────────────────────
//  Logging helpers
// ─────────────────────────────────────────────────────────────
void DebugManager::log(const char* tag, const char* msg) {
    DEBUG_SERIAL.print("["); DEBUG_SERIAL.print(tag);
    DEBUG_SERIAL.print("] "); DEBUG_SERIAL.println(msg);
}

void DebugManager::log_float(const char* tag, const char* label,
                               float val, uint8_t decimals) {
    DEBUG_SERIAL.print("["); DEBUG_SERIAL.print(tag); DEBUG_SERIAL.print("] ");
    DEBUG_SERIAL.print(label); DEBUG_SERIAL.print("=");
    DEBUG_SERIAL.println(val, decimals);
}

void DebugManager::log_value(const char* tag, const char* label, int32_t val) {
    DEBUG_SERIAL.print("["); DEBUG_SERIAL.print(tag); DEBUG_SERIAL.print("] ");
    DEBUG_SERIAL.print(label); DEBUG_SERIAL.print("=");
    DEBUG_SERIAL.println(val);
}

// ─────────────────────────────────────────────────────────────
//  Loop timing
// ─────────────────────────────────────────────────────────────
void DebugManager::mark_loop_start() {
    _loop_start_us = micros();
}

void DebugManager::mark_loop_end() {
    _loop_time_us = micros() - _loop_start_us;
}
