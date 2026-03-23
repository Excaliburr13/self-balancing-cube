#include <Arduino.h>
#include "board_config.h"

// ─────────────────────────────────────────────────────────────
//  Self-Balancing Cube — Main Firmware
//  Layer 1: HAL & board config verification
//  Subsequent layers will be added as includes here
// ─────────────────────────────────────────────────────────────

// ── Timing ────────────────────────────────────────────────────
volatile bool     loop_flag = false;   // set by timer ISR
uint32_t          last_loop_us = 0;
uint32_t          loop_count   = 0;

// ── Forward declarations ──────────────────────────────────────
void control_loop_isr();
void print_system_info();
void blink_status(uint8_t n);

// ═════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════
void setup() {
    // ── Serial debug ──────────────────────────────────────────
    DEBUG_SERIAL.begin(SERIAL_BAUD);
    delay(500);                        // Let USB CDC enumerate
    print_system_info();

    // ── Status LED ────────────────────────────────────────────
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // ── IMU CS pin — hold high (inactive) until driver init ───
    pinMode(IMU_CS_PIN, OUTPUT);
    digitalWrite(IMU_CS_PIN, HIGH);

    // ── Motor enable pins — disabled until control loop ready ─
    pinMode(MOTOR1_EN_PIN, OUTPUT);  digitalWrite(MOTOR1_EN_PIN, LOW);
    pinMode(MOTOR2_EN_PIN, OUTPUT);  digitalWrite(MOTOR2_EN_PIN, LOW);
    pinMode(MOTOR3_EN_PIN, OUTPUT);  digitalWrite(MOTOR3_EN_PIN, LOW);

    // ── 1kHz control loop timer ───────────────────────────────
    // Uses HardwareTimer on STM32 — fires every 1000µs
    HardwareTimer *loop_timer = new HardwareTimer(TIM2);
    loop_timer->setOverflow(CONTROL_LOOP_HZ, HERTZ_FORMAT);
    loop_timer->attachInterrupt(control_loop_isr);
    loop_timer->resume();

    DEBUG_SERIAL.println("[INIT] Control loop timer started at 1kHz");
    DEBUG_SERIAL.println("[INIT] Layer 1 complete — system ready");
    DEBUG_SERIAL.println("[INIT] Waiting for Layer 2 (IMU driver)...");

    blink_status(3);    // 3 blinks = Layer 1 OK
}

// ═════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═════════════════════════════════════════════════════════════
void loop() {
    if (!loop_flag) return;
    loop_flag = false;

    loop_count++;

    // ── Heartbeat every 1000 loops (1 second) ─────────────────
    if (loop_count % 1000 == 0) {
        uint32_t now = micros();
        uint32_t actual_dt = now - last_loop_us;
        last_loop_us = now;

        // Print timing health — should be ~1000000 µs per 1000 loops
        DEBUG_SERIAL.print("[TICK] Loop #");
        DEBUG_SERIAL.print(loop_count);
        DEBUG_SERIAL.print("  dt_avg=");
        DEBUG_SERIAL.print(actual_dt / 1000);
        DEBUG_SERIAL.println("µs/loop");

        // Toggle LED as visual heartbeat
        digitalWrite(LED_PIN, loop_count % 2000 == 0);
    }

    // ── Layer 2+ will insert code here ────────────────────────
    // imu.update();
    // float angle = imu.get_angle();
    // float torque = pid.compute(0.0f - angle);
    // motor.set_torque(torque);
}

// ═════════════════════════════════════════════════════════════
//  TIMER ISR — fires every 1ms
// ═════════════════════════════════════════════════════════════
void control_loop_isr() {
    loop_flag = true;
}

// ═════════════════════════════════════════════════════════════
//  HELPERS
// ═════════════════════════════════════════════════════════════
void print_system_info() {
    DEBUG_SERIAL.println("=========================================");
    DEBUG_SERIAL.println("  Self-Balancing Cube — Firmware v0.1");
    DEBUG_SERIAL.println("  Layer 1: HAL & Board Config");
    DEBUG_SERIAL.println("=========================================");
    DEBUG_SERIAL.print("  Clock : "); DEBUG_SERIAL.print(SYS_CLOCK_MHZ);
    DEBUG_SERIAL.println(" MHz");
    DEBUG_SERIAL.print("  Loop  : "); DEBUG_SERIAL.print(CONTROL_LOOP_HZ);
    DEBUG_SERIAL.println(" Hz");
    DEBUG_SERIAL.print("  Baud  : "); DEBUG_SERIAL.println(SERIAL_BAUD);
    DEBUG_SERIAL.println("=========================================");
}

void blink_status(uint8_t n) {
    for (uint8_t i = 0; i < n; i++) {
        digitalWrite(LED_PIN, HIGH); delay(150);
        digitalWrite(LED_PIN, LOW);  delay(150);
    }
}
