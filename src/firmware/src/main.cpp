#include <Arduino.h>
#include "board_config.h"
#include "imu.h"
#include "pid.h"
#include "motor.h"
#include "debug.h"

// ─────────────────────────────────────────────────────────────
//  Self-Balancing Cube — Main Firmware v1.0
//  Layer 1: HAL & board config
//  Layer 2: IMU driver & sensor fusion
//  Layer 3: PID control loop
//  Layer 4: Motor output (SimpleFOC)
//  Layer 5: UART debug & telemetry
// ─────────────────────────────────────────────────────────────

// ── Objects ───────────────────────────────────────────────────
IMUDriver    imu;
CubePID      pid_roll;
CubePID      pid_pitch;
MotorManager motors;
DebugManager dbg;

// ── Timing ────────────────────────────────────────────────────
volatile bool loop_flag  = false;
uint32_t      loop_count = 0;

// ── Serial command buffer ─────────────────────────────────────
String serial_buf = "";

// ── System state ──────────────────────────────────────────────
bool system_armed = false;

// ── Forward declarations ──────────────────────────────────────
void control_loop_isr();
void print_system_info();
void blink_status(uint8_t n);
void parse_serial_command(String cmd);

// ═════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════
void setup() {
    DEBUG_SERIAL.begin(SERIAL_BAUD);
    delay(500);
    print_system_info();

    // ── GPIO ──────────────────────────────────────────────────
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    pinMode(IMU_CS_PIN, OUTPUT);
    digitalWrite(IMU_CS_PIN, HIGH);

    // ── Layer 2: IMU ──────────────────────────────────────────
    dbg.log("INIT", "Starting IMU...");
    if (!imu.begin()) {
        dbg.log("ERROR", "IMU init failed — check SPI wiring");
        while (1) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
    }
    imu.calibrate(500);

    // ── Layer 3: PID ──────────────────────────────────────────
    pid_roll.begin (18.0f, 0.8f, 1.4f, 0.001f, -5.0f, 5.0f, 50.0f);
    pid_pitch.begin(18.0f, 0.8f, 1.4f, 0.001f, -5.0f, 5.0f, 50.0f);

    // ── Layer 4: Motors ───────────────────────────────────────
    dbg.log("INIT", "Starting motors...");
    if (!motors.begin()) {
        dbg.log("WARN", "Motor init issues — check wiring");
    }

    // ── Layer 5: Debug ────────────────────────────────────────
    dbg.begin();

    // ── 1kHz timer ────────────────────────────────────────────
    HardwareTimer *loop_timer = new HardwareTimer(TIM2);
    loop_timer->setOverflow(CONTROL_LOOP_HZ, HERTZ_FORMAT);
    loop_timer->attachInterrupt(control_loop_isr);
    loop_timer->resume();

    dbg.log("INIT", "All layers ready — firmware v1.0");
    DEBUG_SERIAL.println("─────────────────────────────────────────────");
    DEBUG_SERIAL.println("Commands: ARM | DISARM | KP/KI/KD <val>");
    DEBUG_SERIAL.println("          GAINS | RESET | STATUS");
    DEBUG_SERIAL.println("─────────────────────────────────────────────");
    DEBUG_SERIAL.println(">>> Type ARM to start balancing <<<");

    blink_status(5);    // 5 blinks = all layers ready
}

// ═════════════════════════════════════════════════════════════
//  MAIN LOOP — 1kHz
// ═════════════════════════════════════════════════════════════
void loop() {
    // ── Serial handler ────────────────────────────────────────
    while (DEBUG_SERIAL.available()) {
        char c = DEBUG_SERIAL.read();
        if (c == '\n' || c == '\r') {
            if (serial_buf.length() > 0) {
                parse_serial_command(serial_buf);
                serial_buf = "";
            }
        } else {
            serial_buf += c;
        }
    }

    if (!loop_flag) return;
    loop_flag = false;
    loop_count++;

    // ── Loop timing start ─────────────────────────────────────
    dbg.mark_loop_start();

    // ── Layer 2: IMU update ───────────────────────────────────
    imu.update(0.001f);
    float roll  = imu.getRoll();
    float pitch = imu.getPitch();

    // ── Safety cutoff ─────────────────────────────────────────
    if (fabsf(roll) > MAX_TILT_RAD || fabsf(pitch) > MAX_TILT_RAD) {
        if (system_armed) {
            system_armed = false;
            motors.emergency_stop();
            pid_roll.reset();
            pid_pitch.reset();
            dbg.log("SAFETY", "Tilt limit exceeded — DISARMED");
        }
        return;
    }

    // ── Layer 3: PID ──────────────────────────────────────────
    float tau_x = pid_roll.compute (0.0f, roll);
    float tau_y = pid_pitch.compute(0.0f, pitch);

    // ── Layer 4: Motor output ─────────────────────────────────
    if (system_armed) {
        motors.set_torques(tau_x, tau_y, 0.0f);
        motors.update();
    }

    // ── Loop timing end ───────────────────────────────────────
    dbg.mark_loop_end();

    // ── Layer 5: Telemetry ────────────────────────────────────
    TelemetryPacket pkt = {
        .timestamp_ms = millis(),
        .roll_deg     = roll  * 57.296f,
        .pitch_deg    = pitch * 57.296f,
        .p_term       = pid_roll.getP(),
        .i_term       = pid_roll.getI(),
        .d_term       = pid_roll.getD(),
        .torque_x     = tau_x,
        .torque_y     = tau_y,
        .armed        = system_armed
    };
    dbg.update(pkt, loop_count);

    // ── Heartbeat LED ─────────────────────────────────────────
    if (loop_count % 1000 == 0) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
}

// ═════════════════════════════════════════════════════════════
//  SERIAL COMMAND PARSER
// ═════════════════════════════════════════════════════════════
void parse_serial_command(String cmd) {
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "ARM") {
        system_armed = true;
        motors.enable();
        pid_roll.reset();
        pid_pitch.reset();
        dbg.log("CMD", "ARMED — balancing active");

    } else if (cmd == "DISARM") {
        system_armed = false;
        motors.disable();
        pid_roll.reset();
        pid_pitch.reset();
        dbg.log("CMD", "DISARMED");

    } else if (cmd.startsWith("KP ")) {
        float v = cmd.substring(3).toFloat();
        pid_roll.set_kp(v); pid_pitch.set_kp(v);
        dbg.log_float("CMD", "Kp", v);

    } else if (cmd.startsWith("KI ")) {
        float v = cmd.substring(3).toFloat();
        pid_roll.set_ki(v); pid_pitch.set_ki(v);
        dbg.log_float("CMD", "Ki", v);

    } else if (cmd.startsWith("KD ")) {
        float v = cmd.substring(3).toFloat();
        pid_roll.set_kd(v); pid_pitch.set_kd(v);
        dbg.log_float("CMD", "Kd", v);

    } else if (cmd == "GAINS") {
        PIDGains g = pid_roll.getGains();
        DEBUG_SERIAL.print("[GAINS] Kp="); DEBUG_SERIAL.print(g.kp);
        DEBUG_SERIAL.print(" Ki=");        DEBUG_SERIAL.print(g.ki);
        DEBUG_SERIAL.print(" Kd=");        DEBUG_SERIAL.println(g.kd);

    } else if (cmd == "RESET") {
        pid_roll.reset(); pid_pitch.reset();
        dbg.log("CMD", "PID state reset");

    } else if (cmd == "STATUS") {
        DEBUG_SERIAL.print("[STATUS] Armed=");
        DEBUG_SERIAL.print(system_armed ? "YES" : "NO");
        DEBUG_SERIAL.print("  Loop=");
        DEBUG_SERIAL.print(dbg.get_loop_time_us());
        DEBUG_SERIAL.println("us");
        imu.printAngles();
        pid_roll.print();
        motors.print();

    } else {
        dbg.log("CMD", "Unknown command");
    }
}

// ═════════════════════════════════════════════════════════════
//  ISR & HELPERS
// ═════════════════════════════════════════════════════════════
void control_loop_isr() { loop_flag = true; }

void print_system_info() {
    DEBUG_SERIAL.println("=========================================");
    DEBUG_SERIAL.println("  Self-Balancing Cube — Firmware v1.0");
    DEBUG_SERIAL.println("  L1:HAL L2:IMU L3:PID L4:MOTOR L5:DBG");
    DEBUG_SERIAL.println("=========================================");
}

void blink_status(uint8_t n) {
    for (uint8_t i = 0; i < n; i++) {
        digitalWrite(LED_PIN, HIGH); delay(150);
        digitalWrite(LED_PIN, LOW);  delay(150);
    }
}
