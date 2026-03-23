#include <Arduino.h>
#include "board_config.h"
#include "imu.h"
#include "pid.h"
#include "motor.h"

// ─────────────────────────────────────────────────────────────
//  Self-Balancing Cube — Main Firmware
//  Layer 1: HAL & board config
//  Layer 2: IMU driver & sensor fusion
//  Layer 3: PID control loop
//  Layer 4: Motor output (SimpleFOC)
// ─────────────────────────────────────────────────────────────

// ── Objects ───────────────────────────────────────────────────
IMUDriver     imu;
CubePID pid_roll;
CubePID pid_pitch;
MotorManager  motors;

// ── Timing ────────────────────────────────────────────────────
volatile bool loop_flag  = false;
uint32_t      loop_count = 0;

// ── Serial command buffer ─────────────────────────────────────
String serial_buf = "";

// ── System state ──────────────────────────────────────────────
bool system_armed = false;   // Must be explicitly armed via serial

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
    DEBUG_SERIAL.println("[INIT] Starting IMU...");
    if (!imu.begin()) {
        DEBUG_SERIAL.println("[ERROR] IMU init failed — check SPI wiring");
        while (1) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
    }
    imu.calibrate(500);

    // ── Layer 3: PID ──────────────────────────────────────────
    pid_roll.begin (18.0f, 0.8f, 1.4f, 0.001f, -5.0f, 5.0f, 50.0f);
    pid_pitch.begin(18.0f, 0.8f, 1.4f, 0.001f, -5.0f, 5.0f, 50.0f);

    // ── Layer 4: Motors ───────────────────────────────────────
    DEBUG_SERIAL.println("[INIT] Starting motors...");
    if (!motors.begin()) {
        DEBUG_SERIAL.println("[WARN] Motor init issues — check wiring");
    }

    // ── 1kHz timer ────────────────────────────────────────────
    HardwareTimer *loop_timer = new HardwareTimer(TIM2);
    loop_timer->setOverflow(CONTROL_LOOP_HZ, HERTZ_FORMAT);
    loop_timer->attachInterrupt(control_loop_isr);
    loop_timer->resume();

    DEBUG_SERIAL.println("[INIT] All layers ready");
    DEBUG_SERIAL.println("─────────────────────────────────────────────────────");
    DEBUG_SERIAL.println("Serial commands:");
    DEBUG_SERIAL.println("  ARM        — enable motors and start balancing");
    DEBUG_SERIAL.println("  DISARM     — disable motors immediately");
    DEBUG_SERIAL.println("  KP <val>   — set Kp (both axes)");
    DEBUG_SERIAL.println("  KI <val>   — set Ki (both axes)");
    DEBUG_SERIAL.println("  KD <val>   — set Kd (both axes)");
    DEBUG_SERIAL.println("  GAINS      — print current gains");
    DEBUG_SERIAL.println("  RESET      — reset PID state");
    DEBUG_SERIAL.println("  STATUS     — print full system status");
    DEBUG_SERIAL.println("─────────────────────────────────────────────────────");
    DEBUG_SERIAL.println(">>> Type ARM to start balancing <<<");

    blink_status(4);
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

    // ── Layer 2: IMU update ───────────────────────────────────
    imu.update(0.001f);
    float roll  = imu.getRoll();
    float pitch = imu.getPitch();

    // ── Safety cutoff — disarm if tilt too large ──────────────
    if (fabsf(roll) > MAX_TILT_RAD || fabsf(pitch) > MAX_TILT_RAD) {
        if (system_armed) {
            system_armed = false;
            motors.emergency_stop();
            pid_roll.reset();
            pid_pitch.reset();
            DEBUG_SERIAL.println("[SAFETY] Tilt limit exceeded — DISARMED");
        }
        return;
    }

    // ── Layer 3: PID compute ──────────────────────────────────
    float tau_roll  = pid_roll.compute (0.0f, roll);
    float tau_pitch = pid_pitch.compute(0.0f, pitch);

    // ── Layer 4: Motor output ─────────────────────────────────
    if (system_armed) {
        motors.set_torques(tau_roll, tau_pitch, 0.0f);
        motors.update();
    }

    // ── Telemetry every 50ms (every 50 loops) ─────────────────
    if (loop_count % 50 == 0) {
        DEBUG_SERIAL.print(millis());
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(roll  * 57.296f, 2);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(pitch * 57.296f, 2);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(tau_roll,  3);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(tau_pitch, 3);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.println(system_armed ? "ARMED" : "DISARMED");
        digitalWrite(LED_PIN, loop_count % 1000 == 0);
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
        DEBUG_SERIAL.println("[CMD] ARMED — balancing active");

    } else if (cmd == "DISARM") {
        system_armed = false;
        motors.disable();
        pid_roll.reset();
        pid_pitch.reset();
        DEBUG_SERIAL.println("[CMD] DISARMED");

    } else if (cmd.startsWith("KP ")) {
        float v = cmd.substring(3).toFloat();
        pid_roll.set_kp(v); pid_pitch.set_kp(v);
        DEBUG_SERIAL.print("[CMD] Kp="); DEBUG_SERIAL.println(v);

    } else if (cmd.startsWith("KI ")) {
        float v = cmd.substring(3).toFloat();
        pid_roll.set_ki(v); pid_pitch.set_ki(v);
        DEBUG_SERIAL.print("[CMD] Ki="); DEBUG_SERIAL.println(v);

    } else if (cmd.startsWith("KD ")) {
        float v = cmd.substring(3).toFloat();
        pid_roll.set_kd(v); pid_pitch.set_kd(v);
        DEBUG_SERIAL.print("[CMD] Kd="); DEBUG_SERIAL.println(v);

    } else if (cmd == "GAINS") {
        PIDGains g = pid_roll.getGains();
        DEBUG_SERIAL.print("[GAINS] Kp="); DEBUG_SERIAL.print(g.kp);
        DEBUG_SERIAL.print(" Ki=");        DEBUG_SERIAL.print(g.ki);
        DEBUG_SERIAL.print(" Kd=");        DEBUG_SERIAL.println(g.kd);

    } else if (cmd == "RESET") {
        pid_roll.reset(); pid_pitch.reset();
        DEBUG_SERIAL.println("[CMD] PID state reset");

    } else if (cmd == "STATUS") {
        DEBUG_SERIAL.print("[STATUS] Armed=");
        DEBUG_SERIAL.println(system_armed ? "YES" : "NO");
        imu.printAngles();
        pid_roll.print();
        motors.print();

    } else {
        DEBUG_SERIAL.print("[CMD] Unknown: "); DEBUG_SERIAL.println(cmd);
    }
}

// ═════════════════════════════════════════════════════════════
//  ISR & HELPERS
// ═════════════════════════════════════════════════════════════
void control_loop_isr() { loop_flag = true; }

void print_system_info() {
    DEBUG_SERIAL.println("=========================================");
    DEBUG_SERIAL.println("  Self-Balancing Cube — Firmware v0.4");
    DEBUG_SERIAL.println("  L1:HAL  L2:IMU  L3:PID  L4:MOTOR");
    DEBUG_SERIAL.println("=========================================");
}

void blink_status(uint8_t n) {
    for (uint8_t i = 0; i < n; i++) {
        digitalWrite(LED_PIN, HIGH); delay(150);
        digitalWrite(LED_PIN, LOW);  delay(150);
    }
}
