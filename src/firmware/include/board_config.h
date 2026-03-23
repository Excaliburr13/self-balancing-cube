#pragma once

// ─────────────────────────────────────────────────────────────
//  Self-Balancing Cube — Board Configuration
//  Target: STM32 Nucleo F446RE
//  Framework: Arduino (PlatformIO)
// ─────────────────────────────────────────────────────────────

// ── System ────────────────────────────────────────────────────
#define SYS_CLOCK_MHZ       180     // STM32F446RE max clock
#define CONTROL_LOOP_HZ     1000    // PID loop frequency
#define CONTROL_LOOP_US     1000    // = 1 / CONTROL_LOOP_HZ in µs
#define SERIAL_BAUD         115200

// ── IMU (ICM-42688-P) via SPI ─────────────────────────────────
#define IMU_SPI_BUS         SPI1
#define IMU_CS_PIN          PA4     // Chip select
#define IMU_SCK_PIN         PA5     // Clock
#define IMU_MISO_PIN        PA6     // Master in slave out
#define IMU_MOSI_PIN        PA7     // Master out slave in
#define IMU_INT_PIN         PB0     // Data ready interrupt
#define IMU_SPI_FREQ        8000000 // 8 MHz SPI clock

// ── Motor 1 — X axis reaction wheel ──────────────────────────
#define MOTOR1_PWM_PIN      PA8
#define MOTOR1_DIR_PIN      PA9
#define MOTOR1_EN_PIN       PA10

// ── Motor 2 — Y axis reaction wheel ──────────────────────────
#define MOTOR2_PWM_PIN      PB6
#define MOTOR2_DIR_PIN      PB7
#define MOTOR2_EN_PIN       PB8

// ── Motor 3 — Z axis reaction wheel ──────────────────────────
#define MOTOR3_PWM_PIN      PC6
#define MOTOR3_DIR_PIN      PC7
#define MOTOR3_EN_PIN       PC8

// ── PWM ───────────────────────────────────────────────────────
#define PWM_FREQUENCY_HZ    20000   // 20kHz — above audible range
#define PWM_RESOLUTION_BITS 12      // 0–4095 duty cycle range
#define PWM_MAX             4095

// ── UART Debug ────────────────────────────────────────────────
#define DEBUG_SERIAL        Serial  // USB CDC via ST-Link
#define DEBUG_BAUD          115200

// ── LED (onboard Nucleo) ──────────────────────────────────────
#define LED_PIN             LED_BUILTIN  // PA5 on Nucleo F446RE

// ── Safety limits ─────────────────────────────────────────────
#define MAX_TILT_RAD        0.52f   // ~30 degrees — cut motors beyond this
#define MAX_WHEEL_RPM       3000.0f
#define MOTOR_DEADBAND      0.03f   // Minimum duty below which motor is off

// ── Filter coefficient ────────────────────────────────────────
// Complementary filter: angle = alpha*(angle+gyro*dt) + (1-alpha)*accel_angle
#define COMP_FILTER_ALPHA   0.98f
