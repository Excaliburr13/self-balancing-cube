#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "board_config.h"

// ─────────────────────────────────────────────────────────────
//  ICM-42688-P Register Map (subset used)
// ─────────────────────────────────────────────────────────────
#define ICM_WHO_AM_I        0x75    // Should return 0x47
#define ICM_DEVICE_CONFIG   0x11
#define ICM_PWR_MGMT0       0x4E
#define ICM_GYRO_CONFIG0    0x4F
#define ICM_ACCEL_CONFIG0   0x50
#define ICM_GYRO_CONFIG1    0x51
#define ICM_ACCEL_CONFIG1   0x53
#define ICM_INT_CONFIG      0x14
#define ICM_INT_CONFIG1     0x64
#define ICM_INT_SOURCE0     0x65

// Data registers
#define ICM_TEMP_DATA1      0x1D
#define ICM_ACCEL_DATA_X1   0x1F
#define ICM_ACCEL_DATA_X0   0x20
#define ICM_ACCEL_DATA_Y1   0x21
#define ICM_ACCEL_DATA_Y0   0x22
#define ICM_ACCEL_DATA_Z1   0x23
#define ICM_ACCEL_DATA_Z0   0x24
#define ICM_GYRO_DATA_X1    0x25
#define ICM_GYRO_DATA_X0    0x26
#define ICM_GYRO_DATA_Y1    0x27
#define ICM_GYRO_DATA_Y0    0x28
#define ICM_GYRO_DATA_Z1    0x29
#define ICM_GYRO_DATA_Z0    0x2A

// ─────────────────────────────────────────────────────────────
//  Sensor scaling
// ─────────────────────────────────────────────────────────────
// Accel: ±4g range → 8192 LSB/g
// Gyro:  ±500 dps range → 65.5 LSB/dps
#define ACCEL_SCALE         (4.0f * 9.81f / 32768.0f)  // m/s² per LSB
#define GYRO_SCALE          (500.0f / 32768.0f)         // dps per LSB

// ─────────────────────────────────────────────────────────────
//  IMU Data struct
// ─────────────────────────────────────────────────────────────
struct IMUData {
    // Raw physical units
    float ax, ay, az;       // m/s²
    float gx, gy, gz;       // deg/s
    float temp;             // °C

    // Filtered angles (rad)
    float roll;             // rotation around X axis
    float pitch;            // rotation around Y axis

    // Angular rates (rad/s)
    float roll_rate;
    float pitch_rate;
};

// ─────────────────────────────────────────────────────────────
//  IMU Driver Class
// ─────────────────────────────────────────────────────────────
class IMUDriver {
public:
    IMUDriver();

    // Lifecycle
    bool    begin();                    // Init SPI, configure ICM, return true if WHO_AM_I OK
    void    update(float dt);           // Call every control loop tick
    void    calibrate(uint16_t samples = 500); // Gyro bias calibration at startup

    // Getters
    IMUData getData()    const { return _data; }
    float   getRoll()    const { return _data.roll; }
    float   getPitch()   const { return _data.pitch; }
    float   getRollRate()  const { return _data.roll_rate; }
    float   getPitchRate() const { return _data.pitch_rate; }
    bool    isReady()    const { return _initialized; }

    // Debug
    void    printRaw()   const;
    void    printAngles() const;

private:
    // SPI helpers
    void    _write_reg(uint8_t reg, uint8_t val);
    uint8_t _read_reg(uint8_t reg);
    void    _read_burst(uint8_t reg, uint8_t* buf, uint8_t len);
    void    _cs_low()  { digitalWrite(IMU_CS_PIN, LOW);  }
    void    _cs_high() { digitalWrite(IMU_CS_PIN, HIGH); }

    // Raw read
    void    _read_raw();

    // Filter
    void    _complementary_filter(float dt);

    // State
    IMUData  _data;
    bool     _initialized;

    // Gyro bias (removed during calibration)
    float    _gyro_bias_x;
    float    _gyro_bias_y;
    float    _gyro_bias_z;

    // SPI settings
    SPISettings _spi_settings;
};
