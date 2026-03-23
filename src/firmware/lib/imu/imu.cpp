#include "imu.h"

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────
IMUDriver::IMUDriver()
    : _initialized(false),
      _gyro_bias_x(0), _gyro_bias_y(0), _gyro_bias_z(0),
      _spi_settings(IMU_SPI_FREQ, MSBFIRST, SPI_MODE3)
{
    memset(&_data, 0, sizeof(_data));
}

// ─────────────────────────────────────────────────────────────
//  begin() — init SPI bus, configure ICM-42688-P
// ─────────────────────────────────────────────────────────────
bool IMUDriver::begin() {
    // Init SPI
    SPI.begin();
    pinMode(IMU_CS_PIN, OUTPUT);
    _cs_high();
    delay(10);

    // Soft reset
    _write_reg(ICM_DEVICE_CONFIG, 0x01);
    delay(10);

    // Verify WHO_AM_I
    uint8_t who = _read_reg(ICM_WHO_AM_I);
    if (who != 0x47) {
        DEBUG_SERIAL.print("[IMU] WHO_AM_I failed: 0x");
        DEBUG_SERIAL.println(who, HEX);
        return false;
    }
    DEBUG_SERIAL.println("[IMU] ICM-42688-P detected OK (0x47)");

    // Power on accel + gyro in low-noise mode
    _write_reg(ICM_PWR_MGMT0, 0x0F);
    delay(1);

    // Gyro: ±500 dps, 1kHz ODR
    _write_reg(ICM_GYRO_CONFIG0, 0x26);

    // Accel: ±4g, 1kHz ODR
    _write_reg(ICM_ACCEL_CONFIG0, 0x26);

    // Gyro filter: BW = 73Hz (matches 1kHz loop)
    _write_reg(ICM_GYRO_CONFIG1, 0x01);

    // Accel filter: BW = 73Hz
    _write_reg(ICM_ACCEL_CONFIG1, 0x01);

    delay(50);  // Let sensor stabilize

    _initialized = true;
    DEBUG_SERIAL.println("[IMU] Configuration complete");
    return true;
}

// ─────────────────────────────────────────────────────────────
//  calibrate() — measure gyro bias at rest
//  IMPORTANT: cube must be stationary during this call
// ─────────────────────────────────────────────────────────────
void IMUDriver::calibrate(uint16_t samples) {
    DEBUG_SERIAL.print("[IMU] Calibrating gyro bias (");
    DEBUG_SERIAL.print(samples);
    DEBUG_SERIAL.println(" samples) — keep cube still...");

    double bx = 0, by = 0, bz = 0;

    for (uint16_t i = 0; i < samples; i++) {
        _read_raw();
        bx += _data.gx;
        by += _data.gy;
        bz += _data.gz;
        delay(2);
    }

    _gyro_bias_x = bx / samples;
    _gyro_bias_y = by / samples;
    _gyro_bias_z = bz / samples;

    DEBUG_SERIAL.print("[IMU] Bias: gx=");
    DEBUG_SERIAL.print(_gyro_bias_x, 4);
    DEBUG_SERIAL.print(" gy=");
    DEBUG_SERIAL.print(_gyro_bias_y, 4);
    DEBUG_SERIAL.print(" gz=");
    DEBUG_SERIAL.println(_gyro_bias_z, 4);
}

// ─────────────────────────────────────────────────────────────
//  update() — main call every 1ms from control loop
// ─────────────────────────────────────────────────────────────
void IMUDriver::update(float dt) {
    if (!_initialized) return;
    _read_raw();
    _complementary_filter(dt);
}

// ─────────────────────────────────────────────────────────────
//  _read_raw() — burst read all 12 bytes of accel+gyro data
// ─────────────────────────────────────────────────────────────
void IMUDriver::_read_raw() {
    uint8_t buf[12];
    _read_burst(ICM_ACCEL_DATA_X1, buf, 12);

    // Combine high/low bytes into signed 16-bit
    int16_t raw_ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t raw_gx = (int16_t)((buf[6] << 8) | buf[7]);
    int16_t raw_gy = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t raw_gz = (int16_t)((buf[10]<< 8) | buf[11]);

    // Scale to physical units
    _data.ax = raw_ax * ACCEL_SCALE;
    _data.ay = raw_ay * ACCEL_SCALE;
    _data.az = raw_az * ACCEL_SCALE;

    // Scale and remove gyro bias
    _data.gx = (raw_gx * GYRO_SCALE) - _gyro_bias_x;
    _data.gy = (raw_gy * GYRO_SCALE) - _gyro_bias_y;
    _data.gz = (raw_gz * GYRO_SCALE) - _gyro_bias_z;
}

// ─────────────────────────────────────────────────────────────
//  _complementary_filter()
//  Fuses gyro (fast, drifts) with accel (slow, noisy)
//  alpha=0.98 → 98% gyro, 2% accel correction
// ─────────────────────────────────────────────────────────────
void IMUDriver::_complementary_filter(float dt) {
    // Accel-derived angles (in radians)
    float accel_roll  = atan2f(_data.ay, _data.az);
    float accel_pitch = atan2f(-_data.ax,
                         sqrtf(_data.ay*_data.ay + _data.az*_data.az));

    // Gyro rates in rad/s
    float gyro_roll_rate  = _data.gx * DEG_TO_RAD;
    float gyro_pitch_rate = _data.gy * DEG_TO_RAD;

    // Complementary filter
    _data.roll  = COMP_FILTER_ALPHA * (_data.roll  + gyro_roll_rate  * dt)
                + (1.0f - COMP_FILTER_ALPHA) * accel_roll;

    _data.pitch = COMP_FILTER_ALPHA * (_data.pitch + gyro_pitch_rate * dt)
                + (1.0f - COMP_FILTER_ALPHA) * accel_pitch;

    // Store rates for PID derivative term
    _data.roll_rate  = gyro_roll_rate;
    _data.pitch_rate = gyro_pitch_rate;
}

// ─────────────────────────────────────────────────────────────
//  SPI helpers
// ─────────────────────────────────────────────────────────────
void IMUDriver::_write_reg(uint8_t reg, uint8_t val) {
    SPI.beginTransaction(_spi_settings);
    _cs_low();
    SPI.transfer(reg & 0x7F);   // MSB=0 → write
    SPI.transfer(val);
    _cs_high();
    SPI.endTransaction();
}

uint8_t IMUDriver::_read_reg(uint8_t reg) {
    SPI.beginTransaction(_spi_settings);
    _cs_low();
    SPI.transfer(reg | 0x80);   // MSB=1 → read
    uint8_t val = SPI.transfer(0x00);
    _cs_high();
    SPI.endTransaction();
    return val;
}

void IMUDriver::_read_burst(uint8_t reg, uint8_t* buf, uint8_t len) {
    SPI.beginTransaction(_spi_settings);
    _cs_low();
    SPI.transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = SPI.transfer(0x00);
    }
    _cs_high();
    SPI.endTransaction();
}

// ─────────────────────────────────────────────────────────────
//  Debug helpers
// ─────────────────────────────────────────────────────────────
void IMUDriver::printRaw() const {
    DEBUG_SERIAL.print("ax="); DEBUG_SERIAL.print(_data.ax, 3);
    DEBUG_SERIAL.print(" ay="); DEBUG_SERIAL.print(_data.ay, 3);
    DEBUG_SERIAL.print(" az="); DEBUG_SERIAL.print(_data.az, 3);
    DEBUG_SERIAL.print(" | gx="); DEBUG_SERIAL.print(_data.gx, 3);
    DEBUG_SERIAL.print(" gy="); DEBUG_SERIAL.print(_data.gy, 3);
    DEBUG_SERIAL.print(" gz="); DEBUG_SERIAL.println(_data.gz, 3);
}

void IMUDriver::printAngles() const {
    DEBUG_SERIAL.print("[IMU] roll=");
    DEBUG_SERIAL.print(_data.roll * (180.0f / 3.14159f), 2);
    DEBUG_SERIAL.print("°  pitch=");
    DEBUG_SERIAL.print(_data.pitch * (180.0f / 3.14159f), 2);
    DEBUG_SERIAL.println("°");
}
