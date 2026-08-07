#ifndef MPU6050_SENSOR_H
#define MPU6050_SENSOR_H

#include <Arduino.h>

/**
 * Minimal MPU6050 driver for balance estimation.
 *
 * The class performs one 14-byte burst read per update and exposes calibrated,
 * scaled accelerometer and gyroscope values. It also centralizes the physical
 * balance-axis orientation through Config sign constants.
 */
class Mpu6050Sensor {
  public:
    // Configure I2C and MPU6050 registers. Returns false when the device is invalid.
    bool begin();

    // Read accel + temperature + gyro frame once and update cached scaled values.
    bool read();

    // Acceleration values in g, scaled according to Config::ACCEL_RANGE_SEL.
    float accelX() const;
    float accelY() const;
    float accelZ() const;

    // Balance angle from gravity, in degrees, using the configured physical axis.
    float accelAngle() const;

    // Raw selected gyro axis in degrees per second, after bias calibration.
    float gyroX() const;

    // Gyro rate used by the balance estimator, with centralized sign correction.
    float balanceGyroRate() const;

    // Estimate gyro bias while the robot is motionless. Returns false on no samples.
    bool calibrate(uint16_t samples);

  private:
    // Low-level I2C register helpers.
    void writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
    bool readRegs(uint8_t reg, uint8_t count, uint8_t *buf);

    // Sensor setup sequence.
    void wake();
    void configure();

    // Convert configured full-scale ranges into datasheet scale factors.
    float accelLsbPerG() const;
    float gyroLsbPerDps() const;

    // Gyro bias in raw LSB units, subtracted before converting to deg/s.
    long gyroXOffset = 0;

    // Last valid scaled readings. They are only updated after a successful frame read.
    float lastAccelX = 0.0f;
    float lastAccelY = 0.0f;
    float lastAccelZ = 0.0f;
    float lastGyroX = 0.0f;
};

#endif
