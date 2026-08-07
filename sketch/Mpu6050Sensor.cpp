#include "Mpu6050Sensor.h"

#include <math.h>
#include <Wire.h>

#include "Config.h"

namespace {
  // MPU6050 register map entries used by this minimal driver.
  constexpr uint8_t PWR_MGMT_1 = 0x6B;
  constexpr uint8_t SMPLRT_DIV = 0x19;
  constexpr uint8_t CONFIG_REG = 0x1A;
  constexpr uint8_t GYRO_CONFIG = 0x1B;
  constexpr uint8_t ACCEL_CONFIG = 0x1C;
  constexpr uint8_t ACCEL_XOUT_H = 0x3B;
  constexpr uint8_t WHO_AM_I = 0x75;

  // A burst read starting at ACCEL_XOUT_H returns accel, temperature, and gyro.
  constexpr uint8_t SENSOR_FRAME_BYTES = 14;

  // Byte indices inside the 14-byte burst frame.
  constexpr uint8_t ACCEL_X_HIGH_INDEX = 0;
  constexpr uint8_t ACCEL_X_LOW_INDEX = 1;
  constexpr uint8_t ACCEL_Y_HIGH_INDEX = 2;
  constexpr uint8_t ACCEL_Y_LOW_INDEX = 3;
  constexpr uint8_t ACCEL_Z_HIGH_INDEX = 4;
  constexpr uint8_t ACCEL_Z_LOW_INDEX = 5;
  constexpr uint8_t GYRO_X_HIGH_INDEX = 8;
  constexpr uint8_t GYRO_X_LOW_INDEX = 9;
  constexpr float RADIANS_TO_DEGREES = 57.2957795f;

  // MPU6050 sends high byte first; combine as signed two's-complement int16.
  int16_t readInt16(const uint8_t *buf, uint8_t highIndex, uint8_t lowIndex) {
    return static_cast<int16_t>((buf[highIndex] << 8) | buf[lowIndex]);
  }
}

bool Mpu6050Sensor::begin() {
  // ESP32 I2C pins are configurable, so they are supplied by Config.
  Wire.begin(Config::I2C_SDA_PIN, Config::I2C_SCL_PIN);
  Wire.setClock(Config::I2C_CLOCK);

  Serial.println("Init raw MPU-style driver");
  wake();
  configure();

  uint8_t who = readReg(WHO_AM_I);
  Serial.print("WHO_AM_I raw: 0x");
  Serial.println(who, HEX);

  // 0xFF commonly indicates an open bus; 0x00 is also treated as invalid here.
  if (who == 0xFF || who == 0x00) {
    Serial.println("MPU6050 initialization failed");
    return false;
  }

  return true;
}

bool Mpu6050Sensor::read() {
  uint8_t buf[SENSOR_FRAME_BYTES];

  // Keep this as one I2C transaction so accel and gyro samples belong to the
  // same sensor frame.
  if (!readRegs(ACCEL_XOUT_H, SENSOR_FRAME_BYTES, buf)) {
    Serial.println("Error reading register value, retaining last value");
    return false;
  }

  // Decode raw accelerometer and selected gyroscope axis from the burst frame.
  int16_t ax = readInt16(buf, ACCEL_X_HIGH_INDEX, ACCEL_X_LOW_INDEX);
  int16_t ay = readInt16(buf, ACCEL_Y_HIGH_INDEX, ACCEL_Y_LOW_INDEX);
  int16_t az = readInt16(buf, ACCEL_Z_HIGH_INDEX, ACCEL_Z_LOW_INDEX);
  int16_t gx = readInt16(buf, GYRO_X_HIGH_INDEX, GYRO_X_LOW_INDEX);

  // Convert raw accelerometer LSBs into g using the configured full-scale range.
  float accelScale = accelLsbPerG();
  lastAccelX = static_cast<float>(ax) / accelScale;
  lastAccelY = static_cast<float>(ay) / accelScale;
  lastAccelZ = static_cast<float>(az) / accelScale;

  // Remove the calibrated gyro bias before converting to degrees per second.
  long correctedGyroX = static_cast<long>(gx) - gyroXOffset;
  lastGyroX = static_cast<float>(correctedGyroX) / gyroLsbPerDps();

  return true;
}

float Mpu6050Sensor::accelX() const {
  return lastAccelX;
}

float Mpu6050Sensor::accelY() const {
  return lastAccelY;
}

float Mpu6050Sensor::accelZ() const {
  return lastAccelZ;
}

float Mpu6050Sensor::accelAngle() const {
  // For rotation around X, gravity projected on Y/Z gives the absolute tilt.
  // All physical sign decisions are centralized in Config.
  float y = Config::BALANCE_ACCEL_Y_SIGN * lastAccelY;
  float z = Config::BALANCE_ACCEL_Z_SIGN * lastAccelZ;
  return Config::BALANCE_ACCEL_ANGLE_SIGN * atan2f(y, z) * RADIANS_TO_DEGREES;
}

float Mpu6050Sensor::gyroX() const {
  return lastGyroX;
}

float Mpu6050Sensor::balanceGyroRate() const {
  // Keep sign inversion in one place so the control loop never needs ad-hoc -1.
  return Config::BALANCE_GYRO_RATE_SIGN * lastGyroX;
}

bool Mpu6050Sensor::calibrate(uint16_t samples) {
  // Refuse a meaningless calibration request.
  if (samples == 0) {
    return false;
  }

  long sumGyroX = 0;
  uint16_t validSamples = 0;
  Serial.println("Calibrating offsets. Keep the module static in 90degs.");

  // Only valid I2C frames contribute to the average. The final divisor must be
  // the number of valid frames, not the number requested.
  for (uint16_t i = 0; i < samples; i++) {
    uint8_t buf[SENSOR_FRAME_BYTES];
    if (readRegs(ACCEL_XOUT_H, SENSOR_FRAME_BYTES, buf)) {
      int16_t gx = readInt16(buf, GYRO_X_HIGH_INDEX, GYRO_X_LOW_INDEX);
      sumGyroX += gx;
      validSamples++;
    }
    delay(5);
  }

  // Make calibration failure visible to the orchestrator.
  if (validSamples == 0) {
    Serial.println("Calibration failed. No valid samples.");
    return false;
  }

  // Store offset in raw units so runtime conversion keeps one clear scale path.
  gyroXOffset = sumGyroX / validSamples;
  Serial.print("Calibration complete. Offsets adjusted. gx_off: ");
  Serial.println(gyroXOffset);
  Serial.print("Valid calibration samples: ");
  Serial.println(validSamples);

  return true;
}

void Mpu6050Sensor::writeReg(uint8_t reg, uint8_t val) {
  // Simple single-register write transaction.
  Wire.beginTransmission(Config::MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t Mpu6050Sensor::readReg(uint8_t reg) {
  // Repeated-start register read: write register address, then request data.
  Wire.beginTransmission(Config::MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(Config::MPU_ADDR, static_cast<uint8_t>(1));

  if (Wire.available()) {
    return Wire.read();
  }

  return 0xFF;
}

bool Mpu6050Sensor::readRegs(uint8_t reg, uint8_t count, uint8_t *buf) {
  // Request a contiguous register block. Returning false lets the robot stop
  // instead of continuing with stale sensor data.
  Wire.beginTransmission(Config::MPU_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t received = Wire.requestFrom(Config::MPU_ADDR, count);
  if (received != count) {
    return false;
  }

  for (uint8_t i = 0; i < count; i++) {
    buf[i] = Wire.read();
  }

  return true;
}

void Mpu6050Sensor::wake() {
  // Clear sleep bit in PWR_MGMT_1.
  writeReg(PWR_MGMT_1, 0x00);
  delay(50);
}

void Mpu6050Sensor::configure() {
  // Keep sample-rate divider at zero. With DLPF enabled, gyro output is 1 kHz.
  writeReg(SMPLRT_DIV, 0x00);

  // DLPF setting 3 is a moderate low-pass option for early balance tests.
  writeReg(CONFIG_REG, 0x03);

  // Full-scale selections are shifted into their register bit fields.
  writeReg(GYRO_CONFIG, Config::GYRO_RANGE_SEL << 3);
  writeReg(ACCEL_CONFIG, Config::ACCEL_RANGE_SEL << 3);
  delay(20);
}

float Mpu6050Sensor::accelLsbPerG() const {
  // Datasheet sensitivity values for each accelerometer full-scale range.
  switch (Config::ACCEL_RANGE_SEL) {
    case 0: return 16384.0f;
    case 1: return 8192.0f;
    case 2: return 4096.0f;
    case 3: return 2048.0f;
  }

  return 16384.0f;
}

float Mpu6050Sensor::gyroLsbPerDps() const {
  // Datasheet sensitivity values for each gyroscope full-scale range.
  switch (Config::GYRO_RANGE_SEL) {
    case 0: return 131.0f;
    case 1: return 65.5f;
    case 2: return 32.8f;
    case 3: return 16.4f;
  }

  return 131.0f;
}
