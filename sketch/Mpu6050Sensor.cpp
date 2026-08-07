#include "Mpu6050Sensor.h"

#include <Wire.h>

#include "Config.h"

namespace {
  constexpr uint8_t PWR_MGMT_1 = 0x6B;
  constexpr uint8_t SMPLRT_DIV = 0x19;
  constexpr uint8_t CONFIG_REG = 0x1A;
  constexpr uint8_t GYRO_CONFIG = 0x1B;
  constexpr uint8_t ACCEL_CONFIG = 0x1C;
  constexpr uint8_t ACCEL_XOUT_H = 0x3B;
  constexpr uint8_t WHO_AM_I = 0x75;

  constexpr uint8_t SENSOR_FRAME_BYTES = 14;
  constexpr uint8_t GYRO_X_HIGH_INDEX = 8;
  constexpr uint8_t GYRO_X_LOW_INDEX = 9;
}

bool Mpu6050Sensor::begin() {
  Wire.begin(Config::I2C_SDA_PIN, Config::I2C_SCL_PIN);
  Wire.setClock(Config::I2C_CLOCK);

  Serial.println("Init raw MPU-style driver (ignora WHO_AM_I)");
  wake();
  configure();

  uint8_t who = readReg(WHO_AM_I);
  Serial.print("WHO_AM_I raw: 0x");
  Serial.println(who, HEX);

  return who != 0xFF;
}

bool Mpu6050Sensor::read() {
  uint8_t buf[SENSOR_FRAME_BYTES];

  if (!readRegs(ACCEL_XOUT_H, SENSOR_FRAME_BYTES, buf)) {
    Serial.println("Error reading register value, retaining last value");
    return false;
  }

  int16_t gx = (buf[GYRO_X_HIGH_INDEX] << 8) | buf[GYRO_X_LOW_INDEX];
  long correctedGyroX = static_cast<long>(gx) - gyroXOffset;
  lastGyroX = static_cast<float>(correctedGyroX) / gyroLsbPerDps();

  return true;
}

float Mpu6050Sensor::gyroX() const {
  return lastGyroX;
}

void Mpu6050Sensor::calibrate(uint16_t samples) {
  if (samples == 0) {
    return;
  }

  long sumGyroX = 0;
  Serial.println("Calibrating offsets. Keep the module static in 90degs.");

  for (uint16_t i = 0; i < samples; i++) {
    uint8_t buf[SENSOR_FRAME_BYTES];
    if (readRegs(ACCEL_XOUT_H, SENSOR_FRAME_BYTES, buf)) {
      int16_t gx = (buf[GYRO_X_HIGH_INDEX] << 8) | buf[GYRO_X_LOW_INDEX];
      sumGyroX += gx;
    }
    delay(5);
  }

  gyroXOffset = sumGyroX / samples;
  Serial.print("Calibration complete. Offsets adjusted. gx_off: ");
  Serial.println(gyroXOffset);
}

void Mpu6050Sensor::writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(Config::MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t Mpu6050Sensor::readReg(uint8_t reg) {
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
  writeReg(PWR_MGMT_1, 0x00);
  delay(50);
}

void Mpu6050Sensor::configure() {
  writeReg(SMPLRT_DIV, 0x00);
  writeReg(CONFIG_REG, 0x03);
  writeReg(GYRO_CONFIG, Config::GYRO_RANGE_SEL << 3);
  writeReg(ACCEL_CONFIG, Config::ACCEL_RANGE_SEL << 3);
  delay(20);
}

float Mpu6050Sensor::gyroLsbPerDps() const {
  switch (Config::GYRO_RANGE_SEL) {
    case 0: return 131.0f;
    case 1: return 65.5f;
    case 2: return 32.8f;
    case 3: return 16.4f;
  }

  return 131.0f;
}
