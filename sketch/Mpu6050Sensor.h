#ifndef MPU6050_SENSOR_H
#define MPU6050_SENSOR_H

#include <Arduino.h>

class Mpu6050Sensor {
  public:
    bool begin();
    bool read();
    float gyroX() const;
    void calibrate(uint16_t samples);

  private:
    void writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
    bool readRegs(uint8_t reg, uint8_t count, uint8_t *buf);
    void wake();
    void configure();
    float gyroLsbPerDps() const;

    long gyroXOffset = 0;
    float lastGyroX = 0.0f;
};

#endif
