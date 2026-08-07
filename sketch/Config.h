#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "driver/ledc.h"

namespace Config {
  constexpr uint32_t CONTROL_PERIOD_MS = 10;

  constexpr int MAX_POWER = 140;
  constexpr int MAX_STEP = 5;
  constexpr double DEAD_BAND = 5.0;

  constexpr int RIGHT_ACT_PRESET = 370;
  constexpr int LEFT_ACT_PRESET = 370;

  constexpr double PID_KP = 0.8;
  constexpr double PID_KI = 0.0;
  constexpr double PID_KD = 0.0;
  constexpr double SETPOINT = 0.0;

  constexpr double KALMAN_Q = 0.0001;
  constexpr double KALMAN_R = 0.007;

  constexpr int MOTOR_D1 = 26;
  constexpr int MOTOR_D2 = 25;
  constexpr int MOTOR_E1 = 32;
  constexpr int MOTOR_E2 = 33;

  constexpr int PWM_FREQ = 5000;
  constexpr ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_9_BIT;

  constexpr uint8_t MPU_ADDR = 0x68;
  constexpr int I2C_SDA_PIN = 21;
  constexpr int I2C_SCL_PIN = 22;
  constexpr uint32_t I2C_CLOCK = 100000;

  constexpr uint8_t ACCEL_RANGE_SEL = 2;
  constexpr uint8_t GYRO_RANGE_SEL = 2;
}

#endif
