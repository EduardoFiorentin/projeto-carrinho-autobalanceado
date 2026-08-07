#ifndef SELF_BALANCING_ROBOT_H
#define SELF_BALANCING_ROBOT_H

#include <Arduino.h>

#include "BalanceController.h"
#include "KalmanFilter.h"
#include "MotorDriver.h"
#include "Mpu6050Sensor.h"
#include "PowerRamp.h"

class SelfBalancingRobot {
  public:
    SelfBalancingRobot();

    void begin();
    void update();

  private:
    Mpu6050Sensor sensor;
    MotorDriver motors;
    KalmanFilter filter;
    BalanceController controller;
    PowerRamp powerRamp;
    uint32_t lastUpdate = 0;
};

#endif
