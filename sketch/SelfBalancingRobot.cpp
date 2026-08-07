#include "SelfBalancingRobot.h"

#include <math.h>

#include "Config.h"

SelfBalancingRobot::SelfBalancingRobot()
  : filter(Config::KALMAN_Q, Config::KALMAN_R),
    controller(Config::PID_KP, Config::PID_KI, Config::PID_KD, Config::SETPOINT),
    powerRamp(Config::MAX_STEP, Config::MAX_POWER) {
}

void SelfBalancingRobot::begin() {
  Serial.begin(115200);
  delay(100);

  motors.begin();
  sensor.begin();

  controller.begin(Config::CONTROL_PERIOD_MS, Config::MAX_POWER);
}

void SelfBalancingRobot::update() {
  uint32_t now = millis();
  if (now - lastUpdate < Config::CONTROL_PERIOD_MS) {
    return;
  }
  lastUpdate = now;

  sensor.read();
  double rawGyroX = sensor.gyroX();
  double filteredGyroX = filter.update(rawGyroX);
  double controlSignal = controller.compute(filteredGyroX);

  int targetPower = constrain(static_cast<int>(fabs(controlSignal)), 0, Config::MAX_POWER);
  int previousPower = powerRamp.value();
  int power = powerRamp.update(targetPower);
  int delta = power - previousPower;

  Serial.printf("%d\t%d\t%d\t%lf\t%lf\n", targetPower, power, delta, rawGyroX, filteredGyroX);

  motors.apply(controlSignal, power);
}
