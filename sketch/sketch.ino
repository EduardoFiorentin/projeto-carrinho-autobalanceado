#include "SelfBalancingRobot.h"

SelfBalancingRobot robot;

void setup() {
  robot.begin();
}

void loop() {
  robot.update();
}
