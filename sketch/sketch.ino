#include "SelfBalancingRobot.h"

// The Arduino sketch only owns the application lifecycle.
// All control logic stays inside SelfBalancingRobot and its collaborators.
SelfBalancingRobot robot;

void setup() {
  // Initialize hardware, calibrate the sensor, and arm the robot state machine.
  robot.begin();
}

void loop() {
  // Run one non-blocking scheduler pass. Timing is handled internally in micros().
  robot.update();
}
