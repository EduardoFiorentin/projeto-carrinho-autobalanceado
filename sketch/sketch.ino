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

// #include "MotorDriver.h"

// MotorDriver motors;

// void setup() {
//     Serial.begin(115200);
//     motors.begin();
//     motors.stop();

//     Serial.println("R <duty> | L <duty> | S");
// }

// void loop() {
//     if (!Serial.available()) {
//         return;
//     }

//     char motor = Serial.read();

//     if (motor == 'S' || motor == 's') {
//         motors.stop();
//         Serial.println("STOP");
//         return;
//     }

//     if (motor != 'R' && motor != 'r' &&
//         motor != 'L' && motor != 'l') {
//         return;
//     }

//     int duty = Serial.parseInt();

//     if (motor == 'R' || motor == 'r') {
//         motors.testRightRaw(duty);
//         Serial.printf("RIGHT: %d\n", duty);
//     } else {
//         motors.testLeftRaw(duty);
//         Serial.printf("LEFT: %d\n", duty);
//     }

//     while (Serial.available()) {
//         Serial.read();
//     }
// }

