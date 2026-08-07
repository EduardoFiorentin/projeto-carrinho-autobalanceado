#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

/**
 * Hardware abstraction for the two DC motors through an L298N bridge.
 *
 * The rest of the robot sends only a signed command. Pin mapping, PWM channels,
 * motor inversion, and minimum effective duty compensation stay inside the
 * implementation file.
 */
class MotorDriver {
  public:
    // Configure direction pins, LEDC timer, and PWM channels.
    void begin();

    // Immediately disable both motors and set PWM duty to zero.
    void stop();

    // Apply a signed command: positive = forward, negative = backward.
    void apply(double command);

    // Manual diagnostic helpers that still respect per-motor direction signs.
    void driveForward(int power);
    void driveBackward(int power);
};

#endif
