#include "MotorDriver.h"

#include <math.h>

#include "driver/ledc.h"

#include "Config.h"

namespace {
  // L298N direction and enable pins for the current ESP32 wiring.
  constexpr int L298N_IN1 = 26;
  constexpr int L298N_IN2 = 25;
  constexpr int L298N_IN3 = 33;
  constexpr int L298N_IN4 = 32;
  constexpr int L298N_ENA = 14;
  constexpr int L298N_ENB = 16;

  // Minimum duty that produces useful torque. These values compensate the
  // physical dead zone of the L298N + motor set and must be measured per motor.
  constexpr int RIGHT_MIN_EFFECTIVE_DUTY = 370;
  constexpr int LEFT_MIN_EFFECTIVE_DUTY = 370;

  // Logical motor direction correction. Flip to -1 if a motor reacts inverted.
  constexpr int RIGHT_MOTOR_DIRECTION_SIGN = 1;
  constexpr int LEFT_MOTOR_DIRECTION_SIGN = 1;

  // ESP32 LEDC PWM configuration shared by both motor enable pins.
  constexpr int PWM_FREQ = 5000;
  constexpr ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_9_BIT;
  constexpr int PWM_MAX_DUTY = 511;

  // Complete hardware description for one motor channel.
  struct MotorPort {
    int forwardPin;
    int backwardPin;
    int enablePin;
    ledc_channel_t pwmChannel;
    int minEffectiveDuty;
    int directionSign;
  };

  // Right motor: IN1/IN2 select direction, ENA receives PWM.
  const MotorPort RIGHT_MOTOR = {
    L298N_IN1,
    L298N_IN2,
    L298N_ENA,
    LEDC_CHANNEL_0,
    RIGHT_MIN_EFFECTIVE_DUTY,
    RIGHT_MOTOR_DIRECTION_SIGN
  };

  // Left motor is wired/mounted opposite, so logical forward uses IN4/IN3.
  const MotorPort LEFT_MOTOR = {
    L298N_IN4,
    L298N_IN3,
    L298N_ENB,
    LEDC_CHANNEL_1,
    LEFT_MIN_EFFECTIVE_DUTY,
    LEFT_MOTOR_DIRECTION_SIGN
  };

  void configPwmChannel(int pin, ledc_channel_t channel) {
    // Attach one LEDC channel to one enable pin. Both channels share timer 0.
    ledc_channel_config_t channelConfig = {
      .gpio_num   = pin,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel    = channel,
      .intr_type  = LEDC_INTR_DISABLE,
      .timer_sel  = LEDC_TIMER_0,
      .duty       = 0,
      .hpoint     = 0
    };
    ledc_channel_config(&channelConfig);
  }

  void setDuty(ledc_channel_t channel, uint32_t duty) {
    // ESP-IDF LEDC requires setting the duty and then explicitly updating it.
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
  }

  void configureMotor(const MotorPort& motor) {
    // Direction pins start low so the bridge is disabled before PWM is applied.
    pinMode(motor.forwardPin, OUTPUT);
    pinMode(motor.backwardPin, OUTPUT);
    digitalWrite(motor.forwardPin, LOW);
    digitalWrite(motor.backwardPin, LOW);
    configPwmChannel(motor.enablePin, motor.pwmChannel);
    setDuty(motor.pwmChannel, 0);
  }

  void setMotorPower(const MotorPort& motor, int power) {
    // The public command range is 0..MAX_POWER. Convert it into the hardware
    // duty range while preserving a true zero at command zero.
    power = constrain(power, 0, Config::MAX_POWER);
    if (power == 0) {
      setDuty(motor.pwmChannel, 0);
      return;
    }

    // Non-zero commands are mapped monotonically from the measured minimum
    // effective duty up to the LEDC maximum duty.
    float normalizedPower = static_cast<float>(power) / Config::MAX_POWER;
    int duty = motor.minEffectiveDuty
      + static_cast<int>(roundf(normalizedPower * (PWM_MAX_DUTY - motor.minEffectiveDuty)));
    duty = constrain(duty, motor.minEffectiveDuty, PWM_MAX_DUTY);
    setDuty(motor.pwmChannel, duty);
  }

  void stopMotor(const MotorPort& motor) {
    // Stop by disabling both direction inputs and removing PWM.
    digitalWrite(motor.forwardPin, LOW);
    digitalWrite(motor.backwardPin, LOW);
    setDuty(motor.pwmChannel, 0);
  }

  void driveMotorForward(const MotorPort& motor, int power) {
    // L298N direction: one input high, the opposite input low.
    digitalWrite(motor.forwardPin, HIGH);
    digitalWrite(motor.backwardPin, LOW);
    setMotorPower(motor, power);
  }

  void driveMotorBackward(const MotorPort& motor, int power) {
    // Reverse direction by swapping which bridge input is driven high.
    digitalWrite(motor.forwardPin, LOW);
    digitalWrite(motor.backwardPin, HIGH);
    setMotorPower(motor, power);
  }

  void driveSignedMotor(const MotorPort& motor, int command) {
    // Apply the per-motor logical inversion in exactly one place.
    int adjustedCommand = command * motor.directionSign;

    // Positive and negative signed commands select direction; magnitude selects
    // requested power after dead-zone compensation.
    if (adjustedCommand > 0) {
      driveMotorForward(motor, adjustedCommand);
    } else if (adjustedCommand < 0) {
      driveMotorBackward(motor, -adjustedCommand);
    } else {
      stopMotor(motor);
    }
  }
}

void MotorDriver::begin() {
  // Configure one LEDC timer for both motor enable channels.
  ledc_timer_config_t timer = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .duty_resolution  = PWM_RESOLUTION,
    .timer_num        = LEDC_TIMER_0,
    .freq_hz          = PWM_FREQ,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer);

  // Configure both motor ports and guarantee a stopped initial state.
  configureMotor(RIGHT_MOTOR);
  configureMotor(LEFT_MOTOR);
}

void MotorDriver::stop() {
  // Stop both sides symmetrically.
  stopMotor(RIGHT_MOTOR);
  stopMotor(LEFT_MOTOR);
}

void MotorDriver::apply(double command) {
  // The command is signed: sign = direction, magnitude = control effort.
  double magnitude = fabs(command);

  // Control deadband is logical; physical motor dead-zone is handled separately
  // in setMotorPower().
  if (magnitude <= Config::CONTROL_DEADBAND) {
    stop();
  } else {
    // ceil() preserves tiny non-zero commands as power 1 instead of truncating
    // them to zero.
    int power = constrain(static_cast<int>(ceil(magnitude)), 1, Config::MAX_POWER);
    int signedPower = command >= 0.0 ? power : -power;
    driveSignedMotor(RIGHT_MOTOR, signedPower);
    driveSignedMotor(LEFT_MOTOR, signedPower);
  }
}

void MotorDriver::driveForward(int power) {
  // Diagnostic helper for manually checking that both wheels move forward.
  power = constrain(power, 0, Config::MAX_POWER);
  driveSignedMotor(RIGHT_MOTOR, power);
  driveSignedMotor(LEFT_MOTOR, power);
}

void MotorDriver::driveBackward(int power) {
  // Diagnostic helper for manually checking reverse direction.
  power = constrain(power, 0, Config::MAX_POWER);
  driveSignedMotor(RIGHT_MOTOR, -power);
  driveSignedMotor(LEFT_MOTOR, -power);
}


void MotorDriver::testRightRaw(int duty) {
  duty = constrain(duty, -511, 511);

  if (duty > 0) {
      digitalWrite(RIGHT_MOTOR.forwardPin, HIGH);
      digitalWrite(RIGHT_MOTOR.backwardPin, LOW);
      setDuty(RIGHT_MOTOR.pwmChannel, duty);
  } else if (duty < 0) {
      digitalWrite(RIGHT_MOTOR.forwardPin, LOW);
      digitalWrite(RIGHT_MOTOR.backwardPin, HIGH);
      setDuty(RIGHT_MOTOR.pwmChannel, -duty);
  } else {
      stopMotor(RIGHT_MOTOR);
  }
}

void MotorDriver::testLeftRaw(int duty) {
  duty = constrain(duty, -PWM_MAX_DUTY, PWM_MAX_DUTY);

  if (duty > 0) {
    digitalWrite(LEFT_MOTOR.forwardPin, HIGH);
    digitalWrite(LEFT_MOTOR.backwardPin, LOW);
    setDuty(LEFT_MOTOR.pwmChannel, duty);
  } else if (duty < 0) {
    digitalWrite(LEFT_MOTOR.forwardPin, LOW);
    digitalWrite(LEFT_MOTOR.backwardPin, HIGH);
    setDuty(LEFT_MOTOR.pwmChannel, -duty);
  } else {
    stopMotor(LEFT_MOTOR);
  }
}