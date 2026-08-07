#include "MotorDriver.h"

#include <math.h>

#include "Config.h"

namespace {
  constexpr ledc_channel_t LED_CHANNEL_E1 = LEDC_CHANNEL_0;
  constexpr ledc_channel_t LED_CHANNEL_E2 = LEDC_CHANNEL_1;
  constexpr ledc_channel_t LED_CHANNEL_D1 = LEDC_CHANNEL_2;
  constexpr ledc_channel_t LED_CHANNEL_D2 = LEDC_CHANNEL_3;
}

void MotorDriver::begin() {
  ledc_timer_config_t timer = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .duty_resolution  = Config::PWM_RESOLUTION,
    .timer_num        = LEDC_TIMER_0,
    .freq_hz          = Config::PWM_FREQ,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer);

  configChannel(Config::MOTOR_E1, LED_CHANNEL_E1);
  configChannel(Config::MOTOR_E2, LED_CHANNEL_E2);
  configChannel(Config::MOTOR_D1, LED_CHANNEL_D1);
  configChannel(Config::MOTOR_D2, LED_CHANNEL_D2);
}

void MotorDriver::stop() {
  setDuty(LED_CHANNEL_E1, 0);
  setDuty(LED_CHANNEL_E2, 0);
  setDuty(LED_CHANNEL_D1, 0);
  setDuty(LED_CHANNEL_D2, 0);
}

void MotorDriver::apply(double controlSignal, int power) {
  if (fabs(controlSignal) <= Config::DEAD_BAND) {
    stop();
  } else if (controlSignal >= 0) {
    driveForward(power);
  } else {
    driveBackward(power);
  }
}

void MotorDriver::driveForward(int power) {
  setDuty(LED_CHANNEL_E1, Config::LEFT_ACT_PRESET + power);
  setDuty(LED_CHANNEL_E2, 0);
  setDuty(LED_CHANNEL_D1, Config::RIGHT_ACT_PRESET + power);
  setDuty(LED_CHANNEL_D2, 0);
}

void MotorDriver::driveBackward(int power) {
  setDuty(LED_CHANNEL_E1, 0);
  setDuty(LED_CHANNEL_E2, Config::LEFT_ACT_PRESET + power);
  setDuty(LED_CHANNEL_D1, 0);
  setDuty(LED_CHANNEL_D2, Config::RIGHT_ACT_PRESET + power);
}

void MotorDriver::configChannel(int pin, ledc_channel_t channel) {
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

void MotorDriver::setDuty(ledc_channel_t channel, uint32_t duty) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}
