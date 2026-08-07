#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include "driver/ledc.h"

class MotorDriver {
  public:
    void begin();
    void stop();
    void apply(double controlSignal, int power);
    void driveForward(int power);
    void driveBackward(int power);

  private:
    void configChannel(int pin, ledc_channel_t channel);
    void setDuty(ledc_channel_t channel, uint32_t duty);
};

#endif
