#include "driver/ledc.h"
#include "esp_err.h"

const int MOTOR_D1 = 25;
const int MOTOR_D2 = 26;
const int MOTOR_E1 = 32;
const int MOTOR_E2 = 33;

const int freq = 5000;  
const int resolution = LEDC_TIMER_8_BIT;

const ledc_channel_t ledChannelE1 = LEDC_CHANNEL_0;
const ledc_channel_t ledChannelE2 = LEDC_CHANNEL_1;
const ledc_channel_t ledChannelD1 = LEDC_CHANNEL_2;
const ledc_channel_t ledChannelD2 = LEDC_CHANNEL_3;

void setup_pwm_system() {

  ledc_timer_config_t timer = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .duty_resolution  = LEDC_TIMER_8_BIT,
    .timer_num        = LEDC_TIMER_0,
    .freq_hz          = freq,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer);

  
  config_pwm_channel(MOTOR_E1, ledChannelE1);
  config_pwm_channel(MOTOR_E2, ledChannelE2);
  config_pwm_channel(MOTOR_D1, ledChannelD1);
  config_pwm_channel(MOTOR_D2, ledChannelD2);
}

void config_pwm_channel(int pin, ledc_channel_t ch) {
  ledc_channel_config_t channel = {
    .gpio_num   = pin,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel    = ch,
    .intr_type  = LEDC_INTR_DISABLE,
    .timer_sel  = LEDC_TIMER_0,
    .duty       = 0,
    .hpoint     = 0
  };
  ledc_channel_config(&channel);
};

void set_duty(ledc_channel_t channel, uint32_t duty) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}