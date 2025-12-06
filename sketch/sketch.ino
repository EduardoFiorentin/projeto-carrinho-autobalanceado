#include "driver/ledc.h"
#include "esp_err.h"

// declarations pwm.ino
extern void setup_pwm_system();
extern void config_pwm_channel(int pin, ledc_channel_t ch);
extern void set_duty(ledc_channel_t channel, uint32_t duty);

extern const ledc_channel_t ledChannelE1;
extern const ledc_channel_t ledChannelE2;
extern const ledc_channel_t ledChannelD1;
extern const ledc_channel_t ledChannelD2;

// declarations mpu.ino
extern void setup_mpu6050();
extern float read_gyroscope_x();

void setup() {
  Serial.begin(115200);
  delay(100);
  setup_pwm_system();
  setup_mpu6050();
  // set_duty(ledChannelD1, 0);
  // set_duty(ledChannelD2, 0);

}

void loop() {

  // teste mpu6050
  Serial.println(read_gyroscope_x());
  delay(10);

  // set_duty(ledChannelE1, 0);
  // set_duty(ledChannelE2, 200);
  // set_duty(ledChannelD1, 0);
  // set_duty(ledChannelD2, 0);
  // delay(1000);
  // set_duty(ledChannelE1, 200);
  // set_duty(ledChannelE2, 0);
  // set_duty(ledChannelD1, 0);
  // set_duty(ledChannelD2, 0);
  // delay(1000);

  // set_duty(ledChannelD1, 0);
  // set_duty(ledChannelD2, 0);
  // set_duty(ledChannelE1, 0);
  // set_duty(ledChannelE2, 200);
  // delay(1000);

  // set_duty(ledChannelD1, 0);
  // set_duty(ledChannelD2, 0);
  // set_duty(ledChannelE1, 200);
  // set_duty(ledChannelE2, 0);
  // delay(1000);


  // set_duty(ledChannelE1, 0);
  // set_duty(ledChannelE2, 170);
  // // delay(1000);
  // set_duty(ledChannelD1, 0);
  // set_duty(ledChannelD2, 255);
  // delay(1000);
  // set_duty(ledChannelE1, 0);
  // set_duty(ledChannelE2, 0);
  // set_duty(ledChannelD1, 0);
  // set_duty(ledChannelD2, 0);
  // delay(1000);

  // digitalWrite(32, HIGH);
  // digitalWrite(33, LOW);
}

