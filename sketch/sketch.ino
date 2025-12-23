#include "driver/ledc.h"
#include "esp_err.h"
#include <PID_v1.h>
#include "KalmanFilter.h"

#define pln Serial.println
#define pr Serial.print

// pwm level where motors start rotation
// right - ledChannelD1 and ledChannelD2
// left - ledChannelE1 and ledChannelE2
// these values need to be reconfigured for each new actuator
#define RIGHT_ACT_PRESET  370
#define LEFT_ACT_PRESET   370

#define KALMAN_Q  0.0001
#define KALMAN_R  0.007 

const uint32_t CONTROL_PERIOD_MS = 5;


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
// extern void calibrateOffsets(uint16_t samples);


// time controll 
long    max_time_func = 10000;
long    initTime = millis();

// variables
double   gyro_x = 0.0;
double   gyro_val = 0.0;

double   pid_out = 0;
double   setpoint = 0.0;
int      power = 0;

// PID controll 
// Kp = 1.25
// Ki = 0.8
double  Kp=4.0, Ki=0.0, Kd=0.0;
PID pid(&gyro_val, &pid_out, &setpoint, Kp, Ki, Kd, DIRECT);

KalmanFilter filter(&gyro_x, &gyro_val, KALMAN_Q, KALMAN_R);

void setup() {
  Serial.begin(115200);
  delay(100);
  setup_pwm_system();
  setup_mpu6050();
  // calibrateOffsets(500);

  // setup pid controll
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-140, 140);
  pid.SetSampleTime(CONTROL_PERIOD_MS);

}


static uint32_t lastControl = 0;

void loop() {

  // Frequency controll 
  uint32_t now = millis();
  if (now - lastControl < CONTROL_PERIOD_MS) return;
  lastControl = now;

  gyro_x = (double) read_gyroscope_x();
  
  // do filter on gyro_x and returns to gyro_val
  filter.filtre();

  pid.Compute();

  pr(gyro_x);
  pr("\t");
  pr(gyro_val);
  pr("\t");
  pln(pid_out);

  power = constrain(abs(pid_out), 0, 140);
  
  if (pid_out >= 0) {
    set_duty(ledChannelE1, constrain(LEFT_ACT_PRESET + power, 0, 511));
    set_duty(ledChannelE2, 0);
    set_duty(ledChannelD1, constrain(RIGHT_ACT_PRESET + power, 0, 511));
    set_duty(ledChannelD2, 0);

  } 
  else if (pid_out < 0) {
    set_duty(ledChannelE1, 0);
    set_duty(ledChannelE2, constrain(LEFT_ACT_PRESET + power, 0, 511));
    set_duty(ledChannelD1, 0);
    set_duty(ledChannelD2, constrain(RIGHT_ACT_PRESET + power, 0, 511));
  
  }
}
