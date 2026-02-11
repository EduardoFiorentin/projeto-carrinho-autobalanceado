#include "driver/ledc.h"
#include <PID_v1.h>
#include "KalmanFilter.h"

#define pln Serial.println
#define pr Serial.print

#define RIGHT_ACT_PRESET  370
#define LEFT_ACT_PRESET   370

#define KALMAN_Q  0.0001
#define KALMAN_R  0.007 

// Time controll
const uint32_t CONTROL_PERIOD_MS = 10; // 100 Hz

// Limits
const int MAX_POWER = 140;
const int MAX_STEP  = 5;   // limite de variação por ciclo

// External declarations
extern void setup_pwm_system();
extern void set_duty(ledc_channel_t channel, uint32_t duty);

extern const ledc_channel_t ledChannelE1;
extern const ledc_channel_t ledChannelE2;
extern const ledc_channel_t ledChannelD1;
extern const ledc_channel_t ledChannelD2;

extern void setup_mpu6050();
extern float read_gyroscope_x();

// Variables
double gyro_x = 0.0;
double gyro_val = 0.0;

double pid_out = 0.0;
double setpoint = 0.0;

double Kp = 0.8, Ki = 0.0, Kd = 0.0; 

int power = 0;
int target_power = 0;

// PID + fiter
PID pid(&gyro_val, &pid_out, &setpoint, Kp, Ki, Kd, DIRECT);
KalmanFilter filter(&gyro_x, &gyro_val, KALMAN_Q, KALMAN_R);

// Time
uint32_t last_update = 0;

void setup() {
  Serial.begin(115200);
  delay(100);

  setup_pwm_system();
  setup_mpu6050();

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-MAX_POWER, MAX_POWER);
  pid.SetSampleTime(CONTROL_PERIOD_MS);
}


void loop() {
  uint32_t now = millis();
  if (now - last_update < CONTROL_PERIOD_MS) return;
  last_update = now;

  gyro_x = read_gyroscope_x();
  filter.filtre();

  pid.Compute();

  target_power = constrain(abs(pid_out), 0, MAX_POWER);

  int delta = target_power - power;
  delta = constrain(delta, -MAX_STEP, MAX_STEP);
  power += delta;

  Serial.printf("%d\t%d\t%d\t%lf\t%lf\n", target_power, power, delta, gyro_x, gyro_val);

  apply_motor_output(pid_out, power);
}


void apply_motor_output(double pid_out, int power) {

  if (abs(pid_out) <= 5.0) {
    set_duty(ledChannelE1, 0);
    set_duty(ledChannelE2, 0);
    set_duty(ledChannelD1, 0);
    set_duty(ledChannelD2, 0);
  }
  else if (pid_out >= 0) {
    set_duty(ledChannelE1, LEFT_ACT_PRESET  + power);
    set_duty(ledChannelE2, 0);
    set_duty(ledChannelD1, RIGHT_ACT_PRESET + power);
    set_duty(ledChannelD2, 0);
  } else {
    set_duty(ledChannelE1, 0);
    set_duty(ledChannelE2, LEFT_ACT_PRESET  + power);
    set_duty(ledChannelD1, 0);
    set_duty(ledChannelD2, RIGHT_ACT_PRESET + power);
  }
}