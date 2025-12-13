#include "driver/ledc.h"
#include "esp_err.h"
#include <PID_v1.h>

#define pln Serial.println
#define pr Serial.print

// pwm level where motors start rotation
// right - ledChannelD1 and ledChannelD2
// left - ledChannelE1 and ledChannelE2
// these values need to be reconfigured for each new actuator
#define RIGHT_ACT_PRESET  370
#define LEFT_ACT_PRESET   370



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
extern void calibrateOffsets(uint16_t samples);

// time controll 
long    max_time_func = 10000;
long    initTime = millis();

// variables
double   gyro_x = 0.0;
double   pid_out = 0;
double   setpoint = 0.0;
int      power = 0;

// PID controll 
double Kp=3.0, Ki=0.0, Kd=0.0;
PID pid(&gyro_x, &pid_out, &setpoint, Kp, Ki, Kd, DIRECT);



void setup() {
  Serial.begin(115200);
  delay(100);
  setup_pwm_system();
  setup_mpu6050();
  calibrateOffsets(500);

  // setup pid controll
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-120, 120);
  pid.SetSampleTime(100);


  // set_duty(ledChannelD1, 0);
  // set_duty(ledChannelD2, 0);
  // Serial.println(-1);
  // Serial.println(1);
}

void loop() {

  // if (millis() - initTime < max_time_func) {
  // if (true) {    
    
    // return values between -120 and 120 on x axis
    gyro_x = (double) read_gyroscope_x();
    pid.Compute();

    pr(gyro_x);
    pr("\t");
    pln(pid_out);

    // Serial.print(1.5);
    // Serial.print(" ");
    // Serial.print(setpoint);
    // Serial.print(" ");
    // Serial.print(gyro_x);
    // Serial.print(-1.5);
    // Serial.print(" ");
    // pr(gyro_x*1000);
    // pr("\t");  
    // pr(map_pwm(abs(gyro_x)*1000, 0, 1000, 370, 511));
    // pln();
    // Serial.println();
    // delay(10);
  
    // set_duty(ledChannelD1, 370);
    // set_duty(ledChannelD2, 0);
    // set_duty(ledChannelE1, 511);
    // set_duty(ledChannelE2, 0);

    // power = map_pwm(abs(pid_out), 0, 120, 370, 511);
    // pr(gyro_x); pr("\t");
    // pr(pid_out); pr("\t");
    
    // if (pid_out > 0.00) {
    //   set_duty(ledChannelE1, power);
    //   set_duty(ledChannelE2, 0);
    //   set_duty(ledChannelD1, power);
    //   set_duty(ledChannelD2, 0);
    //   pr("f\t"); pln(power);
    // } 
    // else if (pid_out < -0.01) {
    //   set_duty(ledChannelE1, 0);
    //   set_duty(ledChannelE2, power);
    //   set_duty(ledChannelD1, 0);
    //   set_duty(ledChannelD2, power);
    //   pr("t\t"); pln(power);
    
    // }
    // else {
    //   set_duty(ledChannelE1, 0);
    //   set_duty(ledChannelE2, 0);
    //   set_duty(ledChannelD1, 0);
    //   set_duty(ledChannelD2, 0);
    //   pr("\telse ");
    //   pln(power);
    // }
     
  // }


  // else {
  //   // pln("Set tudo zero");
  //   set_duty(ledChannelE1, 0);
  //   set_duty(ledChannelE2, 0);
  //   set_duty(ledChannelD1, 0);
  //   set_duty(ledChannelD2, 0);
  // }

}


int map_pwm( int value, int ref_min, int ref_max, int res_min, int res_max ) {
  // if (value <= res_min) return res_min;
  if (value >= res_max) return res_max;

  return (value - ref_min) * (res_max - res_min) / (ref_max - ref_min) + res_min;

}