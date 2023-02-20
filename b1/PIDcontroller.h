/* PIDcontroller.h, b1 Mark I - Self Balancing Robot, PID Controller library
 *
 * Copyright 2019-2023 Marcio Pessoa
 *
 * Reference: https://en.wikipedia.org/wiki/Kalman_filter
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

#ifndef PIDcontroller_h
#define PIDcontroller_h

#include "Arduino.h"
#include "KalmanFilter.h"  // Kalman filter

#define KP_SPEED 3.56   // speed loop parameter
#define KI_SPEED 0.072  // speed loop parameter
#define KD_SPEED 0      // speed loop parameter

class PIDcontroller {
 public:
  PIDcontroller();
  void speedPIout(double step0 = 0);
  void countPulse();
  void angleCalculate(int16_t ax, int16_t ay, int16_t az, int16_t gx,
                      int16_t gy, int16_t gz);

  // PID parameter
  float pwm1 = 0, pwm2 = 0;

  // pulse count
  int pulseright, pulseleft;

  //
  int front = 0;  // forward variable
  int back = 0;   // backward
  int left = 0;   // turn left
  int right = 0;  // turn right

  // PI variable parameter
  float speeds_filterold = 0;
  float positions = 0;
  double PI_pwm;

  // interrupt speed count
  // Used to calculate the pulse value calculated by the Hall encoder
  volatile long count_right = 0;
  // (the volatile long type is to ensure the value is valid)
  volatile long count_left = 0;

  // Angular angular velocity by gyroscope calculation
  float Gyro_z;

  // Kalman filter
  float angle;
  float angle_speed;
  float angleY_one;

  const float Q_angle = 0.001;  // Covariance of gyroscope noise
  const float Q_gyro = 0.003;   // Covariance of gyroscope drift noise
  const float R_angle = 0.5;    // Covariance of accelerometer
  const char C_0 = 1;
  const float deltaTime = 0.005;  // The value of dt is the filter sampling time
  const float K1 = 0.05;  // a function containing the Kalman gain is used to
                          // calculate the deviation of the optimal estimate

  // Kalman filter
  KalmanFilter kalman;

 private:
  // pulse count
  int _lz = 0;
  int _rz = 0;
  int _rpluse = 0;
  int _lpluse = 0;
};

#endif
