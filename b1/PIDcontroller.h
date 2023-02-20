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
  void speedpiout(double step0);
  void countpluse();
  void angle_calculate(int16_t ax, int16_t ay, int16_t az, int16_t gx,
                       int16_t gy, int16_t gz, float dt, float Q_angle,
                       float Q_gyro, float R_angle, float C_0, float K1);

  // PID parameter
  float pwm1 = 0, pwm2 = 0;

  // pulse count
  int pulseright, pulseleft;

  //
  int front = 0;  // forward variable
  int back = 0;   // backward

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
