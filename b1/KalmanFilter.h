/* KalmanFilter.h, b1 Mark I - Self Balancing Robot, Kalman filter library
 *
 * Copyright 2019-2023 Marcio Pessoa
 *
 * Reference: https://en.wikipedia.org/wiki/Kalman_filter
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

#ifndef KalmanFilter_h
#define KalmanFilter_h

#include "Arduino.h"

class KalmanFilter {
 public:
  KalmanFilter(float dt, float q_angle, float q_gyro, char c_0, float r_angle);
  void run(float myAngle, double angle_m, double gyro_m);
  float angle;
  float angle_speed;

 private:
  float _pDot[4] = {0, 0, 0, 0};
  float _p[2][2] = {{1, 0}, {0, 1}};
  float _q_bias;  // gyroscope drift
  float _delta_time;
  float _q_angle;
  float _q_gyro;
  float _r_angle;
  char _c_0;
};

#endif
