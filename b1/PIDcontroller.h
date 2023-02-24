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
  void calculateAngle(int16_t ax, int16_t ay, int16_t az, int16_t gx,
                      int16_t gy, int16_t gz);
  void turnspin();
  void calculatePWM(float angle_default = 0);
  void speed();
  void guidance();
  void run();

  // PID parameter
  float pwm1 = 0, pwm2 = 0;

  //
  int front = 0;  // forward variable
  int back = 0;   // backward
  int left = 0;   // turn left
  int right = 0;  // turn right

  // interrupt speed count
  // Used to calculate the pulse value calculated by the Hall encoder
  volatile long count_right = 0;
  // (the volatile long type is to ensure the value is valid)
  volatile long count_left = 0;

  // Kalman filter
  float angle;
  float angle_speed;

 private:
  // Kalman filter
  const float _Q_angle = 0.001;  // Covariance of gyroscope noise
  const float _Q_gyro = 0.003;   // Covariance of gyroscope drift noise
  const float _R_angle = 0.5;    // Covariance of accelerometer
  const char _C_0 = 1;
  const float _delta_time =
      0.005;               // The value of dt is the filter sampling time
  const float _K1 = 0.05;  // a function containing the Kalman gain is used to
                           // calculate the deviation of the optimal estimate
  // float angleY_one;

  KalmanFilter _kalman =
      KalmanFilter(_delta_time, _Q_angle, _Q_gyro, _C_0, _R_angle);

  // PID parameter
  // steering loop parameter
  const double _kp_turn = 24, _ki_turn = 0, _kd_turn = 0.08;
  // angle loop parameter
  const double _kp = 34, _ki = 0, _kd = 0.62;

  // PI variable parameter
  double _PI_pwm;
  float _speeds_filterold = 0;
  float _positions = 0;

  // pulse count
  int _lz = 0;
  int _rz = 0;
  int _rpluse = 0;
  int _lpluse = 0;

  // Angular angular velocity by gyroscope calculation
  float _Gyro_z;

  // turning PD
  float _Turn_pwm = 0;
  int _turnmax, _turnmin, _turnout;

  // pulse count
  int _pulseright, _pulseleft;

  // PI (Proportional Integral)
  int _speed_counter;
  const int _speed_counter_limit = 8;

  // PD (Proportional Derivative)
  int _guidance_counter = 0;
  const int _guidance_counter_limit = 4;
};

#endif
