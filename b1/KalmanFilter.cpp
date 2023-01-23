/* KalmanFilter.cpp, b1 Mark I - Self Balancing Robot, Kalman filter library
 *
 * Copyright 2019-2023 Marcio Pessoa
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

#include "Arduino.h"
#include "KalmanFilter.h"

/// @brief
/// @param dt sampling time period
/// @param q_angle covariance of gyroscope noise
/// @param q_gyro covariance of gyroscope drift noise
/// @param c_0
/// @param r_angle covariance of accelerometer
KalmanFilter::KalmanFilter(float dt, float q_angle, float q_gyro,
                           char c_0, float r_angle)
{
  _dt = dt;
  _q_angle = q_angle;
  _q_gyro = q_gyro;
  _c_0 = c_0;
  _r_angle = r_angle;
}

/// @brief
/// @param myAngle
/// @param angle_mRadial Radial rotation angle calculation formula ; negative sign is direction processing
/// @param gyro_m The X-axis angular velocity calculated by the gyroscope; the negative sign is the direction processing
void KalmanFilter::run(float myAngle, double angle_m, double gyro_m)
{
  angle = myAngle;
  angle += (gyro_m - _q_bias) * _dt; // prior estimate
  float angle_err = angle_m - angle;

  _pDot[0] = _q_angle - _p[0][1] - _p[1][0]; // The differential of the covariance of the prior estimate error
  _pDot[1] = -_p[1][1];
  _pDot[2] = -_p[1][1];
  _pDot[3] = _q_gyro;

  // The integral of the covariance differential of the prior estimate error
  _p[0][0] += _pDot[0] * _dt;
  _p[0][1] += _pDot[1] * _dt;
  _p[1][0] += _pDot[2] * _dt;
  _p[1][1] += _pDot[3] * _dt;

  // Intermediate variables in matrix multiplication
  float PCt_0 = _c_0 * _p[0][0];
  float PCt_1 = _c_0 * _p[1][0];

  // denominator
  float E = _r_angle + _c_0 * PCt_0;

  // gain value
  float K_0 = PCt_0 / E;
  float K_1 = PCt_1 / E;

  float t_0 = PCt_0; // Intermediate variables in matrix multiplication
  float t_1 = _c_0 * _p[0][1];

  _p[0][0] -= K_0 * t_0; // Posterior estimation error covariance
  _p[0][1] -= K_0 * t_1;
  _p[1][0] -= K_1 * t_0;
  _p[1][1] -= K_1 * t_1;

  _q_bias += K_1 * angle_err;     // Posterior estimate
  angle_speed = gyro_m - _q_bias; // The differential of the output value gives the optimal angular velocity
  angle += K_0 * angle_err;       // Posterior estimation; get the optimal angle
}
