/* PIDcontroller.cpp, b1 Mark I - Self Balancing Robot, PID Controller library
 *
 * Copyright 2019-2023 Marcio Pessoa
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

#include "PIDcontroller.h"
#include "Arduino.h"

PIDcontroller::PIDcontroller() {}

/// @brief speed PI
/// @param step0 angle balance point
void PIDcontroller::speedPIout(double step0) {
  float speeds = (_pulseleft + _pulseright) * 1.0;  // speed  pulse value

  _pulseright = _pulseleft = 0;  // clear
  _speeds_filterold *= 0.7;      // first-order complementary filtering

  float speeds_filter = _speeds_filterold + speeds * 0.3;
  _speeds_filterold = speeds_filter;

  _positions += speeds_filter;
  _positions += front;  // Forward control fusion

  _positions += back;                               // backward control fusion
  _positions = constrain(_positions, -3550, 3550);  // Anti-integral saturation

  // speed loop control PI
  _PI_pwm =
      KI_SPEED * (step0 - _positions) + KI_SPEED * (step0 - speeds_filter);
}

/// @brief pulse count
void PIDcontroller::countPulse() {
  _lz = count_left;  // assign the value counted by encoder to _lz
  _rz = count_right;

  count_left = 0;  // Clear count quantity
  count_right = 0;

  _lpluse = _lz;
  _rpluse = _rz;

  if ((pwm1 < 0) &&
      (pwm2 < 0))  // judge the car’s moving direction; if backward (PWM namely
                   // motor voltage is negative), pulse is a negative number.
  {
    _rpluse = -_rpluse;
    _lpluse = -_lpluse;
  }
  // if backward (PWM namely motor voltage is positive), pulse is a positive
  // number.
  else if ((pwm1 > 0) && (pwm2 > 0)) {
    _rpluse = _rpluse;
    _lpluse = _lpluse;
  }
  // judge the car’s moving direction; if turn left, right pulse is a positive
  // number; left pulse is a negative number.
  else if ((pwm1 < 0) && (pwm2 > 0)) {
    _rpluse = _rpluse;
    _lpluse = -_lpluse;
  }
  // judge the car’s moving direction; if turn right, right pulse is a negative
  // number; left pulse is a positive number.
  else if ((pwm1 > 0) && (pwm2 < 0)) {
    _rpluse = -_rpluse;
    _lpluse = _lpluse;
  }

  // enter interrupt per 5ms，pulse number plus
  _pulseright += _rpluse;
  _pulseleft += _lpluse;
}

/// @brief Tilt calculation. Calculate angle filtered by Professor Kálmán
/// @param ax
/// @param ay
/// @param az
/// @param gx
/// @param gy
/// @param gz
void PIDcontroller::calculateAngle(int16_t ax, int16_t ay, int16_t az,
                                   int16_t gx, int16_t gy, int16_t gz) {
  countPulse();

  // Radial rotation angle calculation formula; negative sign is direction
  // processing
  float sensorAngle = -atan2(ay, az) * (180 / PI);

  // The X-axis angular velocity calculated by the gyroscope;  the negative sign
  // is the direction processing
  float Gyro_x = -gx / 131;

  _kalman.run(angle, sensorAngle, Gyro_x);
  angle = _kalman.angle;
  angle_speed = _kalman.angle_speed;

  // calculate the inclined angle with x-axis
  float angleAx = -atan2(ax, az) * (180 / PI);

  float Gyro_y = -gy / 131.00;  // angle speed of Y-axis

  // first-order filtering
  // angleY_one = _K1 * angleAx + (1 - _K1) * (angleY_one + Gyro_y *
  // _delta_time);

  // rotating angle Z-axis parameter
  _Gyro_z = -gz / 131;  // angle speed of Z-axis
}

/// @brief turning
void PIDcontroller::turnspin() {
  int flag = 0;  //
  float turnspeed = 0;
  float rotationratio = 0;

  if (left == 1 || right == 1) {
    if (flag ==
        0)  // judge the speed before rotate, to increase the flexibility
    {
      // current speed ; express in pulse
      turnspeed = (_pulseright + _pulseleft);
      flag = 1;
    }
    if (turnspeed < 0)  // speed absolute value
    {
      turnspeed = -turnspeed;
    }
    // if press left key or right key
    if (left == 1 || right == 1) {
      _turnmax = 3;   // max turning value
      _turnmin = -3;  // min turning value
    }
    rotationratio = 5 / turnspeed;  // speed setting value
    if (rotationratio < 0.5) {
      rotationratio = 0.5;
    }

    if (rotationratio > 5) {
      rotationratio = 5;
    }
  } else {
    rotationratio = 0.5;
    flag = 0;
    turnspeed = 0;
  }
  // plus according to direction parameter
  if (left == 1) {
    _turnout += rotationratio;
    // plus according to direction parameter
  } else if (right == 1) {
    _turnout -= rotationratio;
  } else
    _turnout = 0;
  if (_turnout > _turnmax) _turnout = _turnmax;  // max value of amplitude
  if (_turnout < _turnmin) _turnout = _turnmin;  // min value of amplitude

  // turning PD algorithm control
  _Turn_pwm = -_turnout * _kp_turn - _Gyro_z * _kd_turn;
}

void PIDcontroller::calculatePWM(float angle_default) {
  // angle loop PD control
  int PD_pwm = _kp * (angle + angle_default) + _kd * angle_speed;

  // assign the end value of PWM to motor
  pwm2 = -PD_pwm - _PI_pwm + _Turn_pwm;
  pwm1 = -PD_pwm - _PI_pwm - _Turn_pwm;

  // limit PWM value not greater than255
  if (pwm1 > 255) {
    pwm1 = 255;
  }
  if (pwm1 < -255) {
    pwm1 = -255;
  }
  if (pwm2 > 255) {
    pwm2 = 255;
  }
  if (pwm2 < -255) {
    pwm2 = -255;
  }

  // if tilt angle is greater than 45°，motor will stop
  if (angle > 45 || angle < -45) {
    pwm1 = pwm2 = 0;
  }
}

void PIDcontroller::speed() {
  _speed_counter++;
  // 5*8=40，enter PI algorithm of speed per 40ms
  if (_speed_counter >= _speed_counter_limit) {
    speedPIout();
    _speed_counter = 0;  // Clear
  }
}

void PIDcontroller::guidance() {
  _guidance_counter++;
  // 20ms; enter PD algorithm of steering
  if (_guidance_counter > _guidance_counter_limit) {
    turnspin();
    _guidance_counter = 0;  // Clear
  }
}

void PIDcontroller::run() {
  speed();
  guidance();
}
