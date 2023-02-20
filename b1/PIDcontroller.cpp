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
void PIDcontroller::speedpiout(double step0) {
  float speeds = (pulseleft + pulseright) * 1.0;  // speed  pulse value

  pulseright = pulseleft = 0;  // clear
  speeds_filterold *= 0.7;     // first-order complementary filtering

  float speeds_filter = speeds_filterold + speeds * 0.3;
  speeds_filterold = speeds_filter;

  positions += speeds_filter;
  positions += front;  // Forward control fusion

  positions += back;                              // backward control fusion
  positions = constrain(positions, -3550, 3550);  // Anti-integral saturation

  // speed loop control PI
  PI_pwm = KI_SPEED * (step0 - positions) + KI_SPEED * (step0 - speeds_filter);
}

/// @brief pulse count
void PIDcontroller::countpluse() {
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
  pulseright += _rpluse;
  pulseleft += _lpluse;
}

/// @brief Tilt calculation.
/// @param ax
/// @param ay
/// @param az
/// @param gx
/// @param gy
/// @param gz
void PIDcontroller::angle_calculate(int16_t ax, int16_t ay, int16_t az,
                                    int16_t gx, int16_t gy, int16_t gz) {
  // Radial rotation angle calculation formula; negative sign is direction
  // processing
  float sensorAngle = -atan2(ay, az) * (180 / PI);

  // The X-axis angular velocity calculated by the gyroscope;  the negative sign
  // is the direction processing
  float Gyro_x = -gx / 131;

  kalman.run(angle, sensorAngle, Gyro_x);
  angle = kalman.angle;
  angle_speed = kalman.angle_speed;

  // calculate the inclined angle with x-axis
  float angleAx = -atan2(ax, az) * (180 / PI);

  float Gyro_y = -gy / 131.00;  // angle speed of Y-axis

  // first-order filtering
  angleY_one = K1 * angleAx + (1 - K1) * (angleY_one + Gyro_y * deltaTime);

  // rotating angle Z-axis parameter
  Gyro_z = -gz / 131;  // angle speed of Z-axis
}
