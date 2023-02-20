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
  lz = count_left;  // assign the value counted by encoder to lz
  rz = count_right;

  count_left = 0;  // Clear count quantity
  count_right = 0;

  lpluse = lz;
  rpluse = rz;

  if ((pwm1 < 0) &&
      (pwm2 < 0))  // judge the car’s moving direction; if backward (PWM namely
                   // motor voltage is negative), pulse is a negative number.
  {
    rpluse = -rpluse;
    lpluse = -lpluse;
  }
  // if backward (PWM namely motor voltage is positive), pulse is a positive
  // number.
  else if ((pwm1 > 0) && (pwm2 > 0)) {
    rpluse = rpluse;
    lpluse = lpluse;
  }
  // judge the car’s moving direction; if turn left, right pulse is a positive
  // number; left pulse is a negative number.
  else if ((pwm1 < 0) && (pwm2 > 0)) {
    rpluse = rpluse;
    lpluse = -lpluse;
  }
  // judge the car’s moving direction; if turn right, right pulse is a negative
  // number; left pulse is a positive number.
  else if ((pwm1 > 0) && (pwm2 < 0)) {
    rpluse = -rpluse;
    lpluse = lpluse;
  }

  // enter interrupt per 5ms，pulse number plus
  pulseright += rpluse;
  pulseleft += lpluse;
}
