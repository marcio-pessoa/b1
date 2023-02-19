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
