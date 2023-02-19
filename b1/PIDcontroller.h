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

#define KP_SPEED 3.56   // speed loop parameter
#define KI_SPEED 0.072  // speed loop parameter
#define KD_SPEED 0      // speed loop parameter

class PIDcontroller {
 public:
  PIDcontroller();
  void speedpiout(double step0);
  int pulseright, pulseleft;
  float speeds_filterold = 0;
  float positions = 0;
  int front = 0;  // forward variable
  int back = 0;   // backward
  double PI_pwm;

 private:
};

#endif
