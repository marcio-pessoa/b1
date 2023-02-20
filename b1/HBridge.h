/* HBridge.h, HBridge - HBridge, H-bridge library
 *
 * Copyright 2023-2023 Marcio Pessoa
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

#ifndef HBridge_h
#define HBridge_h

#include "Arduino.h"

class HBridge {
 public:
  HBridge(byte pin1, byte pin2, byte pwm_pin);
  void forward(int pwm);
  void backward(int pwm);

 private:
  byte _pin1, _pin2, _pwm_pin;
};

#endif
