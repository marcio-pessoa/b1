/* HBridge.cpp, HBridge - HBridge, H-bridge library
 *
 * Copyright 2023-2023 Marcio Pessoa
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

#include "HBridge.h"
#include "Arduino.h"

/// @brief HBridge, H-bridge library
/// @param pin1
/// @param pin2
/// @param pwm_pin
HBridge::HBridge(byte pin1, byte pin2, byte pwm_pin) {
  _pin1 = pin1;
  _pin2 = pin2;
  _pwm_pin = pwm_pin;

  pinMode(_pin1, OUTPUT);
  pinMode(_pin2, OUTPUT);
  pinMode(_pwm_pin, OUTPUT);
}

/// @brief
/// @param pwm
void HBridge::forward(int pwm) {
  digitalWrite(_pin1, LOW);
  digitalWrite(_pin2, HIGH);
  analogWrite(_pwm_pin, pwm);
}

/// @brief
/// @param pwm
void HBridge::backward(int pwm) {
  digitalWrite(_pin1, HIGH);
  digitalWrite(_pin2, LOW);
  analogWrite(_pwm_pin, -pwm);
}
