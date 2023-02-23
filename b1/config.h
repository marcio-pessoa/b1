/* config.h, b1 Mark I - Self Balancing Robot, Arduino project config file
 *
 * Copyright 2019-2023 Marcio Pessoa
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

// Debug mode
bool debug_mode = false;

// Serial speed: 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200
const unsigned long serial_speed = 9600;

//
const float FLIMIT = 340282350000000000000000000000000000000;

// Motor left
const byte left_pin1 = 7;
const byte left_pin2 = 6;
const byte left_pwm_pin = 9;
const byte left_encoder_pin = 5;

// Motors right
const byte right_pin1 = 8;
const byte right_pin2 = 12;
const byte right_pwm_pin = 10;
const byte right_encoder_pin = 4;

// Buzzer
const byte buzzer_pin = 11;

// Calibration
const byte button_pin = 13;
