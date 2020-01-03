/* config.h, b1 Mark I - Self Balancing Robot, Arduino project config file
 *
 * Copyright 2019-2020 Marcio Pessoa
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

// LED
const byte led_status_pin = 13;

// Temperature
const byte lm35_pin = 0;

// Random number generator Seed pin
const byte random_Seed_pin = lm35_pin;

// Timers
const byte health_check_timer = 2;  // seconds
const byte sensors_timer = 2;  // seconds
const byte standby_timer = 1;  // minute

// System status (initial state)
byte general_status = UNKNOWN;

// Debug mode
bool debug_mode = false;

// Serial speed: 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200
const unsigned long serial_speed = 115200;

// Move done
bool done = true;

//
const float FLIMIT = 340282350000000000000000000000000000000;
