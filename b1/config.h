/* config.h, b1 Mark I - Self Balancing Robot, Arduino project config file
 *
 * Copyright 2019-2023 Marcio Pessoa
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

// LED
// const byte led_status_pin = 13;

// Temperature
// const byte lm35_pin = 0;

// Random number generator Seed pin
// const byte random_Seed_pin = lm35_pin;

// Timers
// const byte health_check_timer = 2;  // seconds
// const byte sensors_timer = 2;  // seconds
// const byte standby_timer = 1;  // minute

// System status (initial state)
// byte general_status = UNKNOWN;

// Debug mode
bool debug_mode = false;

// Serial speed: 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200
const unsigned long serial_speed = 9600;

// Move done
// bool done = true;

//
// const float FLIMIT = 340282350000000000000000000000000000000;

const int right_R1 = 8;
const int right_R2 = 12;

const int left_L1 = 7;
const int left_L2 = 6;

const int PWM_R = 10;
const int PWM_L = 9;

const int buz = 11;
const int btn = 13;

/////////////////////// angle parameters
float angle0 = 0;  // mechanical balance angle (ideally 0 degrees)
/////////////////////// angle parameter

////////////////////// PID parameter
const double kp = 34, ki = 0, kd = 0.62;  // angle loop parameter
const double kp_turn = 24, ki_turn = 0,
             kd_turn = 0.08;  // steering loop parameter

//////////////////////////////// PI variable parameter
int cc;

////////////////////////////// turning PD
int turnmax, turnmin, turnout;
float Turn_pwm = 0;
int turncc = 0;

int i, button;
